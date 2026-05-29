# CLAUDE.md — pushback_shockwave

MHHS Robotics VEX V5 competition robot (VEX Pushback season). PROS + LemLib.
This file is the handoff summary for a fresh Claude session. Read it first.

---

## 1. Build & toolchain (IMPORTANT on Ubuntu/Pop!_OS)

- Build with `make` or `pros make`. Output: `bin/hot.package.bin`. Upload with `pros upload` / `pros mu`.
- **The project requires `gnu++26`** (`common.mk:24` → `CXX_STANDARD?=gnu++26`). The flag name `--std=gnu++26` is only recognized by **GCC 14+**.
- Ubuntu's apt `gcc-arm-none-eabi` is **GCC 10.3** (gnu++20 max) → build fails with `unrecognized command-line option '--std=gnu++26'`.
- **Fix already applied on this machine:** ARM GNU Toolchain 14.2.rel1 (GCC 14.2.1) installed at
  `~/.local/share/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi`, and `~/.bashrc` exports:
  ```bash
  export PROS_TOOLCHAIN="$HOME/.local/share/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi"
  export PATH="$PROS_TOOLCHAIN/bin:$PATH"
  ```
  New terminals pick it up automatically; existing shells need `source ~/.bashrc`. If a sandbox build can't find these, export them for the build command.
- The old apt GCC 10 is still installed but shadowed.

### Uploading on Ubuntu ("device NACKD" errors)
Usually USB permissions, not code. Add user to dialout (`sudo usermod -aG dialout $USER`, then re-login) and add a udev rule for VEX (vendor `2888`). Mac doesn't hit this because USB is user-accessible by default.

---

## 2. Hardware / ports (`src/hardware.cpp`)

- **Left drive:** ports −19, −13, −17 (blue gearset). **Right drive:** 12, 18, 14 (blue).
- **Intake** 15 (green), **Middle** 7 (blue), **Outtake** 10 (green).
- **IMU** port 3. **Vertical rotation** −11. **Horizontal rotation** 20.
- **Optical (color sort)** 8. **Distance (color sort)** 2.
- **Perimeter distance sensors:** `ldist_sens` 9, `rdist_sens` 4, `fdist_sens` 5, `bdist_sens` 6.
- **Pneumatics (ADI):** matchLoad `E`, limiter `A`, wing `C`.
- Known conflict: port 6 wanted by a `limiter_light` optical but used by `bdist_sens`; `limiter_light` is commented out.

### Driver control (`src/motors.cpp`, `src/pneumatics.cpp`)
Arcade. R1 = score (intake+middle+outtake, limiter up). R2 = intake, limiter down. Y / B = reverse-outtake variants. L2 = reverse all. L1 = wing toggle. RIGHT = matchload toggle. UP = bench localization-test toggle. X = snap field pose from sensors + record route to `/usd/dtData.txt`.

---

## 3. Localization — architecture & math (the bulk of recent work)

Two filters fuse wheel odom with the **4 perimeter distance sensors** to estimate field-absolute pose. Field frame: inches, **(0,0) = red-side bottom-left**, +X right, +Y away from red driver. Heading θ: **0 = +Y, CW positive** (LemLib convention); forward unit vector = `(sin θ, cos θ)`.

### Files
- `include/robot/field_model.hpp` — **THE one place to tune.** Field size, sensor mounts, raycast, grazing-angle, sensor-world helpers. Namespace `field`.
- `src/mcl_rerun.cpp` / `include/robot/mcl_rerun.hpp` — Monte Carlo (particle) filter, `N=500`. Also the SD-route replay (`mcl::rerun`).
- `src/oekf_rerun.cpp` / `include/robot/oekf_rerun.hpp` — 3-state EKF `[x,y,θ]`. Also EKF route replay.
- `src/localization.cpp` / `include/robot/localization.hpp` — `loc::` background-task wrapper used by autons/tests. `loc::start(x,y,deg,Method,correct)`, `loc::stop()`, `loc::estimate()`.
- `src/start_pose.cpp` / `include/robot/start_pose.hpp` — `determine_start_pose(theta_deg)`: infers field pose from the walls (cardinal-snap, opposite-wall averaging).

### `loc::` usage
```cpp
loc::start(fieldX, fieldY, fieldDeg);              // MCL, correction ON (production)
chassis.moveToPoint(...); chassis.turnToHeading(...);
loc::stop();                                       // ALWAYS before the auton returns
// correction OFF (testing) — only estimates, never touches chassis pose:
loc::start(x, y, deg, loc::Method::MCL, /*correct=*/false);
```
`loc::start` calls `stop()` first if already running (clean re-seed each run). It calls `chassis.setPose(x,y,deg)`, so it seeds odom AND the filter together.

### KEY DESIGN DECISIONS (do not regress these)
1. **Heading comes from the IMU, not the filter.** Each tick, particle headings are pinned to the IMU (`mcl::set_heading(filter, imu_rad, ~1°)`) and the EKF sets `ekf.theta = imu_rad`. The filters only solve x,y. Letting heading free-run caused wrong wall-association → divergence / "huge errors". This is standard MCL-with-known-orientation on a rectangular map.
2. **True 2-D sensor model.** Each sensor has a measured body-frame **face position** `(bx, by)` and pointing `dir`. The ray is cast from the sensor's actual world position (`field::sensor_world` → `field::raycast`); `measured = raw_mm/25.4` with **no center offset added**. Exact at any heading. (Old code used a single center-offset scalar, only correct when square to the wall.)
3. **Obstacle rejection.** A beam whose reading differs from the expected wall distance by more than a gate is dropped (MCL `OBSTACLE_REJECT_IN = 12"`; EKF `INNOV_GATE_IN = 14"`). Handles long goal / game pieces / other robots WITHOUT mapping them. NOTE: obstacles are not yet map landmarks — to localize *off* the goals, add their rectangles to `field::raycast` (needs exact coordinates from the user; don't guess them).
4. **Field size `FIELD_IN = 140.43`** (interior wall-to-wall, NOT the 144" outer). Canonical in `field_model.hpp`; `oekf::FIELD_IN` aliases it.
5. **Theta deltas are wrap-normalized** to [−180,180] before use (in localization.cpp, mcl_rerun.cpp, oekf_rerun.cpp). Skipping this rotated the whole cloud on heading wrap.
6. **MCL collapse handling:** if all particle weights collapse on a bad frame, keep particle positions and reset weights uniform (ignore the bad update) — do NOT re-scatter across the whole field (that froze the estimate at a wrong value).

### VEX V5 Distance sensor facts (verified: kb.vex.com, SIGBots wiki, PROS header)
- `get()` / `get_distance()` return **mm** (identical). Return **9999 when no object**, `PROS_ERR` on fault.
- Range **20–2000 mm**. Accuracy ±15 mm below 200 mm, ~5% above.
- `get_confidence()` 0–63, **only meaningful when distance > 200 mm** (below that it reads low/0 but distance is still good).
- **Validity gate used everywhere:** in range 20–2000 AND (raw ≤ 200 → accept; else confidence ≥ 40). The old code gated confidence at all ranges, silently discarding any wall closer than ~8".
- **"Overheating" is a myth here:** Distance sensors have no thermal throttle (only Smart Motors do). Past "sensor stops working" symptoms were the filter freezing + the gate/range bugs, now fixed.

---

## 4. Tuning the sensor mounts (`field_model.hpp`)

Each `SensorMount{ bx, by, dir }` is measured on the real robot from the **tracking center** to the sensor **lens**:
- `bx` = inches to the RIGHT (+) / LEFT (−) of center.
- `by` = inches in FRONT (+) / BEHIND (−) of center.
- `dir` = which way it points, relative to forward, radians: front `0`, right `PI/2`, back `PI`, left `-PI/2`. **Not measured — just which axis it faces.** Leave at the cardinal unless a sensor is physically aimed off-axis (keep beams ~perpendicular to walls; the grazing gate rejects > 30° off-normal).

Per sensor, the **along-beam** component you already roughly know; the **perpendicular** component is the one to measure carefully:
- FRONT/BACK: along = `by`, perpendicular = `bx` (left/right of the fore-aft centerline).
- LEFT/RIGHT: along = `bx`, perpendicular = `by` (fore/aft of the left-right axis through center).

**Find the tracking center:** horizontal tracking wheel sits 5.5" behind it, vertical wheel 1.5" to the side (from `chassis_config.cpp` offsets); their crossing lines = center. Sanity check: spin robot in place — the non-orbiting point should be that mark.

**Verify:** seed square in a corner (est should read true coord within ~½"), then rotate in place ~30–45°. If `est` drifts during rotation, a perpendicular offset (or its sign) is wrong — nudge ±0.5" / try sign swap and re-test. Get this right and you should not need to re-tune `FIELD_IN`.

---

## 5. Autons (`src/autonomous.cpp`)

`autonomous()` (in `src/main.cpp`) currently calls **`localization_test()`** (MCL, correction OFF). Other options commented out: `oekf_rerun()`, `mcl_rerun()`, `corrected_auton()`, `skills_auton()`.

- `localization_test()` — seeds pose from `determine_start_pose(TDEG)` (odom + filter start at the same sensor-measured coord), then prints od/est/var on the controller so you can compare. Place robot near a corner, square-ish, at heading `TDEG`.
- `left_auton`, `park_auton`, `skills_auton` — hand-coded routines (relative coords from setPose(0,0,0)).
- `corrected_auton` — example of MCL correction ON with field-absolute waypoints.
- `right_auton` — stub.

Route recording/replay: press X in opcontrol to snap pose + log to `/usd/dtData.txt`; replay via `mcl_rerun()` / `oekf_rerun()`. Embedded-route option exists (`USE_EMBEDDED_ROUTE` in mcl_rerun.cpp, generated by `tools/embed_route.sh`).

---

## 6. Working agreements / context

- User is a high-school team programmer; develops on Ubuntu (primary) and Mac. Casual style; wants concise, correct answers and **no hallucinated facts** — verify against the PROS headers / VEX docs before claiming sensor behavior or field dimensions.
- Coordinates for localization must be **field-absolute inches**; `loc::stop()` must run before an auton returns.
- When changing localization, keep the 6 design decisions in §3 intact and rebuild to verify (full clean build links `bin/hot.package.bin`).
