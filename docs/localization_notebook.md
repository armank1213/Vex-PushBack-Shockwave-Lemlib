# Localization — Engineering Notebook Pages

These pages document the robot's two localization filters for the engineering
notebook. They are meant to follow the existing **Page 1** (the problem: odometry
drift over a long autonomous routine).

Order:
- **Page 2** — EKF, plain-English (for any judge)
- **Page 2A** — EKF, the mathematics (for a technical reader)
- **Page 3** — MCL, plain-English (for any judge)
- **Page 3A** — MCL, the mathematics (for a technical reader)
- **Diagram descriptions** — hand-off notes for whoever illustrates the pages

Both filters share the exact same field map and run about 50 times per second.
**MCL is the filter we actually run in autonomous**; the EKF was built as a
lightweight comparison. Every number and equation in the technical pages matches
the code in `src/oekf_rerun.cpp`, `src/mcl_rerun.cpp`, and `src/localization.cpp`.

---

# Page 2 — Approach #1: The Extended Kalman Filter (EKF)

### The idea in one sentence
The EKF keeps **one single best guess** of where the robot is, and constantly nudges that guess using the distance sensors so it stays honest.

### How it works (the simple version)
Think of the EKF as one detective with a single theory of the robot's location. Every fraction of a second it does two things:

1. **Predict.** The robot's tracking wheels and gyro say "you just moved forward 3 inches." The detective slides the guess forward by 3 inches. But it knows wheels slip, so it also becomes a little *less sure* — it widens the "I'm somewhere around here" bubble around the guess.

2. **Correct.** The four distance sensors measure how far the robot is from the walls. The detective compares what the sensors *should* read if the guess were true (calculated from a built-in map of the field) against what they *actually* read. If there's a difference, it slides the guess toward the position that better explains the sensors, and shrinks the uncertainty bubble back down.

Over and over — predict, correct, predict, correct — the single guess is dragged back onto the truth every time it starts to drift.

### Why the uncertainty bubble matters
The EKF doesn't just track *where* it thinks the robot is; it tracks *how confident* it is, as a bubble around the guess. When it's unsure, it leans heavily on the sensors. When it's confident, it trusts itself and ignores small sensor noise. This is what makes the correction smooth instead of jumpy.

### A safety feature: ignoring the goals
The field has big structures — the two long goals, the center goal, and the matchload tubes — that a distance sensor can mistake for a wall. Our map knows where those are. If a sensor beam is pointed at a goal instead of a wall, or its reading is wildly different from what a wall should give (off by more than ~14 inches), the EKF **throws that reading out** instead of trusting it. The same trick handles loose blocks and other robots.

### The trade-off
The EKF is extremely **fast and cheap** — it only tracks one guess, so it barely uses any of the robot's computing power. Its weakness is that it can only believe in *one* possibility at a time. If the sensor data is confusing and the robot could plausibly be in two different spots, the EKF has to pick one. If it picks wrong, it can get stuck on a bad answer. That limitation is exactly why we built a second approach.

---

# Page 2A — The EKF in Detail (the mathematics)

*For the technical reader. This expands the plain-English Page 2.*

### State and uncertainty
The filter tracks a 3-element **state vector** and a 3×3 **covariance matrix** that encodes how uncertain it is:

```
x = [ x ,  y ,  θ ]ᵀ        P (3×3, units in², in², rad²)
```

`x, y` are field-absolute inches; `θ` is heading in radians. `P` is seeded fairly uncertain at startup — diagonal **(10, 10, 0.3)** — and shrinks as readings come in.

### Predict step
Odometry (tracking wheels + gyro) gives the motion since the last tick as a world-frame delta `u = [Δx, Δy, Δθ]`. Because that delta is already in the state's own coordinates, the motion Jacobian is the identity, **F = I**, so the predict step is just:

```
x⁻ = x + u
P⁻ = F P Fᵀ + Q   →   P⁻ = P + Q
```

`Q` is the per-tick **process noise** that models how much the wheels could have lied: diagonal **(0.5, 0.3, 0.005)**. Adding `Q` every tick is what grows the uncertainty bubble between corrections.

### Measurement model
Each distance sensor produces a scalar reading `z` (millimetres → inches). The predicted reading is a **nonlinear** function `h(x)`: place the sensor at its true on-robot position for the current pose, then ray-cast to the nearest field feature (`expected_range → raycast_map`). The same map is shared with MCL, so the two filters agree exactly.

Because `h` is nonlinear, the EKF **linearizes** it with a Jacobian `H = ∂h/∂x`, computed numerically by finite differences (step `ε = 1e-4`):

```
H[0] = (h(x+ε, y,   θ  ) − h(x,y,θ)) / ε
H[1] = (h(x,   y+ε, θ  ) − h(x,y,θ)) / ε
H[2] = (h(x,   y,   θ+ε) − h(x,y,θ)) / ε
```

### Update step
With measurement noise variance **R = 3 in²** per sensor:

```
innovation:        ν = z − h(x)
innovation cov.:   S = H P Hᵀ + R           (scalar, since z is 1-D)
Kalman gain:       K = P Hᵀ / S             (3×1)
state update:      x⁺ = x + K · ν
covariance update: P⁺ = (I − K H) P
```

The gain `K` is the heart of it: when `P` is large (unsure) `K` is large and the reading moves the guess a lot; when `P` is small (confident) `K` is small and noisy readings are largely ignored. All four sensors are fused one after another each tick.

### Gating (rejecting bad readings)
Before a reading is fused it must survive three gates, or it is discarded:
1. **Range gate** — raw distance must be 20–2000 mm (the sensor's valid window; 9999 means "nothing seen").
2. **Obstacle gate** — if the ray-cast says the beam hits a mapped goal/matchload rather than a wall, skip it (this also avoids a meaningless Jacobian across an obstacle edge).
3. **Innovation gate** — if `|ν| > 14 in`, the beam is hitting something unmapped (a loose block, another robot); reject it.

### Heading handling and write-back
Although `θ` is part of the state, each cycle the heading is **re-anchored to the gyro** before the wall corrections, so the sensors effectively only refine `x` and `y` — the gyro is far more accurate at angle than four beams on flat walls. The corrected pose is only pushed back into the robot's odometry once the filter is confident (`P[0][0] < 0.5` and `P[1][1] < 0.5`).

### Cost
The state is only 3-D, so every matrix op is trivial — the EKF is **O(1)** for our purposes and essentially free on the V5 brain. Its limitation is structural: a single Gaussian can represent only **one** hypothesis. If the evidence is genuinely ambiguous, it must commit to one peak — the reason MCL exists.

---

# Page 3 — Approach #2: Monte Carlo Localization (MCL), and how it fixes autonomous

### The problem we're solving (recap)
As explained on Page 1, the robot's wheels and gyro slowly lose track of the true position during a long autonomous routine. By the end, the robot can *think* it's in one place while it's actually inches away — so it scores into the wrong spot. We need something that constantly re-anchors the robot to the real field.

### The idea in one sentence
Instead of one guess, MCL keeps **500 guesses at once** and lets the distance sensors vote on which ones are right — so the robot's belief is decided by a crowd, not a single opinion.

### How it works (the simple version)
Picture 500 dots scattered on the field, each one a guess of where the robot might be. Every fraction of a second:

1. **Move the guesses.** When the robot drives forward, all 500 dots slide forward too — each with a tiny bit of random variation, because no two could be affected by wheel slip identically.

2. **Vote with the sensors.** For every dot, MCL asks: "If the robot were *here*, what would the four distance sensors read?" The dots whose predictions best match the *actual* sensor readings get a high score; the ones that don't match get a low score.

3. **Survival of the fittest.** MCL periodically "resamples" — it keeps and copies the high-scoring dots and drops the low-scoring ones. The cloud of dots naturally crowds together around the spot that best explains everything the sensors see. The robot's reported position is the center of that crowd.

Because it's a whole crowd, MCL can hold **several possibilities at the same time** and let the evidence sort them out — something the single-guess EKF can't do. That makes it much harder to fool and quicker to recover if a reading is briefly bad.

### How this fixes autonomous
MCL runs quietly in the background while our normal driving commands execute. At chosen checkpoints — when the robot is stopped at a waypoint near a wall — we call a function (`snapPose`) that lets the crowd settle, confirms it's confident, and then **snaps the robot's recorded position to the true value the sensors found**. All the drift that built up since the last checkpoint is erased in one step. The next move then starts from a correct position, so it lands where we actually intended.

Crucially, this only fixes the robot's **position** (left/right and forward/back). The **heading** (which way it's facing) is left to the gyro, which is far more accurate at angles than four sensors aimed at flat walls could ever be. Mixing the two up was an early mistake that made the robot turn incorrectly; keeping them separate is a deliberate design decision.

### Built-in robustness
- **It ignores the goals and obstacles.** Just like the EKF, MCL uses our field map to recognize when a beam is hitting a long goal, the center goal, a matchload tube, a loose block, or another robot — and refuses to treat those as walls.
- **It survives a bad frame.** If one noisy sensor reading would make *every* guess look wrong, MCL doesn't panic and re-scatter across the field (which would freeze it on a wrong answer). It simply ignores that one bad update and trusts the crowd it already had.

### The trade-off
MCL is far more **robust and accurate** than the EKF, which is why it's the version we run in competition. The cost is computing power: it does the sensor calculation for all 500 guesses, ~50 times a second. The VEX brain handles this comfortably, so for us the extra reliability is well worth it.

---

# Page 3A — MCL in Detail (the mathematics)

*For the technical reader. This expands the plain-English Page 3.*

### Representing belief as samples
MCL approximates the position belief — a probability distribution over the whole field — with a set of **N = 500 weighted particles**:

```
particle i :  { xⁱ, yⁱ, θⁱ, wⁱ }      Σ wⁱ = 1
```

There is no Gaussian assumption: the cloud of particles can take any shape, including several separate clusters (multiple hypotheses) at once.

### 1 — Motion model (predict)
Odometry gives body-frame deltas: `d_forward`, `d_lateral` (inches) and `Δθ` (rad). Following Thrun's odometry motion model, the per-tick noise standard deviations grow with the size of the motion, scaled by four tuning coefficients **α = {0.05, 0.02, 0.02, 0.05}**:

```
t = √(d_forward² + d_lateral²)        (translation magnitude)
σ_rot   = √(α₁·|Δθ| + α₂·t)
σ_trans = √(α₃·t   + α₄·|Δθ|)
```

Each particle is moved by a **noisy** copy of the motion, then rotated into the world by **its own** heading (θ = 0 faces +Y, CW positive ⇒ forward = (sin θ, cos θ), lateral = (cos θ, −sin θ)):

```
df = d_forward + N(0, σ_trans)
dl = d_lateral + N(0, σ_trans)
Δx = df·sin θⁱ + dl·cos θⁱ
Δy = df·cos θⁱ − dl·sin θⁱ
```

Then the heading of every particle is **pinned to the gyro**, `θⁱ = θ_gyro + N(0, 1°)`, so the wall-matching below uses the trusted heading rather than a drifting per-particle angle.

### 2 — Measurement update (weighting)
For each sensor reading `z`, every particle predicts what that sensor *should* read by ray-casting from its hypothesized pose (`raycast_map`). The reading is scored with a Gaussian likelihood (measurement variance **R = 3 in²**):

```
νⁱ = z − ẑⁱ
p(z | particleⁱ) ∝ exp( −νⁱ² / (2R) )
```

Weights are updated multiplicatively, `wⁱ ← wⁱ · p(z | particleⁱ)`, but the work is done in **log-space with the log-sum-exp trick** (subtract the max log-weight before exponentiating) to stay numerically stable:

```
log wⁱ = log(wⁱ) − νⁱ² / (2R)
```

A beam that hits a mapped obstacle, or grazes a wall at more than **30°** off-normal, contributes `0` (uninformative) instead of penalizing the particle. A whole reading is rejected up front if, at the current mean pose, it disagrees with the expected wall by more than **12 in** (obstacle rejection). Weights are then normalized to sum to 1.

### 3 — Resampling (survival of the fittest)
The estimate quality is monitored with the **effective sample size**:

```
N_eff = 1 / Σ (wⁱ)²
```

When `N_eff` drops below **N/2 = 250** (the cloud is dominated by a few heavy particles), MCL runs a **systematic / low-variance resampler**: draw a single offset `r ∈ [0, 1/N)` and step through the cumulative weight with a comb of N evenly spaced pointers, copying high-weight particles and dropping low-weight ones. To prevent the cloud from collapsing to a single point (which would make the variance meaningless), each survivor is **roughened** with small jitter — `N(0, 0.75 in)` on x and y, `N(0, 1.5°)` on θ.

### 4 — Extracting the estimate
The reported pose is the weighted mean of the cloud; heading uses a **circular mean** so it wraps correctly:

```
x̄ = Σ wⁱ xⁱ ,   ȳ = Σ wⁱ yⁱ
θ̄ = atan2( Σ wⁱ sin θⁱ , Σ wⁱ cos θⁱ )
var_xy = Σ wⁱ · ( (xⁱ−x̄)² + (yⁱ−ȳ)² )      (in², the confidence metric)
```

### Robustness detail: surviving a bad frame
If a single noisy reading makes **every** particle's likelihood underflow (total weight ≈ 0), MCL does **not** re-scatter across the field — that would discard a good belief and can freeze the estimate on a wrong answer. Instead it keeps all particle positions and resets the weights to uniform, i.e. it ignores that one bad update.

### How it drives autonomous (the production path)
MCL runs as a background task at **~50 Hz** (20 ms loop). At a stopped waypoint we call `snapPose()`, which lets the cloud settle and then accepts the estimate only if it is confident — **var_xy < 4 in²** *and* **N_eff > N/2**. If so, it hard-sets the robot's recorded `x, y` to the filter's value (heading left to the gyro) and re-seeds the cloud there, erasing all accumulated drift in one step. (A continuous variant also exists: a partial nudge of strength `trust = clamp(1 − var_xy/4, 0, 1) · 0.35` toward the estimate every tick.)

### Cost vs. benefit
MCL is **O(N × sensors)** per tick — 500 ray-casts × 4 sensors, 50 times a second — versus the EKF's single evaluation. The V5 brain handles this comfortably, and in exchange MCL represents arbitrary, multi-hypothesis beliefs and recovers gracefully from bad data. That robustness is why MCL is the filter we actually run in competition, with the EKF kept as a lightweight comparison.

---

# Diagram descriptions (for the illustrator)

These are notes to hand to whoever draws the notebook figures. Nothing here is
code; each box describes one labeled illustration.

### Figure A — EKF: the uncertainty bubble shrinking (goes on Page 2 / 2A)
A horizontal strip showing the **same robot pose three times**, left to right, with a curved arrow labeled "predict → correct" between each step.

- **Panel 1 (start / after predict):** a robot icon with a **large dashed ellipse** drawn around its center — wide and faint. Label the ellipse "uncertainty `P` (large = unsure)." Draw the field's left and bottom walls nearby with a thin beam line from each side sensor toward a wall, labeled "distance sensor reading `z`."
- **Panel 2 (during correct):** the same robot, but draw a short **arrow nudging the robot center** toward the position the sensors imply, labeled "correction `K·ν`." The ellipse is now **medium-sized** and less dashed.
- **Panel 3 (after correct):** the robot sitting on the true spot with a **small tight ellipse**, labeled "uncertainty `P` (small = confident)." Add a caption strip under all three: *"Each predict step grows the ellipse (adds Q); each sensor correction shrinks it. One guess, always pulled back to truth."*

Key visual point: **one** robot/ellipse the whole time (single hypothesis), the ellipse breathing larger then smaller — contrast this with Figure B's crowd.

### Figure B — MCL: the particle cloud converging + resampling comb (goes on Page 3 / 3A)
Two stacked illustrations sharing the same field outline (draw the field as a square with the left/bottom walls emphasized, plus one obstacle rectangle labeled "long goal — ignored").

- **B1 — convergence (3 mini-frames, left to right):**
  - *Frame 1:* ~500 small dots **scattered widely** across the field, label "t = 0: belief spread out (500 particles)."
  - *Frame 2:* the dots **clumping toward two loose blobs**, with the blob nearer the walls drawn denser/darker, label "sensors vote — good guesses score higher."
  - *Frame 3:* a **single tight cluster** on the true pose with a small **+** at its center labeled "reported pose = weighted mean `(x̄, ȳ)`"; annotate the cluster width "`var_xy` (confidence)."
- **B2 — the systematic resampling comb (one detail figure):** a **horizontal bar from 0 to 1** representing cumulative weight, divided into uneven segments — wide segments labeled "high-weight particle (copied several times)" and thin slivers labeled "low-weight (dropped)." Below the bar draw **N evenly spaced upward arrows (a comb)**, the first one offset slightly from 0, labeled "single random start `r ∈ [0, 1/N)`, then equal steps `1/N`." Caption: *"Evenly spaced pointers land more often inside wide (heavy) segments, so good particles survive and multiply — low variance, no full re-scatter."* Optionally add a tiny "+jitter" wiggle on the copied dots labeled "roughening (0.75 in / 1.5°)."

Key visual point: a **crowd** of dots (many hypotheses) collapsing onto one answer, versus the EKF's single ellipse. The comb figure explains *how* survival-of-the-fittest is done fairly.
