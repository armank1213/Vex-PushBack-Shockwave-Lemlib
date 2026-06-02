#include "robot/oekf_rerun.hpp"
#include "robot/chassis_config.hpp"
#include "robot/hardware.hpp"
#include "robot/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace oekf {

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr const char* DATA_PATH = "/usd/dtData.txt";

// Sensor mount geometry lives in robot/field_model.hpp.

// Replay tuning.
constexpr int    WAYPOINT_SKIP      = 8;
constexpr int    MIN_SPEED          = 70;
constexpr int    MAX_SPEED          = 110;
constexpr float  EARLY_EXIT_MAX     = 6.0f;
constexpr double TURN_DETECT_DEG    = 25.0;
constexpr double TURN_RATIO_THRESH  = 8.0;
constexpr double SHORT_MOVE_IN      = 4.0;
constexpr double TURN_THRESHOLD_DEG = 15.0;
constexpr int    TURN_TIMEOUT_MS    = 500;

// Per-segment timeout = recorded ms * multiplier. 1.6 = 60% headroom.
constexpr double TIMEOUT_MULTIPLIER = 1.6;

// Measured top speed at full power. Tune empirically.
constexpr double MAX_SPEED_IPS = 48.0;

// Distance validity window. VEX V5 Distance: 20-2000 mm (9999 = no object),
// confidence valid only above 200 mm.
constexpr double DIST_MIN_MM  = 20.0;
constexpr double DIST_MAX_MM  = 2000.0;
constexpr double CONF_DIST_MM = 200.0;
constexpr int    CONF_GATE    = 40;

// In range, and either close (good to ±15 mm) or far with passing confidence.
inline bool dist_ok(int raw_mm, int confidence) {
    if (raw_mm < DIST_MIN_MM || raw_mm > DIST_MAX_MM) return false;
    if (raw_mm <= CONF_DIST_MM)                       return true;
    return confidence >= CONF_GATE;
}

// Innovation gate: reject obvious wild measurements (in inches).
constexpr double INNOV_GATE_IN = 14.0;

// Process noise (per tick).
constexpr double Q_DIAG[3][3] = {{0.5, 0.0, 0.0},
                                 {0.0, 0.3, 0.0},
                                 {0.0, 0.0, 0.005}};

// Expected sensor-face-to-wall range for a pose + mount: place the sensor at
// its true world position and raycast. Same geometry as MCL (field_model.hpp).
double expected_range(double x, double y, double th, const field::SensorMount& m,
                      bool* hit_obstacle = nullptr) {
    double sx, sy, sang;
    field::sensor_world(x, y, th, m, sx, sy, sang);
    return field::raycast_map(sx, sy, sang, hit_obstacle);
}

} // anonymous namespace


// ─── EKF predict ────────────────────────────────────────────────
// Pose deltas come straight from LemLib odom, so F = I: add the delta and
// inflate P by Q.
//   x' = x + u ;  P' = P + Q
void predict(State& s,
             double dx_world,
             double dy_world,
             double dtheta_rad,
             const double Q[3][3]) {
    s.x     += dx_world;
    s.y     += dy_world;
    s.theta += dtheta_rad;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            s.P[i][j] += Q[i][j];
}


// ─── EKF update ─────────────────────────────────────────────────
// Scalar measurement z = h(state) + v, v ~ N(0, R).
//   y = z - h(x)                ; innovation
//   H = ∂h/∂x  evaluated at x   ; 1x3 Jacobian, numerical
//   S = H P H^T + R             ; innovation cov (scalar)
//   K = P H^T / S               ; 3x1 Kalman gain
//   x' = x + K y
//   P' = (I - K H) P
bool update(State& s, const WallObs& obs) {
    if (obs.raw_mm < DIST_MIN_MM || obs.raw_mm > DIST_MAX_MM) return false;

    // raw is sensor-face-to-wall; expected_range accounts for the mount, so
    // no center offset is added.
    const double measured = obs.raw_mm / field::MM_PER_IN;
    bool hitObs = false;
    const double expected = expected_range(s.x, s.y, s.theta, obs.mount, &hitObs);
    // Beam blocked by a mapped obstacle: no wall info, skip (also avoids a bad
    // Jacobian across the obstacle edge).
    if (hitObs) return false;
    const double innov    = measured - expected;
    // Innovation gate rejects beams that hit an unmapped object.
    if (std::abs(innov) > INNOV_GATE_IN) return false;

    // Numerical Jacobian H = [∂h/∂x, ∂h/∂y, ∂h/∂θ].
    constexpr double eps = 1e-4;
    double H[3];
    H[0] = (expected_range(s.x + eps, s.y, s.theta, obs.mount) - expected) / eps;
    H[1] = (expected_range(s.x, s.y + eps, s.theta, obs.mount) - expected) / eps;
    H[2] = (expected_range(s.x, s.y, s.theta + eps, obs.mount) - expected) / eps;

    // S = H P H^T + R  (scalar since z is 1D).
    double HP[3] = {0.0, 0.0, 0.0};
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
            HP[j] += H[k] * s.P[k][j];
    double S = obs.R_noise;
    for (int j = 0; j < 3; ++j) S += HP[j] * H[j];
    if (std::abs(S) < 1e-9) return false;

    // K = P H^T / S
    double K[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            K[i] += s.P[i][k] * H[k];
    for (int i = 0; i < 3; ++i) K[i] /= S;

    // x' = x + K y
    s.x     += K[0] * innov;
    s.y     += K[1] * innov;
    s.theta += K[2] * innov;

    // P' = (I - K H) P
    double newP[3][3];
    for (int i = 0; i < 3; ++i) {
        double IKH_row[3] = {-K[i]*H[0], -K[i]*H[1], -K[i]*H[2]};
        IKH_row[i] += 1.0;
        for (int j = 0; j < 3; ++j) {
            newP[i][j] = 0.0;
            for (int k = 0; k < 3; ++k)
                newP[i][j] += IKH_row[k] * s.P[k][j];
        }
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            s.P[i][j] = newP[i][j];

    return true;
}


// ─── Replay loop ────────────────────────────────────────────────
//
// NOTE: the wall update assumes (s.x, s.y) are field-absolute. Recorded poses
// start at setPose(0,0,0), so unless you setPose() to the real field-absolute
// start before recording, the innovation gate rejects everything (update is a
// no-op).
void run() {
    if (!pros::usd::is_installed()) {
        controller.print(0, 0, "FAIL: no SD card  ");
        pros::delay(1000);
        return;
    }

    std::ifstream fs(DATA_PATH);
    if (!fs.is_open()) {
        controller.print(0, 0, "FAIL: no file     ");
        pros::delay(1000);
        return;
    }

    // Count lines (quick sanity print).
    int lineCount = 0;
    std::string countLine;
    while (std::getline(fs, countLine)) if (!countLine.empty()) lineCount++;
    fs.clear();
    fs.seekg(0);
    controller.print(0, 0, "Lines: %d         ", lineCount);
    pros::delay(500);

    if (lineCount == 0) {
        controller.print(0, 0, "FAIL: empty file  ");
        pros::delay(1000);
        return;
    }

    chassis.setPose(0, 0, 0);

    State ekf;
    lemlib::Pose prevPose = chassis.getPose();

    std::string line;
    int step             = 0;
    int lastWaypointStep = 0;
    double lastHeading   = 0.0;

    bool prev_l1        = false;
    bool prev_right     = false;
    bool prev_x         = true;  // suppress phantom toggle from record-start X press
    bool wingState      = false;
    bool matchloadState = false;
    bool limiterState   = true;

    while (std::getline(fs, line)) {
        if (line.empty()) continue;

        // Parse: x y theta r1 r2 y b l2 l1 right x
        std::stringstream ss(line);
        std::string tok;
        double filex = 0, filey = 0, filet = 0;
        int r1=0, r2=0, y_btn=0, b_btn=0, l2=0, l1=0, right_btn=0, x_btn=0;
        try {
            std::getline(ss, tok, ' '); filex     = std::stod(tok);
            std::getline(ss, tok, ' '); filey     = std::stod(tok);
            std::getline(ss, tok, ' '); filet     = std::stod(tok);
            std::getline(ss, tok, ' '); r1        = std::stoi(tok);
            std::getline(ss, tok, ' '); r2        = std::stoi(tok);
            std::getline(ss, tok, ' '); y_btn     = std::stoi(tok);
            std::getline(ss, tok, ' '); b_btn     = std::stoi(tok);
            std::getline(ss, tok, ' '); l2        = std::stoi(tok);
            std::getline(ss, tok, ' '); l1        = std::stoi(tok);
            std::getline(ss, tok, ' '); right_btn = std::stoi(tok);
            std::getline(ss, tok, ' '); x_btn     = std::stoi(tok);
        } catch (...) { step++; continue; }

        if (step == 0) {
            controller.print(0, 0, "OK %.1f %.1f %.1f", filex, filey, filet);
            pros::delay(100);
        }

        // ── Mechanism replay ──────────────────────────────────────
        if (r1) {
            outtake(200); intake(200); middletake(600);
            limiter.set_value(1);
        } else if (r2) {
            intake(200); middletake(600); outtake(200);
            limiter.set_value(0);
        } else if (y_btn) {
            limiter.set_value(0);
            outtake(-200); intake(200); middletake(600);
        } else if (b_btn) {
            limiter.set_value(0);
            outtake(-55); intake(200); middletake(600);
        } else if (l2) {
            outtake(-200); intake(-200); middletake(-600);
        } else {
            intake(0); outtake(0); middletake(0);
        }
        if (l1 && !prev_l1) {
            wingState = !wingState;
            wing.set_value(wingState);
        }
        prev_l1 = l1;
        if (right_btn && !prev_right) {
            matchloadState = !matchloadState;
            matchLoad.set_value(matchloadState);
        }
        prev_right = right_btn;
        if (x_btn && !prev_x) {
            limiterState = !limiterState;
            limiter.set_value(limiterState);
            // limiter_light disabled — port 6 collision
        }
        prev_x = x_btn;

        // ── EKF predict from LemLib odom delta ────────────────────
        lemlib::Pose currPose = chassis.getPose();
        double dx_w     = currPose.x - prevPose.x;
        double dy_w     = currPose.y - prevPose.y;
        double dtheta_deg_raw = currPose.theta - prevPose.theta;
        while (dtheta_deg_raw >  180.0) dtheta_deg_raw -= 360.0;
        while (dtheta_deg_raw < -180.0) dtheta_deg_raw += 360.0;
        double dtheta_w = dtheta_deg_raw * PI / 180.0;
        prevPose = currPose;
        predict(ekf, dx_w, dy_w, dtheta_w, Q_DIAG);
        ekf.theta = currPose.theta * PI / 180.0;  // IMU owns heading

        // ── EKF update from each perimeter sensor ────────────────
        const int f_mm = fdist_sens.get(), f_cf = fdist_sens.get_confidence();
        const int b_mm = bdist_sens.get(), b_cf = bdist_sens.get_confidence();
        const int l_mm = ldist_sens.get(), l_cf = ldist_sens.get_confidence();
        const int r_mm = rdist_sens.get(), r_cf = rdist_sens.get_confidence();
        if (dist_ok(f_mm, f_cf)) update(ekf, {field::FRONT, (double)f_mm, 3.0});
        if (dist_ok(b_mm, b_cf)) update(ekf, {field::BACK,  (double)b_mm, 3.0});
        if (dist_ok(l_mm, l_cf)) update(ekf, {field::LEFT,  (double)l_mm, 3.0});
        if (dist_ok(r_mm, r_cf)) update(ekf, {field::RIGHT, (double)r_mm, 3.0});

        // Push EKF back into LemLib pose only when confident.
        if (ekf.P[0][0] < 0.5 && ekf.P[1][1] < 0.5) {
            chassis.setPose(ekf.x, ekf.y, ekf.theta * 180.0 / PI, false);
            prevPose = chassis.getPose();
        }

        // Skip most ticks for waypoint planning; replay every Nth.
        if (step % WAYPOINT_SKIP != 0) { step++; continue; }

        const double actX     = chassis.getPose().x;
        const double actY     = chassis.getPose().y;
        const double actTheta = chassis.getPose().theta;

        const double fx   = filex - actX;
        const double fy   = filey - actY;
        const double dist = std::sqrt(fx*fx + fy*fy);

        const double thetaRad = actTheta * PI / 180.0;
        const double dot      = fx * std::sin(thetaRad) + fy * std::cos(thetaRad);
        const bool goingBackwards = (dist > 1.0) && (dot < 0);

        double headingErr = filet - actTheta;
        while (headingErr >  180) headingErr -= 360;
        while (headingErr < -180) headingErr += 360;

        // Dynamic timeout + speed from recorded cadence.
        const int framesDelta    = step - lastWaypointStep;
        const int recordedTimeMs = framesDelta * 20;
        const int dynamicTimeout = std::max(200, (int)(recordedTimeMs * TIMEOUT_MULTIPLIER));

        const double recordedSpeedIPS = (recordedTimeMs > 0)
                                        ? (dist / (recordedTimeMs / 1000.0))
                                        : 20.0;

        int dynamicMaxSpeed = (int)(recordedSpeedIPS / MAX_SPEED_IPS * 127.0);
        dynamicMaxSpeed = std::max(MIN_SPEED, std::min(MAX_SPEED, dynamicMaxSpeed));

        float earlyExit = std::min(EARLY_EXIT_MAX,
                                   (float)(dist * 0.4 * (recordedSpeedIPS / MAX_SPEED_IPS)));
        earlyExit = std::max(0.5f, earlyExit);

        lastWaypointStep = step;

        // Classify segment: pure turn vs short precision vs long arc.
        const double turnRatio = (dist > 0.5) ? (std::abs(headingErr) / dist) : 999.0;
        const bool isPureTurn  = (std::abs(headingErr) > TURN_DETECT_DEG) &&
                                 (turnRatio > TURN_RATIO_THRESH || dist < SHORT_MOVE_IN);

        if (isPureTurn) {
            // Scan ahead to the end of the turn sequence so we fire one clean
            // blocking turn, not many micro-turns.
            double finalTurnHeading  = filet;
            std::streampos savedPos  = fs.tellg();
            std::string peekLine;

            while (std::getline(fs, peekLine)) {
                if (peekLine.empty()) continue;
                std::stringstream ps(peekLine);
                std::string pv;
                double px = 0, py = 0, pt = 0;
                try {
                    std::getline(ps, pv, ' '); px = std::stod(pv);
                    std::getline(ps, pv, ' '); py = std::stod(pv);
                    std::getline(ps, pv, ' '); pt = std::stod(pv);
                } catch (...) { continue; }

                const double pdx   = px - actX;
                const double pdy   = py - actY;
                const double pdist = std::sqrt(pdx*pdx + pdy*pdy);
                double pherr = pt - actTheta;
                while (pherr >  180) pherr -= 360;
                while (pherr < -180) pherr += 360;
                const double pratio = (pdist > 0.5) ? (std::abs(pherr) / pdist) : 999.0;
                const bool stillTurning = (std::abs(pherr) > TURN_DETECT_DEG) &&
                                          (pratio > TURN_RATIO_THRESH || pdist < SHORT_MOVE_IN);
                if (stillTurning) finalTurnHeading = pt;
                else break;
            }
            fs.clear();
            fs.seekg(savedPos);

            int turnTimeout = std::max(600, dynamicTimeout);
            chassis.turnToHeading(finalTurnHeading, turnTimeout, {.maxSpeed = 90}, false);

        } else if (dist < SHORT_MOVE_IN) {
            chassis.moveToPoint(filex, filey, dynamicTimeout,
                                {.forwards = !goingBackwards,
                                 .maxSpeed = (float)std::min(80, dynamicMaxSpeed),
                                 .minSpeed = 0},
                                false);

        } else {
            chassis.moveToPoint(filex, filey, dynamicTimeout,
                                {.forwards       = !goingBackwards,
                                 .maxSpeed       = (float)dynamicMaxSpeed,
                                 .minSpeed       = (float)MIN_SPEED,
                                 .earlyExitRange = earlyExit},
                                true);
        }

        lastHeading = filet;
        step++;
    }

    // Settle + final heading correction.
    chassis.waitUntilDone();

    double finalErr = lastHeading - chassis.getPose().theta;
    while (finalErr >  180) finalErr -= 360;
    while (finalErr < -180) finalErr += 360;
    if (std::abs(finalErr) > TURN_THRESHOLD_DEG)
        chassis.turnToHeading(lastHeading, TURN_TIMEOUT_MS, {}, false);

    controller.print(0, 0, "Done: %d steps    ", step);
    fs.close();
}

} // namespace oekf


void oekf_rerun() { oekf::run(); }
