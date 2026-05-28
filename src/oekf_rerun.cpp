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

// Sensor offsets, robot center -> sensor face along sensor pointing dir.
constexpr double FRONT_OFFSET = 7.75;
constexpr double BACK_OFFSET  = 9.00;
constexpr double LEFT_OFFSET  = 7.50;
constexpr double RIGHT_OFFSET = 7.50;

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

// Distance sensor validity window. VEX Distance is ~30-2000 mm spec'd,
// but past ~1900 mm the readings get noisy fast.
constexpr double DIST_MIN_MM = 10.0;
constexpr double DIST_MAX_MM = 1900.0;

// Innovation gate: reject obvious wild measurements (in inches).
constexpr double INNOV_GATE_IN = 14.0;

// Process noise (per tick). Tuned by feel for now.
constexpr double Q_DIAG[3][3] = {{0.5, 0.0, 0.0},
                                 {0.0, 0.3, 0.0},
                                 {0.0, 0.0, 0.005}};

// Distance from (rx,ry) along world heading `angle` to the nearest wall
// of an axis-aligned [0, FIELD_IN] x [0, FIELD_IN] square. Returns inches.
//
// Convention: angle 0 = +Y world (matches LemLib heading), increasing CW.
double raycastField(double rx, double ry, double angle) {
    const double cx = std::sin(angle);
    const double cy = std::cos(angle);
    double t = 1e9;
    if (cx >  1e-6) t = std::min(t, (FIELD_IN - rx) / cx);
    if (cx < -1e-6) t = std::min(t, (rx)            / (-cx));
    if (cy >  1e-6) t = std::min(t, (FIELD_IN - ry) / cy);
    if (cy < -1e-6) t = std::min(t, (ry)            / (-cy));
    return t;
}

} // anonymous namespace


// ─── EKF predict ────────────────────────────────────────────────
// Linear motion model in the state we track directly (we get pose deltas
// from LemLib odom already, so F = I and the only thing to do is add
// the delta and inflate P by Q).
//
//   x_k|k-1 = x_k-1|k-1 + u_k
//   P_k|k-1 = F P_k-1|k-1 F^T + Q   ; F = I here
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

    const double measured = (obs.raw_mm / 25.4) + obs.sensor_offset_in;
    const double expected = raycastField(s.x, s.y, obs.sensor_world_angle);
    const double innov    = measured - expected;
    if (std::abs(innov) > INNOV_GATE_IN) return false;

    // Numerical Jacobian H = [∂h/∂x, ∂h/∂y, ∂h/∂θ].
    constexpr double eps = 1e-4;
    double H[3];
    H[0] = (raycastField(s.x + eps, s.y, obs.sensor_world_angle) - expected) / eps;
    H[1] = (raycastField(s.x, s.y + eps, obs.sensor_world_angle) - expected) / eps;
    H[2] = (raycastField(s.x, s.y, obs.sensor_world_angle + eps) - expected) / eps;

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
// NOTE on field frame: raycastField assumes (s.x, s.y) live in [0, 144]
// field-absolute inches. The recorded poses start at chassis.setPose(0,0,0)
// at the start of opcontrol — so to make the EKF wall update meaningful,
// the user has to setPose() to the *actual field-absolute* start pose
// before recording. Until that's wired, the EKF update is a no-op in
// practice (innovation gate will reject everything).
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
        double dtheta_w = (currPose.theta - prevPose.theta) * PI / 180.0;
        prevPose = currPose;
        predict(ekf, dx_w, dy_w, dtheta_w, Q_DIAG);

        // ── EKF update from each perimeter sensor ────────────────
        // VEX Distance get_confidence() returns 0..63. Only fuse readings
        // where the sensor itself reports it's looking at a real surface.
        constexpr int CONF_GATE = 40;
        if (fdist_sens.get_confidence() > CONF_GATE)
            update(ekf, {ekf.theta,            FRONT_OFFSET, (double)fdist_sens.get(), 3.0});
        if (bdist_sens.get_confidence() > CONF_GATE)
            update(ekf, {ekf.theta + PI,       BACK_OFFSET,  (double)bdist_sens.get(), 3.0});
        if (ldist_sens.get_confidence() > CONF_GATE)
            update(ekf, {ekf.theta - PI / 2.0, LEFT_OFFSET,  (double)ldist_sens.get(), 3.0});
        if (rdist_sens.get_confidence() > CONF_GATE)
            update(ekf, {ekf.theta + PI / 2.0, RIGHT_OFFSET, (double)rdist_sens.get(), 3.0});

        // Only push EKF back into LemLib pose when we're confident.
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
            // Scan ahead — find the END of the turn sequence so we fire
            // ONE clean blocking turn, not many micro-turns.
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
