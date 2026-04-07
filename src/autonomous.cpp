#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

const double PI = 3.14159265358979323846;
const char* DATA_PATH = "/usd/dtData.txt";

// ===================== MOTION FUNCTION GUIDE =====================
// chassis.moveToPose(x, y, heading, timeout, {.forwards=true})
//   - Drives to (x,y) AND ends facing the given heading
//   - ALWAYS turns to face the target point before driving
//   - .forwards = false → drives backward to the point, ends facing heading
//
// chassis.moveToPoint(x, y, timeout, {.forwards=true})
//   - Drives to (x,y), doesn't care about final heading
//   - .forwards = false → drives STRAIGHT BACKWARD, no turning first
//
// RULE OF THUMB:
//   Going forward                → moveToPoint forwards=true
//   Backing up                   → moveToPoint forwards=false
//   Moving with a final heading  → moveToPose
//
// WAYPOINT_SKIP: frames to skip between moveToPoint calls.
// MOVE_TIMEOUT_MS: ms LemLib waits per waypoint.
// ================================================================

// ===================== SPEED TUNING =====================
// MIN_SPEED: prevents LemLib from decelerating too much between waypoints.
// MAX_SPEED: caps top speed. Lower if overshooting.
// TURN_THRESHOLD_DEG: only fire turnToHeading if heading diff > this.
// TURN_TIMEOUT_MS: how long turnToHeading has to complete.
// ========================================================

// ─── EKF State ───────────────────────────────────────────────
struct EKFState {
    double x = 0, y = 0, theta = 0; // theta in RADIANS internally
    double P[3][3] = {{10,0,0},{0,5,0},{0,0,0.3}}; // covariance
};

// Ray-cast from (rx, ry) in world direction `angle` to nearest wall of
// a 144x144 inch field. Returns distance in inches.
static double raycast144(double rx, double ry, double angle) {
    double cx = std::sin(angle); // VEX heading: 0 = +Y, CW positive
    double cy = std::cos(angle);
    double t  = 1e9;
    if (cx >  1e-6) t = std::min(t, (144.0 - rx) / cx);
    if (cx < -1e-6) t = std::min(t, (       rx) / (-cx));
    if (cy >  1e-6) t = std::min(t, (144.0 - ry) / cy);
    if (cy < -1e-6) t = std::min(t, (       ry) / (-cy));
    return t;
}

// Single EKF measurement update for one sensor.
// sensor_world_angle: direction sensor faces in world frame (radians)
// sensor_offset: how far sensor face is from robot center (inches)
// raw_mm: distance sensor reading in mm
static void ekf_update(EKFState& s,
                       double sensor_world_angle,
                       double sensor_offset,
                       double raw_mm,
                       double R_noise=3)
{
    // Reject bad readings (object in the way or sensor maxed out)
    if (raw_mm > 1900.0 || raw_mm < 10.0) return;
    double measured = (raw_mm / 25.4) + sensor_offset; // convert to inches, add offset

    // Expected wall distance from current estimated pose
    double expected = raycast144(s.x, s.y, sensor_world_angle);

    double innov = measured - expected;
    // Innovation gate: skip if unreasonably large (game object in the way)
    if (std::abs(innov) > 14.0) return;

    // Jacobian H via finite differences (1×3)
    const double eps = 1e-4;
    double H[3];
    H[0] = (raycast144(s.x + eps, s.y, sensor_world_angle) - expected) / eps;
    H[1] = (raycast144(s.x, s.y + eps, sensor_world_angle) - expected) / eps;
    // theta changes the world angle of the sensor
    H[2] = (raycast144(s.x, s.y, sensor_world_angle + eps) - expected) / eps;

    // S = H*P*H^T + R
    double HP[3] = {0, 0, 0};
    for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++)
            HP[j] += H[k] * s.P[k][j];
    double Sval = R_noise;
    for (int j = 0; j < 3; j++) Sval += HP[j] * H[j];
    if (std::abs(Sval) < 1e-9) return;

    // Kalman gain K = P*H^T / S (3×1)
    double K[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            K[i] += s.P[i][k] * H[k];
    for (int i = 0; i < 3; i++) K[i] /= Sval;

    // State update
    s.x     += K[0] * innov;
    s.y     += K[1] * innov;
    s.theta += K[2] * innov;

    // Covariance update P = (I - K*H)*P
    double newP[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double IKH_row[3] = {-K[i]*H[0], -K[i]*H[1], -K[i]*H[2]};
            IKH_row[i] += 1.0;
            newP[i][j] = 0;
            for (int k = 0; k < 3; k++)
                newP[i][j] += IKH_row[k] * s.P[k][j];
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            s.P[i][j] = newP[i][j];
}
// ─────────────────────────────────────────────────────────────


void odom_ekf_run() {
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

    int lineCount = 0;
    std::string countLine;
    while (std::getline(fs, countLine)) if (!countLine.empty()) lineCount++;
    fs.clear(); fs.seekg(0);

    controller.print(0, 0, "Lines: %d         ", lineCount);
    pros::delay(500);

    if (lineCount == 0) {
        controller.print(0, 0, "FAIL: empty file  ");
        pros::delay(1000);
        return;
    }

    chassis.setPose(0, 0, 0);

    const int    WAYPOINT_SKIP      = 20;
    const int    MOVE_TIMEOUT_MS    = 400;
    const int    MIN_SPEED          = 70;
    const int    MAX_SPEED          = 110;
    const double TURN_THRESHOLD_DEG = 15.0;
    const int    TURN_TIMEOUT_MS    = 275;

    // Sensor face offsets from robot center (inches) — TUNE THESE
    const double FRONT_OFFSET = 7.75;
    const double BACK_OFFSET  = 9.0;
    const double LEFT_OFFSET  = 7.5;
    const double RIGHT_OFFSET = 7.5;

    // EKF — init at pose (0,0,0), moderate uncertainty
    EKFState ekf;
    ekf.x = 0; ekf.y = 0; ekf.theta = 0;

    // Process noise Q (added to P each predict step)
    const double Q[3][3] = {{0.5,0,0},{0,0.3,0},{0,0,0.005}};

    lemlib::Pose prevPose = chassis.getPose();

    std::string line;
    int step = 0;

    bool prev_l1    = false;
    bool prev_right = false;
    bool prev_x     = false;
    bool wingState      = false;
    bool matchloadState = false;
    bool limiterState   = true;

    while (std::getline(fs, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string val;
        double filex, filey, filet;
        int r1=0,r2=0,y_btn=0,b_btn=0,l2=0,l1=0,right_btn=0,x_btn=0;

        try {
            std::getline(ss, val, ' '); filex     = std::stod(val);
            std::getline(ss, val, ' '); filey     = std::stod(val);
            std::getline(ss, val, ' '); filet     = std::stod(val);
            std::getline(ss, val, ' '); r1        = std::stoi(val);
            std::getline(ss, val, ' '); r2        = std::stoi(val);
            std::getline(ss, val, ' '); y_btn     = std::stoi(val);
            std::getline(ss, val, ' '); b_btn     = std::stoi(val);
            std::getline(ss, val, ' '); l2        = std::stoi(val);
            std::getline(ss, val, ' '); l1        = std::stoi(val);
            std::getline(ss, val, ' '); right_btn = std::stoi(val);
            std::getline(ss, val, ' '); x_btn     = std::stoi(val);
        } catch (...) { step++; continue; }

        if (step == 0) {
            controller.print(0, 0, "OK %.1f %.1f %.1f", filex, filey, filet);
            pros::delay(100);
        }

        // ── Mechanism replay (unchanged) ──────────────────────────
        if (r1) {
            outtake(200); intake(200); middletake(600); limiter.set_value(1);
        } else if (r2) {
            intake(200); middletake(600); outtake(200); limiter.set_value(0);
        } else if (y_btn) {
            limiter.set_value(0); outtake(-200); intake(200); middletake(600);
        } else if (b_btn) {
            limiter.set_value(0); outtake(-55); intake(200); middletake(600);
        } else if (l2) {
            outtake(-200); intake(-200); middletake(-600);
        } else {
            intake(0); outtake(0); middletake(0);
        }
        if (l1 && !prev_l1) { wingState = !wingState; wing.set_value(wingState); }
        prev_l1 = l1;
        if (right_btn && !prev_right) { matchloadState = !matchloadState; matchLoad.set_value(matchloadState); }
        prev_right = right_btn;
        if (x_btn && !prev_x) { limiterState = !limiterState; limiter.set_value(limiterState); limiter_light.set_led_pwm(limiterState ? 100 : 0); }
        prev_x = x_btn;
        // ─────────────────────────────────────────────────────────

        // ── EKF: Predict from odometry delta ─────────────────────
        lemlib::Pose currPose = chassis.getPose();
        double dx     = currPose.x - prevPose.x;
        double dy     = currPose.y - prevPose.y;
        double dtheta = (currPose.theta - prevPose.theta) * PI / 180.0;
        prevPose = currPose;

        ekf.x     += dx;
        ekf.y     += dy;
        ekf.theta += dtheta;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ekf.P[i][j] += Q[i][j];

        // ── EKF: Update from all four distance sensors ────────────
        // World angles: sensor_world_angle = ekf.theta + sensor_mounting_angle
        // Front: same dir as robot heading (0 offset), Back: +PI, Left: -PI/2, Right: +PI/2
        ekf_update(ekf, ekf.theta,            FRONT_OFFSET, fdist_sens.get());
        ekf_update(ekf, ekf.theta + PI,       BACK_OFFSET,  bdist_sens.get());
        ekf_update(ekf, ekf.theta - PI / 2.0, LEFT_OFFSET,  ldist_sens.get());
        ekf_update(ekf, ekf.theta + PI / 2.0, RIGHT_OFFSET, rdist_sens.get());

        // Re-seed LemLib only once covariance is small enough (converged)
        if (ekf.P[0][0] < 0.5 && ekf.P[1][1] < 0.5) {
            chassis.setPose(ekf.x, ekf.y,
                            ekf.theta * 180.0 / PI, false);
            prevPose = chassis.getPose(); // sync prevPose after re-seed
        }
        // ─────────────────────────────────────────────────────────

        if (step % WAYPOINT_SKIP != 0) { step++; continue; }

        double actX     = chassis.getPose().x;
        double actY     = chassis.getPose().y;
        double actTheta = chassis.getPose().theta;

        double fx = filex - actX, fy = filey - actY;
        double dist = std::sqrt(fx*fx + fy*fy);
        double thetaRad = actTheta * PI / 180.0;
        double dot = fx * std::sin(thetaRad) + fy * std::cos(thetaRad);
        bool goingBackwards = (dist > 1.0) && (dot < 0);

        chassis.moveToPoint(filex, filey, MOVE_TIMEOUT_MS,
                            {.forwards  = !goingBackwards,
                             .maxSpeed  = MAX_SPEED,
                             .minSpeed  = MIN_SPEED}, false);

        double headingErr = filet - actTheta;
        while (headingErr >  180) headingErr -= 360;
        while (headingErr < -180) headingErr += 360;
        if (std::abs(headingErr) > TURN_THRESHOLD_DEG)
            chassis.turnToHeading(filet, TURN_TIMEOUT_MS, {}, false);

        pros::delay(20);
        step++;
    }

    controller.print(0, 0, "Done: %d steps    ", step);
    fs.close();
}

void angular_tuning() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000, {}, false);
}

void lateral_tuning() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20, 1000, {}, false);
}

void left_auton() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 20, 0, 2000);
}

void right_auton() {
    chassis.setPose(0, 0, 0);
    // add right auton path here
}

void skills_auton() {
    chassis.setPose(0, 0, 0);
    // add skills path here
}