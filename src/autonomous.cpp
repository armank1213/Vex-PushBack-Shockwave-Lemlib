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

    const int    WAYPOINT_SKIP      = 20;
    const int    MOVE_TIMEOUT_MS    = 400;
    const int    MIN_SPEED          = 70;
    const int    MAX_SPEED          = 110;
    const double TURN_THRESHOLD_DEG = 15.0;
    const int    TURN_TIMEOUT_MS    = 275;

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
        int r1 = 0, r2 = 0, y_btn = 0, b_btn = 0, l2 = 0;
        int l1 = 0, right_btn = 0, x_btn = 0;

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
        } catch (...) {
            step++;
            continue;
        }

        if (step == 0) {
            controller.print(0, 0, "OK %.1f %.1f %.1f", filex, filey, filet);
            pros::delay(300);
        }

        // ---- Replay recorded mechanism states ----

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
            limiter_light.set_led_pwm(limiterState ? 100 : 0);
        }
        prev_x = x_btn;

        // ------------------------------------------

        // Skip chassis movement on non-waypoint frames — no delay, instant
        if (step % WAYPOINT_SKIP != 0) {
            step++;
            continue;
        }

        // Auto-detect backward movement using dot product
        double actX     = chassis.getPose().x;
        double actY     = chassis.getPose().y;
        double actTheta = chassis.getPose().theta;

        double dx = filex - actX;
        double dy = filey - actY;
        double dist = std::sqrt(dx * dx + dy * dy);

        double thetaRad = actTheta * PI / 180.0;
        double fwdX = std::sin(thetaRad);
        double fwdY = std::cos(thetaRad);
        double dot  = dx * fwdX + dy * fwdY;

        bool goingBackwards = (dist > 1.0) && (dot < 0);

        chassis.moveToPoint(filex, filey, MOVE_TIMEOUT_MS,
                            {.forwards = !goingBackwards,
                             .maxSpeed = MAX_SPEED,
                             .minSpeed = MIN_SPEED}, false);

        double headingErr = filet - actTheta;
        while (headingErr > 180)  headingErr -= 360;
        while (headingErr < -180) headingErr += 360;

        if (std::abs(headingErr) > TURN_THRESHOLD_DEG) {
            chassis.turnToHeading(filet, TURN_TIMEOUT_MS, {}, false);
        }

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