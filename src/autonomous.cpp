#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "robot/re_run_helpers.hpp" // IWYU pragma: keep
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
//   - Use for: moving to a position where final heading matters
//   - .forwards = false → drives backward to the point, ends facing heading
//
// chassis.moveToPoint(x, y, timeout, {.forwards=true})
//   - Drives to (x,y), doesn't care about final heading
//   - .forwards = false → drives STRAIGHT BACKWARD, no turning first
//   - Use for: backing up, simple point-to-point movement
//
// RULE OF THUMB:
//   Going forward to a new position      → moveToPose or moveToPoint forwards=true
//   Backing up (reversing)               → moveToPoint forwards=false
//   Moving to a point with a final angle → moveToPose
//
// WAYPOINT_SKIP: frames to skip between moveToPose calls in odom_ekf_run.
//   20 is a good starting point.
// MOVE_TIMEOUT_MS: ms LemLib waits per waypoint. Start at 500.
// ================================================================

// ===================== SPEED TUNING =====================
// MIN_SPEED: prevents LemLib from decelerating too much between waypoints.
//   - Higher = faster throughout, less precise
//   - Lower  = more precise, but slower near each waypoint
//   - Start at 60. Try 60-110 to match driver speed.
//
// MAX_SPEED: caps the top speed LemLib will use.
//   - 127 = uncapped (full speed)
//   - Lower if the robot overshoots waypoints badly
//   - Start at 100. Lower to 80 if overshooting.
//
// TURN_THRESHOLD_DEG: minimum heading change to trigger a turnToHeading.
//   - Prevents tiny corrections from firing a turn on every waypoint.
//   - Start at 15 degrees. Lower if turns are being skipped, raise if
//     there are too many micro-turns.
//
// TURN_TIMEOUT_MS: how long turnToHeading has to complete.
//   - Should be enough for a fast turn. Start at 400ms.
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
    const int    MOVE_TIMEOUT_MS    = 500;
    const int    MIN_SPEED          = 60;
    const int    MAX_SPEED          = 100;
    const double TURN_THRESHOLD_DEG = 15.0;
    const int    TURN_TIMEOUT_MS    = 400;

    std::string line;
    int step = 0;

    while (std::getline(fs, line)) {
        if (line.empty()) continue;

        if (step % WAYPOINT_SKIP != 0) {
            step++;
            continue;
        }

        std::stringstream ss(line);
        std::string val;
        double filex, filey, filet;

        try {
            std::getline(ss, val, ' '); filex = std::stod(val);
            std::getline(ss, val, ' '); filey = std::stod(val);
            std::getline(ss, val, ' '); filet = std::stod(val);
        } catch (...) {
            step++;
            continue;
        }

        if (step == 0) {
            controller.print(0, 0, "OK %.1f %.1f %.1f", filex, filey, filet);
            pros::delay(300);
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