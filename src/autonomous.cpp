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

    const int WAYPOINT_SKIP   = 20;
    const int MOVE_TIMEOUT_MS = 500;

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

        // dot < 0 means target is behind the robot → go backwards
        // dist > 1.0 guard avoids noise when already near the waypoint
        bool goingBackwards = (dist > 1.0) && (dot < 0);

        chassis.moveToPose(filex, filey, filet, MOVE_TIMEOUT_MS,
                           {.forwards = !goingBackwards}, false);

        step++;
    }

    controller.print(0, 0, "Done: %d steps    ", step);
    fs.close();
}

void re_run() {
    if (!pros::usd::is_installed()) return;

    std::ifstream ifs(DATA_PATH);
    if (!ifs.is_open()) return;

    chassis.setPose(0, 0, 0);

    std::string line;

    const double kP_lateral = 0.3;
    const double kP_theta   = 8.0;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string val;
        double left1V, right1V, expX, expY, expTheta;

        try {
            std::getline(ss, val, ' '); left1V   = std::stod(val);
            std::getline(ss, val, ' '); right1V  = std::stod(val);
            std::getline(ss, val, ' '); expX     = std::stod(val);
            std::getline(ss, val, ' '); expY     = std::stod(val);
            std::getline(ss, val, ' '); expTheta = std::stod(val);
        } catch (...) {
            continue;
        }

        double actX     = chassis.getPose().x;
        double actY     = chassis.getPose().y;
        double actTheta = chassis.getPose().theta;

        double errorX     = expX     - actX;
        double errorY     = expY     - actY;
        double errorTheta = expTheta - actTheta;

        while (errorTheta > 180)  errorTheta -= 360;
        while (errorTheta < -180) errorTheta += 360;

        double thetaRad = actTheta * M_PI / 180.0;
        double lateralError = errorX * sin(thetaRad) + errorY * cos(thetaRad);

        double lateralCorrection = kP_lateral * lateralError * 120;
        double thetaCorrection   = kP_theta   * errorTheta;

        const double maxLateralCorrection = 2000;
        const double maxThetaCorrection   = 3000;
        lateralCorrection = std::clamp(lateralCorrection, -maxLateralCorrection, maxLateralCorrection);
        thetaCorrection   = std::clamp(thetaCorrection,   -maxThetaCorrection,   maxThetaCorrection);

        double battery = pros::battery::get_voltage() / 12000.0;

        leftMotors.move_voltage( (left1V  * battery) + lateralCorrection - thetaCorrection);
        rightMotors.move_voltage((right1V * battery) + lateralCorrection + thetaCorrection);

        pros::delay(20);
    }

    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
    ifs.close();
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