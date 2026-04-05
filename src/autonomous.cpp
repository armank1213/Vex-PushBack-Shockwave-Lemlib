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

// ===================== TUNING GUIDE =====================
// kP_theta: proportional turning gain. Start at 90.
// kD_theta: derivative damping. Start at 600. Only active when turning.
// kP_lateral: forward/backward gain. Start at 50.
// THETA_DEADBAND: ignore heading errors smaller than this (degrees). Keep at 2.0.
// TURN_THRESHOLD: turn in place if error exceeds this (degrees). Keep at 10.0.
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

    const double kP_lateral     = 90.0;
    const double kP_theta       = 90.0;
    const double kD_theta       = 600.0;
    const double maxVoltage     = 10000;
    const double THETA_DEADBAND = 2.0;
    const double TURN_THRESHOLD = 10.0;

    std::string line;
    int step = 0;
    double prevErrorTheta = 0.0;

    while (std::getline(fs, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string val;
        double filex, filey, filet;

        try {
            std::getline(ss, val, ' '); filex = std::stod(val);
            std::getline(ss, val, ' '); filey = std::stod(val);
            std::getline(ss, val, ' '); filet = std::stod(val);
        } catch (...) {
            if (step == 0) {
                controller.print(0, 0, "BAD FMT line 0    ");
                pros::delay(1000);
            }
            continue;
        }

        if (step == 0) {
            controller.print(0, 0, "OK %.1f %.1f %.1f", filex, filey, filet);
            pros::delay(300);
        }

        double actX     = chassis.getPose().x;
        double actY     = chassis.getPose().y;
        double actTheta = chassis.getPose().theta;

        double errorX     = filex - actX;
        double errorY     = filey - actY;
        double errorTheta = filet - actTheta;

        while (errorTheta > 180)  errorTheta -= 360;
        while (errorTheta < -180) errorTheta += 360;

        // Deadband: zero out small heading errors
        if (std::abs(errorTheta) < THETA_DEADBAND) {
            errorTheta = 0.0;
            // IMPORTANT: reset prevErrorTheta so derivative doesn't spike
            // on the transition into/out of the deadband
            prevErrorTheta = 0.0;
        }

        // Derivative only meaningful when actively turning — zero it out in deadband
        double dErrorTheta = errorTheta - prevErrorTheta;
        prevErrorTheta = errorTheta;

        double thetaRad = actTheta * PI / 180.0;

        double fwdX = std::sin(thetaRad);
        double fwdY = std::cos(thetaRad);
        double dot  = errorX * fwdX + errorY * fwdY;

        double driveVoltage = 0.0;
        double turnVoltage  = std::clamp(
            kP_theta * errorTheta + kD_theta * dErrorTheta,
            -maxVoltage, maxVoltage
        );

        if (std::abs(errorTheta) <= TURN_THRESHOLD) {
            driveVoltage = std::clamp(kP_lateral * dot, -maxVoltage, maxVoltage);
        }

        double leftVoltage  = std::clamp(driveVoltage + turnVoltage, -maxVoltage, maxVoltage);
        double rightVoltage = std::clamp(driveVoltage - turnVoltage, -maxVoltage, maxVoltage);

        leftMotors.move_voltage(leftVoltage);
        rightMotors.move_voltage(rightVoltage);

        pros::delay(20);
        step++;
    }

    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
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

        double thetaRad = actTheta * PI / 180.0;
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
    // add left auton path here
}

void right_auton() {
    chassis.setPose(0, 0, 0);
    // add right auton path here
}

void skills_auton() {
    chassis.setPose(0, 0, 0);
    // add skills path here
}