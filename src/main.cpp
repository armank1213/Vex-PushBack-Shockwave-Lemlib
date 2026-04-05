#include "main.h" // IWYU pragma: keep
#include "robot/hardware.hpp"
#include "robot/chassis_config.hpp"
#include "robot/ui.hpp" // IWYU pragma: keep
#include "robot/autonomous.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "robot/motors.hpp"
#include "robot/color_sort.hpp" // IWYU pragma: keep
#include "robot/pneumatics.hpp"
#include "robot/re_run_helpers.hpp"

#include <cstdio>
#include <iostream>
#include <fstream>

std::ofstream ofs;
bool open = false;

void initialize() {
    chassis.calibrate();

    vertical_rotation.reset();
    horizontal_rotation.reset();

    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    initializeUI();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    odom_ekf_run();
}

void opcontrol() {
    chassis.setPose(0, 0, 0);

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // open = false by default; only start recording when X is pressed
    open = false;

    if (pros::usd::is_installed()) {
        // Open/truncate the file now, but don't start recording yet
        ofs.open("/usd/dtData.txt", std::ofstream::out | std::ofstream::trunc);
    }

    while (true) {
        // Get joystick positions
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Chassis Drive
        chassis.arcade(leftY, rightX, false, .6);

        // Intake and outtake control
        control();

        // Pneumatics toggles
        matchloadToggle();
        wingToggle();

        // Press X to START recording
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            if (pros::usd::is_installed() && ofs.is_open()) {
                open = true;
            }
        }

        // Recording — only writes when open == true
        pose_recording(ofs, open);

        pros::delay(20);
    }

    ofs.close();
}