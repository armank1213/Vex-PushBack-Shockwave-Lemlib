#include "main.h" // IWYU pragma: keep
#include "pros/motors.h"
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
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    //odom_ekf_run();
    skills_auton();
}

void opcontrol() {
    //chassis.setPose(0, 0, 0);

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    open = false;
    // Don't open the file here — wait for X press

    while (true) {
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX, false, .6);
        control();
        matchloadToggle();
        wingToggle();

        // Press X to toggle recording ON (open/reopen the file)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            if (pros::usd::is_installed()) {
                if (ofs.is_open()) ofs.close(); // close any previous session
                ofs.open("/usd/dtData.txt", std::ofstream::out | std::ofstream::trunc);
                if (ofs.is_open()) {
                    open = true;
                    controller.rumble("."); // optional: confirm to driver
                }
            }
        }

        pose_recording(ofs, open);

        pros::delay(20);
    }

    if (ofs.is_open()) ofs.close();
}