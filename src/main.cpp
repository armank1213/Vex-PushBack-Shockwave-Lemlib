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

// re run thing
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

    /*if (autonSelection == 0) {
        left_auton();
    } else if (autonSelection == 1) {
        right_auton();
    } else if (autonSelection == 2) {
        skills_auton();
    }*/

    //voltage_re_run();
    odom_ekf_run();
}

void opcontrol() {
    chassis.setPose(0,0,0);

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    const bool allianceColor = true;
    bool isOurBlock = false;

    if (pros::usd::is_installed()) {
        // reset pose so recording and re-run share the same 0,0,0 origin
        // always rewrite the file from scratch on new recording
        ofs.open("/usd/dtData.txt", std::ofstream::out | std::ofstream::trunc);
        open = true;
    } else {
        // stop recording
        ofs.close();
        open = false;
    }

    while (true) {
        // Get joystick positions
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Chassis Drive Functions
        chassis.arcade(leftY, rightX, false, .6);

        // Intake and outtake control functions
        control();

        // Matchload Pneumatics Toggle
        matchloadToggle();
        // Wing Mech Pneumatics Toggle
        wingToggle();

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            open = true;
        }

        // recording — only writes when open == true
        //voltage_recording(ofs, open);
        pose_recording(ofs, open);

        pros::delay(20);
    }

    ofs.close();
}