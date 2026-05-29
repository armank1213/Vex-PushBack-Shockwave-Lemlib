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
#include "robot/start_pose.hpp"
#include "robot/localization.hpp"

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

    //oekf_rerun();        // EKF replay of a recorded/drawn route
    //mcl_rerun();         // MCL replay of a recorded/drawn route
    //corrected_auton();   // hand-coded auton + background MCL correction
    localization_test(); // live readout to test EKF/MCL (correction off)
    //skills_auton();
}

void opcontrol() {
    //chassis.setPose(0, 0, 0);

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    open = false;
    // Don't open the file here — wait for X press

    // Bench localization test (works in driver control). Press UP to toggle:
    // reads start pose from the distance sensors, runs MCL with correction
    // OFF, and shows odom vs estimate on the controller while you drive.
    bool locTestOn = false;
    int  dbgTick   = 0;

    while (true) {
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX, false, .6);
        control();
        matchloadToggle();
        wingToggle();

        // UP = toggle the localization bench test.
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            if (!locTestOn) {
                lemlib::Pose sp = determine_start_pose(0.0);   // infer field pose from walls
                loc::start(sp.x, sp.y, sp.theta, loc::Method::MCL, /*correct=*/false);
                locTestOn = true;
                controller.rumble("-");
            } else {
                loc::stop();
                locTestOn = false;
            }
        }

        // Cycle the 3 readout lines so we stay under the controller's
        // print rate limit (~50ms/line).
        //   od  = LemLib wheel odometry (x y theta)
        //   est = filter estimate        (x y theta)
        //   var = position variance; n = effective particle count (MCL)
        if (locTestOn) {
            loc::Estimate e = loc::estimate();
            lemlib::Pose  p = chassis.getPose();
            int phase = dbgTick % 15;
            if (phase == 0)  controller.print(0, 0, "od  %4d %4d %4d", (int)p.x, (int)p.y, (int)p.theta);
            if (phase == 5)  controller.print(1, 0, "est %4d %4d %4d", (int)e.x, (int)e.y, (int)e.theta_deg);
            if (phase == 10) controller.print(2, 0, "var %5.1f n%3d ", e.var_xy, (int)e.extra);
        }
        dbgTick++;

        // Press X to toggle recording ON (open/reopen the file).
        // On record-start: snap chassis to field-absolute pose by
        // reading the perimeter distance sensors. Robot must be placed
        // at its true start position and pointing along its intended
        // heading before pressing X.
        // X is also a logged button column — replay-side suppresses the
        // first-frame phantom toggle by initializing prev_x = true.
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            if (pros::usd::is_installed()) {
                lemlib::Pose start = determine_start_pose(0.0);
                chassis.setPose(start.x, start.y, start.theta);
                controller.print(0, 0, "Start %.0f %.0f", start.x, start.y);

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