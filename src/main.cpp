#include "main.h"
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

#include <cstdio>
#include <iostream>
#include <fstream>

//re run thing
std::ofstream ofs;
bool open = false;
/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
   // Calibrate chassis (drivetrain, odometry, PID, sensors, controller steering)
   chassis.calibrate();


   // Reset rotation sensors
   vertical_rotation.reset();
   horizontal_rotation.reset();


   // Wait for IMU calibration
   while (imu.is_calibrating()) {
       pros::delay(10);
   }
   //pros::lcd::initialize(); // initialize brain screen


   /*pros::Task([&] {
       while (true) {
           pros::lcd::print(0, "X: %f", chassis.getPose().x);
           pros::lcd::print(1, "Y: %f", chassis.getPose().y);
           pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
           pros::delay(10);
       }
   });*/

   // Initialize UI
   initializeUI();

   // re run thing
    if (pros::usd::is_installed()) {  // FIXED: was Brain.SDcard.is_inserted()
        ofs.open("dtData.txt", std::ofstream::out);
        open = true;
    }
}


/**
* Runs while the robot is disabled
*/
void disabled() {}


/**
* Runs after initialize if the robot is connected to field control
*/
void competition_initialize() {}


/**
* Runs during autonomous
*/
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
    re_run();
}

void opcontrol() {
   leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

   // Anti-jam control variables
   const bool allianceColor = true; // true for red, false for blue
   bool isOurBlock = false;


   while (true) {
       // Get joystick positions
       int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
       int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);


       // Chassis Drive Functions
       chassis.arcade(leftY, rightX, false, .6);


       // lmao deal with it naman + chey - achin (hella right - arman)
       //chassis.tank(leftY, rightY, false);


       //chassis.curvature(leftY, rightY, false);


       // Intake and outtake control functions
       //middleControl();
       control();



       // Set light to 100% and get distance and color readings
       /*colorSensor.set_led_pwm(100);
       int distance = distanceSensor.get_distance();
       double hue = colorSensor.get_hue();*/


       // Matchload Pneumatics Toggle
       matchloadToggle();
       // Limiter Pneumatics Toggle
       //limiterToggle();
       // Wing Mech Pneumatics Toggle
       wingToggle();

       // re-run
        if (open == true) {  // FIXED: was = instead of ==

            double l1 = leftMotors.get_voltage();   // FIXED: MotorGroup not array
            double r1 = rightMotors.get_voltage();  // FIXED: MotorGroup not array

            ofs << l1 << " " << r1 << "\n";  // FIXED: only 2 values since MotorGroup

            ofs.flush();
        }
        pros::delay(20);
    }
   ofs.close();
}