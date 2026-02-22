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
   //leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   //rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

   if (autonSelection == 0) {
       left_auton();
   } else if (autonSelection == 1) {
       right_auton();
   } else if (autonSelection == 2) {
       skills_auton();
   }
}


void opcontrol() {
   leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


   /*// Matchload piston variables
   bool matchloadTogle = false;
   static bool lastAButtonState = false;


   // Limiter piston variables
   bool limiterPistonToggle = false;
   static bool lastYButtonState = false;


   // Wing piston variables
   bool wingPistonToggle = false;
   static bool lastXButtonState = false;
   // Color sorting variables
   static int sortMode = 0;
   static bool LastB_ButtonState = false;*/


   // Anti-jam control variables
   const bool allianceColor = true; // true for red, false for blue
   bool isOurBlock = false;


   while (true) {
       // Get joystick positions
       int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
       int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);


       // Chassis Drive Functions
       //chassis.arcade(leftY, rightX, false, .6);


       // lmao deal with it naman + chey - achin (hella right - arman)
       chassis.tank(leftY, rightY, false);


       //chassis.curvature(leftY, rightY, false);


       // Intake and outtake control functions
       //middleControl();
       control();



       // Set light to 100% and get distance and color readings
       /*colorSensor.set_led_pwm(100);
       int distance = distanceSensor.get_distance();
       double hue = colorSensor.get_hue();*/
  


       // Color sorting functions
       /*if (colorSortMode == 0) {
           if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
               if (limiter.is_extended() == false) {
                   red_colorSort(distance, hue);
               }
           }
       }
       else if (colorSortMode == 1) {
           if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
               if (limiter.is_extended() == false) {
                   blue_colorSort(distance, hue);
               }
           }
       }*/


       // Matchload Pneumatics Toggle
       matchloadToggle();
       // Limiter Pneumatics Toggle
       //limiterToggle();
       // Wing Mech Pneumatics Toggle
       wingToggle();




       // Delay to save resources and update lvgl timer
       lv_timer_handler();
       pros::delay(30);
   }
}



