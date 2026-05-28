#include "robot/motors.hpp"
#include "robot/hardware.hpp"


// Basic motor control functions
void intake(int intakeRPM) {
    intakeMotor.move_velocity(-intakeRPM);
}

void middletake(int middleRPM) {
    middleMotor.move_velocity(-middleRPM);
}

void outtake(int outtakeRPM) {
   outtakeMotor.move_velocity(-outtakeRPM);
}

// Control functions
void control() {
    
    static bool middleScore_voltageToggle = false;
    static bool lastBButtonState = false;

   if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
       outtake(200);
       intake(200);
       middletake(600);
       limiter.set_value(1);
       // limiter_light.set_led_pwm(100); // disabled, port 6 collision
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // R2 for chey, L1 for naman
       intake(200);
       middletake(600);
       outtake(200);
       limiter.set_value(0);
       // limiter_light.set_led_pwm(0); // disabled, port 6 collision
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {//chey l1 naman R2
       limiter.set_value(0);
       outtake(-200);
       intake(200);
       middletake(600);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {//chey l1 naman R2
       limiter.set_value(0);
       outtake(-55);
       intake(200);
       middletake(600);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       outtake(-200);
       intake(-200);
       middletake(-600);
   }
   else {
       intake(0);
       outtake(0);
       middletake(0);
   }


}


void middleControl() {
   if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {//chey l1 naman R2
      
       limiter.set_value(0);
       outtake(-200);
       intake(200);
       middletake(600);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       outtake(-200);
       intake(-200);
       middletake(-600);
   }
   else {
       outtake(0);
   }
}



