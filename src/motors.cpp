#include "robot/motors.hpp"
#include "robot/hardware.hpp"


// Basic motor control functions
void intake(int intakePower) {
   intakeMotor.move(-intakePower);
   middleMotor.move(-intakePower);
}


void outtake(int outtakePower) {
   outtakeMotor.move(-outtakePower);
}


void middletake(int sortPower) {
   middleMotor.move(sortPower);
}


// Control functions
void intakeControl() {
   if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
       outtake(127);
       intake(127);
       limiter.set_value(1);
       limiter_light.set_led_pwm(100);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // R2 for chey, L1 for naman
       intake(127);
       outtake(127);
       limiter.set_value(0);
       limiter_light.set_led_pwm(0);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {//chey l1 naman R2
      
       limiter.set_value(0);
       outtake(-127);
       intake(127);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       outtake(-127);
       intake(-127);
   }
   else {
       intake(0);
       outtake(0);
   }


}


void middleControl() {
   if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {//chey l1 naman R2
      
       limiter.set_value(0);
       outtake(-127);
       intake(127);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       outtake(-127);
       intake(-127);
   }
   else {
       outtake(0);
   }
}



