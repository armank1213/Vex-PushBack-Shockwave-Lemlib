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
        intake(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // R2 for chey, L1 for naman
        intake(-127);
    }
    else {
        intake(0);
    }

}

void outtakeControl() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {//chey l1 naman R2
        outtake(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        outtake(-127);
    }
    else {
        outtake(0);
    }
}
