#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep

// Get paths used for pure pursuit


void left_auton() {
    chassis.setPose(0,0,0);

    limiter.set_value(1);
    intakeMotor.move(-127);
    middleMotor.move(-127);
    outtake(127);
    //chassis.moveToPose(4, 15, 9, 1500, {.forwards = true, .lead = 0.5, .maxSpeed = 80}, true);
    // pros::delay(250);
    chassis.moveToPose(0, 29, -5, 3000, {.forwards = true, .lead = 0.6, .minSpeed=30}, false);
    //chassis.turnToHeading(-96, 2000);
    //chassis.moveToPose(7,15, -105, 2500, {.forwards = false, .lead = 0, .minSpeed=40}, false);
    limiter.set_value(0);
    //middleMotor.move(100);
    outtake(-127);
    middleMotor.move(-127);
    pros::delay(1500);
    //chassis.moveToPose(-48, -15, -150, 3000, {.forwards = true, .lead = .6, .minSpeed=67}, false);
    /*chassis.moveToPoint(-44, -6.5, 3000, {.forwards=true, .minSpeed=55}); // originally -7 for y
    chassis.turnToHeading(-150, 1000);
    matchLoad.set_value(1);
    chassis.moveToPoint(-44, -25, 3000, {.forwards=true, .minSpeed=35}, true);
    limiter.set_value(1);
    intakeMotor.move(-127);
    middleMotor.move(-127);
    outtake(127);
    pros::delay(1250); // here
    chassis.turnToHeading(-154, 1000);
    chassis.moveToPoint(-34, 2, 2500, {.forwards=false, .minSpeed=45}, false);
    matchLoad.set_value(0);
    limiter.set_value(0);
    outtake(127);
    middleMotor.move(-127);*/
    
    
    /*matchLoad.set_value(1);
    limiter.set_value(1);
    chassis.moveToPose(-50, -20, -152, 1500, {.forwards = true, .lead = 0, .minSpeed=35}, false);
    intakeMotor.move(-127);
    middleMotor.move(-127);
    outtake(127);*/

}

void right_auton() {
    chassis.setPose(0, 0, 0);
    limiter.set_value(1);
    intake(127);
    outtake(127);
    chassis.moveToPoint(0,50,2000,{.maxSpeed=70});
    chassis.turnToHeading(95,1000);
    matchLoad.set_value(1);
    chassis.moveToPoint(21,41,2000, {.minSpeed = 35});
    pros::delay(2000);
    chassis.moveToPoint(-40,40,2500, {.forwards=false,.maxSpeed=60}, false);
    matchLoad.set_value(0);
    limiter.set_value(0);

}

void skills_auton() {
    chassis.setPose(0,0,0);
    intakeMotor.move(-127);
    chassis.moveToPoint(0,-20,2000, {.forwards=false,.maxSpeed=50});
    chassis.moveToPoint(0,25,2000, {.minSpeed = 80});
}
