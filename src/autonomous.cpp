#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep

// Get paths used for pure pursuit

void angular_tuning() {
    chassis.setPose(0,0,0);
    chassis.turnToHeading(90,1000,{},false);
}
void lateral_tuning() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,20,1000,{},false);
}

void left_auton() {
    chassis.setPose(0,0,0);
    wing.set_value(0);
    limiter.set_value(1);
    intakeMotor.move(-200);
    middleMotor.move(-200);
    outtake(200);
    //Picking up 3 blocks
    chassis.moveToPose(0, 16, 10, 1000, {.forwards = true, .lead = 0, .minSpeed=30}, false);
    pros::delay(150);
    chassis.moveToPose(0, 18, 0, 1500, {.forwards = true, .lead = 0, .minSpeed=30}, false);
    pros::delay(150);
    chassis.moveToPose(0,24,10,1000,{.forwards=true, .lead=0, .minSpeed=40},false);
    pros::delay(150);
    chassis.turnToHeading(-104,1000);
    //middle goal scoring
    chassis.moveToPoint(19,37,1500,{.forwards=false,.maxSpeed=70},false);
    limiter.set_value(1);
    outtake(-150);
    pros::delay(200);
    middletake(600);
    pros::delay(200);
    middletake(600);
    pros::delay(1500);
    intake(0);
    outtake(0);
    middletake(0);
    //matchloading
    chassis.moveToPose(-38,12,-158,2500,{.forwards=true,.lead=0},false);
    matchLoad.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(-38,3,500,{.forwards=true},false);
    intakeMotor.move(-200);
    middleMotor.move(-200);
    outtake(200);
    pros::delay(1000);
    //long goal scoring
    chassis.moveToPoint(-28,34,1500,{.forwards=false},false);
    matchLoad.set_value(false);
    limiter.set_value(0);
    pros::delay(1500);
    intake(0);
    outtake(0);
    middletake(0);
    //winging after long goal
    wing.set_value(1);
    chassis.swingToHeading(-330, DriveSide::LEFT, 1000);
    wing.set_value(0);
    chassis.turnToHeading(-300,500);


    
    
    /*middletake(0);
    outtake(0);
    chassis.moveToPose(-39,12,-158,2000,{.forwards=true, .maxSpeed=80},false);
    matchLoad.set_value(true);
    chassis.moveToPoint(-36,3,2000,{.forwards=true, .maxSpeed=80},false);

    limiter.set_value(1);
    intake(200);
    middletake(600);
    outtake(200);
    pros::delay(250);
    middletake(-600);
    pros::delay(250);
    middletake(600);
    pros::delay(2000);
    chassis.moveToPoint(-26,31,2000, {.forwards=false},false);
    outtake(200);
    //chassis.moveToPose(18,24,-115,2000,{.forwards=false},false); 
    //chassis.moveToPoint(0,24,-115,2000,{.forwards=false},false);
    //chassis.moveToPose(-1,6,-114,2500,{.forwards=false, .lead=0, .maxSpeed=127},false);
    //chassis.turnToHeading(-96, 2000);
    //chassis.moveToPose(7,15, -105, 2500, {.forwxards = false, .lead = 0, .minSpeed=40}, false);
    //middleMotor.move(100);*/

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
