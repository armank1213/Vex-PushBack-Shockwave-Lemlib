#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include <fstream>
#include <sstream>


// Get paths used for pure pursuit

void re_run() {
    
    if (!pros::usd::is_installed()) {
        return;
    }
    
    std::ifstream ifs("dtData.txt");

    if (!ifs.is_open()) {
        return;
    }    
    
    std::string line;
    
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        
        std::stringstream ss(line); // FIXED: was ss(str: line)
        std::string val;
        double left1V, right1V;

        try {
            std::getline(ss, val, ' '); left1V  = std::stod(val);
            std::getline(ss, val, ' '); right1V = std::stod(val);
        } catch (...) {
            continue;
        }

        // FIXED: MotorGroup takes voltage directly, no subscript
        leftMotors.move_voltage(left1V);
        rightMotors.move_voltage(right1V);

        pros::delay(20);
    }

    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);

    ifs.close();
}



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
    middleMotor.move(-600);
    outtake(200);
    //Picking up 3 blocks
    chassis.moveToPose(0, 13, 10, 1000, {.forwards = true, .lead = 0, .minSpeed=30}, false);
    chassis.moveToPose(0, 17, 10, 1000, {.forwards = true, .lead = 0, .minSpeed=50}, false);

    //pros::delay(100);
    chassis.moveToPose(0, 19, -3, 1000, {.forwards = true, .lead = 0, .maxSpeed=30}, false);
    //pros::delay(100);
    chassis.moveToPose(0,27,10,1000,{.forwards=true, .lead=0, .maxSpeed=35},false);
    //pros::delay(100);
    chassis.turnToHeading(-109,500);
    //middle goal scoring
    chassis.moveToPoint(20,37,2000,{.forwards=false,.maxSpeed=65},false);
    chassis.turnToHeading(-113,200);
    limiter.set_value(1);
    outtake(-150);
    pros::delay(150);
    middletake(600);
    pros::delay(200);
    middletake(600);
    pros::delay(1000);
    intake(0);
    outtake(0);
    middletake(0);
    // FIRST TRILAT
    //matchloading
    chassis.moveToPoint(-34,11,2500,{.forwards=true,.maxSpeed=80},false);
    chassis.turnToHeading(-156,500);
    //pros::delay(50);
    matchLoad.set_value(true);
    
    chassis.moveToPoint(-36,0,500,{.forwards=true},false);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtake(200);
    pros::delay(500);
    //long goal scoring
    chassis.turnToHeading(-160,500);
    chassis.moveToPoint(-26,35,1500,{.forwards=false},false);
    matchLoad.set_value(false);
    limiter.set_value(0);
    pros::delay(2500);

}

void right_auton() {
    wing.set_value(0);
    limiter.set_value(1);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtake(200);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,10,500,{.forwards=true,.maxSpeed=40},false);
    chassis.turnToHeading(35,500);
    chassis.moveToPoint(1,17,500,{.forwards=true,.maxSpeed=40},false);
    chassis.turnToHeading(52,500);
    chassis.moveToPoint(4,19,500,{.forwards=true,.maxSpeed=40},false);
    chassis.turnToHeading(25,500);
    chassis.moveToPoint(9,25,500,{.forwards=true,.maxSpeed=40},false);
    chassis.turnToHeading(-40,500);
    chassis.moveToPoint(-3,38,1000,{.forwards=true,.maxSpeed=60},false);
    intakeMotor.move(200);
    middleMotor.move(600);
    outtake(-200);
    pros::delay(2750);
    intakeMotor.move(0);
    middleMotor.move(0);
    outtake(0);
    chassis.moveToPoint(30,8,1500,{.forwards=false,.minSpeed=60},false);
    pros::delay(350);
    chassis.turnToHeading(-175,500);
    matchLoad.set_value(true);
    intakeMotor.move(-200);
    middletake(600);
    outtake(200);
    chassis.moveToPoint(31,-10,1500,{.forwards=true,.minSpeed=30},false);
    pros::delay(500);
    chassis.turnToHeading(-170,500);
    chassis.moveToPoint(35,27,1000,{.forwards=false,.minSpeed=60},false);
    limiter.set_value(0);
    matchLoad.set_value(false);

    /*
    chassis.moveToPoint(33,-12,500,{.forwards=true},false);
    pros::delay(3000);

    chassis.moveToPoint(33,18,500,{.forwards=true},false);
    limiter.set_value(0);
*/
}

void skills_auton() {
    chassis.setPose(0,0,0);
    limiter.set_value(1);
    wing.set_value(0);


    intakeMotor.move(-127);
    chassis.moveToPoint(0,-20,2000, {.forwards=false,.maxSpeed=50});
    chassis.moveToPoint(0,25,2000, {.minSpeed = 80});
}
