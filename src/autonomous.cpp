#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "robot/re_run_helpers.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include <fstream>
#include <sstream>
#include <algorithm>

const double PI = 3.14159265358979323846;


void odom_ekf_run() {
    if (!pros::usd::is_installed()) return;

    std::fstream fs("/usd/dtData.txt", std::ios::in | std::ios::out);
    if (!fs.is_open()) return;

    chassis.setPose(0,0,0);

    std::string line;

    double file_prevx = 0, file_prevy = 0, file_prevt = 0;
    double prevx = 0, prevy = 0, prevt = 0;
    
    double start_time = pros::millis();
    double delta_time = .02;

    double a = .05;

    bool skip_linex_nav = false;
    bool skip_liney_nav = false;
    bool skip_linet_nav = false;

    std::streampos rpos = 0;  // save current read position


    while (std::getline(fs, line)) {

        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string val;
        
        double filex, filey, filet;

        start_time = pros::millis();

        try {
            std::getline(ss, val, ' '); filex = std::stod(val);
            std::getline(ss, val, ' '); filey = std::stod(val);
            std::getline(ss, val, ' '); filet = std::stod(val);
        } catch (...) {
            continue;
        }

        double currentx, currenty, currentt;

        currentx = chassis.getPose().x;
        currenty = chassis.getPose().y;
        currentt = chassis.getPose().theta;

        double file_dx, file_dy, file_dt;
        double dx, dy, dt;

        // can use vlive variable time but for now using fixed 20 ms value instead of 
        // delta_time = pros::millis()-start_time;
        delta_time = .02;

        file_dx = (filex-file_prevx)/delta_time;
        file_dy = (filey-file_prevy)/delta_time;
        file_dt = (filet-file_prevt)/delta_time;
        dx = (currentx-prevx)/delta_time;
        dy = (currenty-prevy)/delta_time;
        dt = (currentt-prevt)/delta_time;

        double future_file_x = filex + delta_time*file_dx;
        double future_file_y = filey + delta_time*file_dy;
        double future_file_t = filet + delta_time*file_dt;

        double future_x = currentx + delta_time*dx;
        double future_y = currenty + delta_time*dy;
        double future_t = currentt + delta_time*dt;

        double dx_diff = std::abs(dx-file_dx);
        double dy_diff = std::abs(dy-file_dy);
        double dt_diff = std::abs(dt-file_dt);

        if (dx_diff > a && std::abs(file_dx) > 0.001) {
                double percent_change = (dx-file_dx)/file_dx;
                percent_change/=2;
                dx = percent_change*file_dx+file_dx;
                future_x = currentx + delta_time*dx;
                skip_linex_nav = true;
        }
        if (dy_diff > a && std::abs(file_dy) > 0.001) {
                double percent_change = (dy-file_dy)/file_dy;
                percent_change/=2;
                dy = percent_change*file_dy+file_dy;
                future_y = currenty + delta_time*dy;
                skip_liney_nav = true;
        }
        if (dt_diff > a && std::abs(file_dt) > 0.001) {
                double percent_change = (dt-file_dt)/file_dt;
                percent_change/=2;
                dt = percent_change*file_dt+file_dt;
                future_t = currentt + delta_time*dt;
                skip_linet_nav = true;
        }

        double fdist, bdist, ldist, rdist;

        fdist = fdist_sens.get_distance();
        bdist = bdist_sens.get_distance();
        rdist = rdist_sens.get_distance();
        ldist = ldist_sens.get_distance();

        double theta = std::fmod(currentt,360.0);
        while (theta < 0) {
            theta += 360.0;
        }

        bool fwall = true, bwall = true, rwall = true, lwall = true;
        
        if (90 > theta && theta > 0) {
            int quadrant = 1;
            if (fdist == 9999) fwall = false;
            if (bdist == 9999) bwall = false;
            if (rdist == 9999) rwall = false;
            if (ldist == 9999) lwall = false;
            // call dist sens pose correction func
        }
        else if (180 > theta && theta > 90) {
            int quadrant = 2;
            if (fdist == 9999) fwall = false;
            if (bdist == 9999) bwall = false;
            if (rdist == 9999) rwall = false;
            if (ldist == 9999) lwall = false;
        }
        else if (270 > theta && theta > 180) {
            int quadrant = 3;
            if (fdist == 9999) fwall = false;
            if (bdist == 9999) bwall = false;
            if (rdist == 9999) rwall = false;
            if (ldist == 9999) lwall = false;
        }
        else if (360 > theta && theta > 270) {
            int quadrant = 0;
            if (fdist == 9999) fwall = false;
            if (bdist == 9999) bwall = false;
            if (rdist == 9999) rwall = false;
            if (ldist == 9999) lwall = false;
        }

        // move the robot
        if (skip_linex_nav) {
            if (skip_liney_nav) {
                if (skip_linet_nav) {
                    chassis.moveToPose(future_x, future_y, future_t,200);
                }
                else {
                    chassis.moveToPose(future_x, future_y, filet,200);
                }
            }
            else {
                if (skip_linet_nav) {
                    chassis.moveToPose(future_x, filey, future_t,200);
                }
                else {
                    chassis.moveToPose(future_x, filey, filet,200);
                }
            }
        }
        else {
            if (skip_liney_nav) {
                if (skip_linet_nav) {
                    chassis.moveToPose(filex, future_y, future_t,200);
                }
                else {
                    chassis.moveToPose(filex, future_y, filet,200);
                }
            }
            else {
                if (skip_linet_nav) {
                    chassis.moveToPose(filex, filey, future_t,200);
                }
                else {
                    chassis.moveToPose(filex, filey, filet,200);
                }
            }
        }

        // end of func resetting work
        file_prevx = filex;
        file_prevy = filey;
        file_prevt = filet;
        prevx = currentx;
        prevy = currenty;
        prevt = currentt;
        skip_linex_nav = false;
        skip_liney_nav = false;
        skip_linet_nav = false;
    }
}


void voltage_re_run() {
    if (!pros::usd::is_installed()) return;
    
    std::ifstream ifs("/usd/dtData.txt");
    if (!ifs.is_open()) return;

    // set odom to same origin as recording
    chassis.setPose(0, 0, 0);
    
    std::string line;

    // separate gains — lateral error is in inches, theta error is in degrees,
    // they live on completely different scales and need independent gains
    const double kP_lateral = 0.3;
    const double kP_theta   = 8.0;  // start here, tune up if heading still drifts
    
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        
        std::stringstream ss(line);
        std::string val;
        double left1V, right1V, expX, expY, expTheta;

        try {
            std::getline(ss, val, ' '); left1V   = std::stod(val);
            std::getline(ss, val, ' '); right1V  = std::stod(val);
            std::getline(ss, val, ' '); expX     = std::stod(val);
            std::getline(ss, val, ' '); expY     = std::stod(val);
            std::getline(ss, val, ' '); expTheta = std::stod(val);
        } catch (...) {
            continue;
        }

        // get actual current position
        double actX     = chassis.getPose().x;
        double actY     = chassis.getPose().y;
        double actTheta = chassis.getPose().theta;

        // calculate positional error
        double errorX     = expX     - actX;
        double errorY     = expY     - actY;
        double errorTheta = expTheta - actTheta;

        // normalize theta error to -180 to 180
        while (errorTheta > 180)  errorTheta -= 360;
        while (errorTheta < -180) errorTheta += 360;

        // calculate how far off we are in the direction the robot is facing
        double thetaRad = actTheta * PI / 180.0;
        double lateralError = errorX * sin(thetaRad) + errorY * cos(thetaRad);

        // correction voltages — separate gains, theta NOT battery-normalized
        double lateralCorrection = kP_lateral * lateralError * 120;
        double thetaCorrection   = kP_theta   * errorTheta;   // no *120, gain absorbs scaling

        // cap corrections so they dont overpower the rerun
        const double maxLateralCorrection = 2000;
        const double maxThetaCorrection   = 3000;  // allow stronger turning correction
        lateralCorrection = std::clamp(lateralCorrection, -maxLateralCorrection, maxLateralCorrection);
        thetaCorrection   = std::clamp(thetaCorrection,   -maxThetaCorrection,   maxThetaCorrection);

        // battery normalization — applied to base replay voltages only, not corrections
        // corrections fight real-world deviation and must not be dampened by battery level
        double battery = pros::battery::get_voltage() / 12000.0;

        // apply rerun voltages with correction and battery normalization
        leftMotors.move_voltage( (left1V  * battery) + lateralCorrection - thetaCorrection);
        rightMotors.move_voltage((right1V * battery) + lateralCorrection + thetaCorrection);

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
    chassis.moveToPose(0, 13, 10, 1000, {.forwards = true, .lead = 0, .minSpeed=30}, false);
    chassis.moveToPose(0, 17, 10, 1000, {.forwards = true, .lead = 0, .minSpeed=50}, false);
    chassis.moveToPose(0, 19, -3, 1000, {.forwards = true, .lead = 0, .maxSpeed=30}, false);
    chassis.moveToPose(0,27,10,1000,{.forwards=true, .lead=0, .maxSpeed=35},false);
    chassis.turnToHeading(-109,500);
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
    chassis.moveToPoint(-34,11,2500,{.forwards=true,.maxSpeed=80},false);
    chassis.turnToHeading(-156,500);
    matchLoad.set_value(true);
    chassis.moveToPoint(-36,0,500,{.forwards=true},false);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtake(200);
    pros::delay(500);
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
}

void skills_auton() {
    chassis.setPose(0,0,0);
    limiter.set_value(1);
    wing.set_value(0);
    intakeMotor.move(-127);
    chassis.moveToPoint(0,-20,2000, {.forwards=false,.maxSpeed=50});
    chassis.moveToPoint(0,25,2000, {.minSpeed = 80});
}