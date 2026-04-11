#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

const double PI = 3.14159265358979323846;
const char* DATA_PATH = "/usd/dtData.txt";

// ─── EKF State ───────────────────────────────────────────────
struct EKFState {
    double x = 0, y = 0, theta = 0;
    double P[3][3] = {{10,0,0},{0,5,0},{0,0,0.3}};
};

static double raycast144(double rx, double ry, double angle) {
    double cx = std::sin(angle);
    double cy = std::cos(angle);
    double t  = 1e9;
    if (cx >  1e-6) t = std::min(t, (144.0 - rx) / cx);
    if (cx < -1e-6) t = std::min(t, (       rx) / (-cx));
    if (cy >  1e-6) t = std::min(t, (144.0 - ry) / cy);
    if (cy < -1e-6) t = std::min(t, (       ry) / (-cy));
    return t;
}

static void ekf_update(EKFState& s,
                       double sensor_world_angle,
                       double sensor_offset,
                       double raw_mm,
                       double R_noise = 3)
{
    if (raw_mm > 1900.0 || raw_mm < 10.0) return;
    double measured = (raw_mm / 25.4) + sensor_offset;
    double expected = raycast144(s.x, s.y, sensor_world_angle);
    double innov = measured - expected;
    if (std::abs(innov) > 14.0) return;

    const double eps = 1e-4;
    double H[3];
    H[0] = (raycast144(s.x + eps, s.y, sensor_world_angle) - expected) / eps;
    H[1] = (raycast144(s.x, s.y + eps, sensor_world_angle) - expected) / eps;
    H[2] = (raycast144(s.x, s.y, sensor_world_angle + eps) - expected) / eps;

    double HP[3] = {0, 0, 0};
    for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++)
            HP[j] += H[k] * s.P[k][j];
    double Sval = R_noise;
    for (int j = 0; j < 3; j++) Sval += HP[j] * H[j];
    if (std::abs(Sval) < 1e-9) return;

    double K[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            K[i] += s.P[i][k] * H[k];
    for (int i = 0; i < 3; i++) K[i] /= Sval;

    s.x     += K[0] * innov;
    s.y     += K[1] * innov;
    s.theta += K[2] * innov;

    double newP[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double IKH_row[3] = {-K[i]*H[0], -K[i]*H[1], -K[i]*H[2]};
            IKH_row[i] += 1.0;
            newP[i][j] = 0;
            for (int k = 0; k < 3; k++)
                newP[i][j] += IKH_row[k] * s.P[k][j];
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            s.P[i][j] = newP[i][j];
}
// ─────────────────────────────────────────────────────────────


/*void odom_ekf_run() {
    if (!pros::usd::is_installed()) {
        controller.print(0, 0, "FAIL: no SD card  ");
        pros::delay(1000);
        return;
    }

    std::ifstream fs(DATA_PATH);
    if (!fs.is_open()) {
        controller.print(0, 0, "FAIL: no file     ");
        pros::delay(1000);
        return;
    }

    int lineCount = 0;
    std::string countLine;
    while (std::getline(fs, countLine)) if (!countLine.empty()) lineCount++;
    fs.clear(); fs.seekg(0);

    controller.print(0, 0, "Lines: %d         ", lineCount);
    pros::delay(500);

    if (lineCount == 0) {
        controller.print(0, 0, "FAIL: empty file  ");
        pros::delay(1000);
        return;
    }

    chassis.setPose(0, 0, 0);

    // ── Tuning ────────────────────────────────────────────────────
    const int    WAYPOINT_SKIP     = 8;
    const int    MIN_SPEED         = 70;
    const int    MAX_SPEED         = 110;
    const float  EARLY_EXIT_MAX    = 6.0f;
    const double TURN_DETECT_DEG   = 25.0;
    const double TURN_RATIO_THRESH = 8.0;
    const double SHORT_MOVE_IN     = 4.0;
    const double TURN_THRESHOLD_DEG = 15.0;
    const int    TURN_TIMEOUT_MS   = 500;

    // ── Dynamic timing tuning ─────────────────────────────────────
    // Recorded time per segment × this multiplier = motion timeout.
    // 1.6 = 60% buffer on top of recorded time.
    // Raise to 2.0 if slow segments time out early.
    const double TIMEOUT_MULTIPLIER = 1.6;

    // Your robot's real top speed in inches/sec at full power.
    // Measure: drive full speed, time 1 second, measure distance.
    // Typical 600rpm VEX drive ≈ 40-55 in/s.
    const double MAX_SPEED_IPS     = 48.0;

    // Sensor offsets (inches from robot center to sensor face)
    const double FRONT_OFFSET = 7.75;
    const double BACK_OFFSET  = 9.0;
    const double LEFT_OFFSET  = 7.5;
    const double RIGHT_OFFSET = 7.5;

    EKFState ekf;
    ekf.x = 0; ekf.y = 0; ekf.theta = 0;
    const double Q[3][3] = {{0.5,0,0},{0,0.3,0},{0,0,0.005}};
    lemlib::Pose prevPose = chassis.getPose();

    std::string line;
    int step = 0;
    int lastWaypointStep = 0;
    double lastHeading = 0;

    bool prev_l1        = false;
    bool prev_right     = false;
    bool prev_x         = false;
    bool wingState      = false;
    bool matchloadState = false;
    bool limiterState   = true;

    while (std::getline(fs, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string val;
        double filex, filey, filet;
        int r1=0,r2=0,y_btn=0,b_btn=0,l2=0,l1=0,right_btn=0,x_btn=0;

        try {
            std::getline(ss, val, ' '); filex     = std::stod(val);
            std::getline(ss, val, ' '); filey     = std::stod(val);
            std::getline(ss, val, ' '); filet     = std::stod(val);
            std::getline(ss, val, ' '); r1        = std::stoi(val);
            std::getline(ss, val, ' '); r2        = std::stoi(val);
            std::getline(ss, val, ' '); y_btn     = std::stoi(val);
            std::getline(ss, val, ' '); b_btn     = std::stoi(val);
            std::getline(ss, val, ' '); l2        = std::stoi(val);
            std::getline(ss, val, ' '); l1        = std::stoi(val);
            std::getline(ss, val, ' '); right_btn = std::stoi(val);
            std::getline(ss, val, ' '); x_btn     = std::stoi(val);
        } catch (...) { step++; continue; }

        if (step == 0) {
            controller.print(0, 0, "OK %.1f %.1f %.1f", filex, filey, filet);
            pros::delay(100);
        }

        // ── Mechanism replay ─────────────────────────────────────
        if (r1) {
            outtake(200); intake(200); middletake(600); limiter.set_value(1);
        } else if (r2) {
            intake(200); middletake(600); outtake(200); limiter.set_value(0);
        } else if (y_btn) {
            limiter.set_value(0); outtake(-200); intake(200); middletake(600);
        } else if (b_btn) {
            limiter.set_value(0); outtake(-55); intake(200); middletake(600);
        } else if (l2) {
            outtake(-200); intake(-200); middletake(-600);
        } else {
            intake(0); outtake(0); middletake(0);
        }
        if (l1 && !prev_l1) { wingState = !wingState; wing.set_value(wingState); }
        prev_l1 = l1;
        if (right_btn && !prev_right) { matchloadState = !matchloadState; matchLoad.set_value(matchloadState); }
        prev_right = right_btn;
        if (x_btn && !prev_x) { limiterState = !limiterState; limiter.set_value(limiterState); limiter_light.set_led_pwm(limiterState ? 100 : 0); }
        prev_x = x_btn;
        // ─────────────────────────────────────────────────────────

        // ── EKF predict ──────────────────────────────────────────
        lemlib::Pose currPose = chassis.getPose();
        double dx     = currPose.x - prevPose.x;
        double dy     = currPose.y - prevPose.y;
        double dtheta = (currPose.theta - prevPose.theta) * PI / 180.0;
        prevPose = currPose;
        ekf.x += dx; ekf.y += dy; ekf.theta += dtheta;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ekf.P[i][j] += Q[i][j];

        // ── EKF update ───────────────────────────────────────────
        ekf_update(ekf, ekf.theta,            FRONT_OFFSET, fdist_sens.get());
        ekf_update(ekf, ekf.theta + PI,       BACK_OFFSET,  bdist_sens.get());
        ekf_update(ekf, ekf.theta - PI / 2.0, LEFT_OFFSET,  ldist_sens.get());
        ekf_update(ekf, ekf.theta + PI / 2.0, RIGHT_OFFSET, rdist_sens.get());

        if (ekf.P[0][0] < 0.5 && ekf.P[1][1] < 0.5) {
            chassis.setPose(ekf.x, ekf.y, ekf.theta * 180.0 / PI, false);
            prevPose = chassis.getPose();
        }
        // ─────────────────────────────────────────────────────────

        if (step % WAYPOINT_SKIP != 0) { step++; continue; }

        double actX     = chassis.getPose().x;
        double actY     = chassis.getPose().y;
        double actTheta = chassis.getPose().theta;

        double fx   = filex - actX;
        double fy   = filey - actY;
        double dist = std::sqrt(fx*fx + fy*fy);

        double thetaRad = actTheta * PI / 180.0;
        double dot      = fx * std::sin(thetaRad) + fy * std::cos(thetaRad);
        bool goingBackwards = (dist > 1.0) && (dot < 0);

        double headingErr = filet - actTheta;
        while (headingErr >  180) headingErr -= 360;
        while (headingErr < -180) headingErr += 360;

        // ── Dynamic timeout + speed from recorded cadence ─────────
        int    framesDelta     = step - lastWaypointStep;
        int    recordedTimeMs  = framesDelta * 20;
        int    dynamicTimeout  = std::max(200, (int)(recordedTimeMs * TIMEOUT_MULTIPLIER));

        // Speed the driver was moving during this segment (in/s)
        double recordedSpeedIPS = (recordedTimeMs > 0)
                                  ? (dist / (recordedTimeMs / 1000.0))
                                  : 20.0;

        // Map to 0-127 motor speed, clamped to [MIN_SPEED, MAX_SPEED]
        int dynamicMaxSpeed = (int)(recordedSpeedIPS / MAX_SPEED_IPS * 127.0);
        dynamicMaxSpeed = std::max(MIN_SPEED, std::min(MAX_SPEED, dynamicMaxSpeed));

        // earlyExit scales with speed — fast = exit earlier, slow = precise
        float earlyExit = std::min(EARLY_EXIT_MAX,
                          (float)(dist * 0.4 * (recordedSpeedIPS / MAX_SPEED_IPS)));
        earlyExit = std::max(0.5f, earlyExit);

        lastWaypointStep = step;
        // ─────────────────────────────────────────────────────────

        double turnRatio = (dist > 0.5) ? (std::abs(headingErr) / dist) : 999.0;
        bool isPureTurn  = (std::abs(headingErr) > TURN_DETECT_DEG) &&
                           (turnRatio > TURN_RATIO_THRESH || dist < SHORT_MOVE_IN);

        if (isPureTurn) {
            // ── IN-PLACE TURN ─────────────────────────────────────
            // Scan ahead to find the final heading of this turn
            // sequence, then fire ONE clean blocking turn to it.
            double finalTurnHeading = filet;
            std::streampos savedPos = fs.tellg();
            std::string peekLine;

            while (std::getline(fs, peekLine)) {
                if (peekLine.empty()) continue;
                std::stringstream ps(peekLine);
                std::string pv;
                double px, py, pt;
                try {
                    std::getline(ps, pv, ' '); px = std::stod(pv);
                    std::getline(ps, pv, ' '); py = std::stod(pv);
                    std::getline(ps, pv, ' '); pt = std::stod(pv);
                } catch (...) { continue; }

                double pdx    = px - actX;
                double pdy    = py - actY;
                double pdist  = std::sqrt(pdx*pdx + pdy*pdy);
                double pherr  = pt - actTheta;
                while (pherr >  180) pherr -= 360;
                while (pherr < -180) pherr += 360;
                double pratio = (pdist > 0.5) ? (std::abs(pherr) / pdist) : 999.0;
                bool stillTurning = (std::abs(pherr) > TURN_DETECT_DEG) &&
                                    (pratio > TURN_RATIO_THRESH || pdist < SHORT_MOVE_IN);

                if (stillTurning) finalTurnHeading = pt;
                else break;
            }
            fs.clear();
            fs.seekg(savedPos);

            // Single blocking turn to end of full turn sequence
            int turnTimeout = std::max(600, dynamicTimeout);
            chassis.turnToHeading(finalTurnHeading, turnTimeout,
                                  {.maxSpeed = 90},
                                  false);

        } else if (dist < SHORT_MOVE_IN) {
            // ── SHORT PRECISION MOVE ──────────────────────────────
            chassis.moveToPoint(filex, filey, dynamicTimeout,
                                {.forwards = !goingBackwards,
                                 .maxSpeed = std::min(80, dynamicMaxSpeed),
                                 .minSpeed = 0},
                                false);

        } else {
            // ── LONG STRAIGHT / SWEEPING ARC ─────────────────────
            chassis.moveToPoint(filex, filey, dynamicTimeout,
                                {.forwards       = !goingBackwards,
                                 .maxSpeed       = dynamicMaxSpeed,
                                 .minSpeed       = MIN_SPEED,
                                 .earlyExitRange = earlyExit},
                                true);
        }

        lastHeading = filet;
        step++;
    }

    // ── Settle and final heading fix ──────────────────────────────
    chassis.waitUntilDone();

    double actThetaFinal = chassis.getPose().theta;
    double finalErr = lastHeading - actThetaFinal;
    while (finalErr >  180) finalErr -= 360;
    while (finalErr < -180) finalErr += 360;
    if (std::abs(finalErr) > TURN_THRESHOLD_DEG)
        chassis.turnToHeading(lastHeading, TURN_TIMEOUT_MS, {}, false);

    controller.print(0, 0, "Done: %d steps    ", step);
    fs.close();
}*/


void angular_tuning() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000, {}, false);
}

void lateral_tuning() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20, 1000, {}, false);
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
    chassis.setPose(0, 0, 0);
}
void park_auton() {
chassis.setPose(0,0,0);
    wing.set_value(0);
    limiter.set_value(1);
    //Matchload No.1
    chassis.moveToPoint(-2,41,2000,{.forwards=true},false);
    chassis.turnToHeading(91,500,{});
    matchLoad.set_value(false);
    chassis.moveToPoint(14,0,2000,{.forwards=true},false);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    pros::delay(2000);}
    


void skills_auton() {
    //Initial bot state
    chassis.setPose(0,0,0);
    wing.set_value(0);
    limiter.set_value(1);
    matchLoad.set_value(false);
    //Movement to line up for park
    chassis.moveToPoint(-2,41,2000,{.forwards=true},false);
    chassis.turnToHeading(91,500,{});
    //Drive into park
    chassis.moveToPoint(18,0,2000,{.forwards=true, .maxSpeed=110, .minSpeed=95},false); 
    // ^ Takes advantage of unintended response during lack of theta value, Also adjusted the speed value to improve consistency. 
    //Pick up blocks that dont clear in park
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtakeMotor.move(-200);
    pros::delay(2000);


    /*chassis.setPose(0,0,0);
    wing.set_value(0);
    limiter.set_value(1);
    //Matchload No.1
    chassis.moveToPoint(-2,44,2000,{.forwards=true},false);
    chassis.turnToHeading(91,500,{});
    matchLoad.set_value(true);
    chassis.moveToPoint(12,43,2000,{.forwards=true},false);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtakeMotor.move(-200);
    chassis.moveToPoint(14,43,2000,{.forwards=true},false);
    outtakeMotor.move(-200);
    pros::delay(4000);
    intakeMotor.move(0);
    middleMotor.move(0);
    outtakeMotor.move(0);
    //chassis.moveToPoint(-7,43,1000,{.forwards=false},false);
    matchLoad.set_value(false);
    chassis.moveToPoint(-18,41,1500,{.forwards=false},false);
    limiter.set_value(0);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtakeMotor.move(-200);
    pros::delay(3500);
    intakeMotor.move(0);
    middleMotor.move(0);
    outtakeMotor.move(0);
    chassis.moveToPose(18,26,180, 1500,{.forwards=true, .lead=.2, .maxSpeed=110, .minSpeed=95},false); // og y 16
    */
    
    /*chassis.turnToHeading(177,200,{});
    chassis.moveToPoint(-7,27,1000,{.forwards=true},false);
    chassis.turnToHeading(267,1000,{});
    chassis.moveToPose(-89,22,267,3000,{.forwards=true},false);
    chassis.turnToHeading(355,500,{});
    chassis.moveToPoint(-90,41,2250,{.forwards = true},false);
    chassis.turnToHeading(272,500,{});
    chassis.moveToPoint(-79,41,2250,{.forwards = false},false);
    limiter.set_value(0);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtakeMotor.move(-200);
    matchLoad.set_value(true);
    pros::delay(2000);
    chassis.moveToPose(-113,38,267,4000, {.forwards = true, .maxSpeed=35},false);
    limiter.set_value(1);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtakeMotor.move(-200);
    chassis.moveToPoint(-114,38,2000, {.forwards = true},false);
    chassis.moveToPoint(-115,38,2000, {.forwards = true},false);
    pros::delay(4000);
    intakeMotor.move(0);
    middleMotor.move(0);
    outtakeMotor.move(0);
    chassis.moveToPose(-76,40,267,3000,{.forwards=false},false);
    matchLoad.set_value(false);
    limiter.set_value(0);
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtakeMotor.move(-200);
    pros::delay(4000);
    //chassis.moveToPoint(-80,43,2000,{.forwards=false},false);
*/
}