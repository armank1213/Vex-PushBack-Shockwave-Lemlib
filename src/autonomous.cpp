#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "robot/oekf_rerun.hpp" // IWYU pragma: keep
#include "robot/localization.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep

// EKF replay lives in src/oekf_rerun.cpp. See robot/oekf_rerun.hpp.
//
// MCL pose-correction for hand-coded autons: bracket your moves with
// loc::start(fieldX, fieldY, fieldDeg) ... loc::stop(). While running, a
// background task fuses odom + distance sensors and trims drift so your
// moveTo*/turnTo* calls land on true field positions. Coords MUST be
// field-absolute (0,0 = red bottom-left, 144"). See corrected_auton().

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
    pros::delay(2000);
}


// ── LOCALIZATION TEST ─────────────────────────────────────────────────
// Place the robot at a MEASURED field-absolute pose (edit TX/TY/TDEG to
// match). Put it near a CORNER so at least two walls are within ~75" —
// the distance sensors max out past that range. Correction is OFF, so the
// filter only ESTIMATES; compare its estimate to where the robot really is.
//
//   od  = LemLib wheel odometry (x y theta)
//   est = filter estimate        (x y theta)
//   var = position variance (shrinks as it gets confident); x = N_eff (MCL)
//
// Tests:
//  1. Robot still at (TX,TY): est should sit at (TX,TY), var should shrink.
//     If est drifts off → sensor offsets or field size (144 vs 140.43) wrong.
//  2. Seed wrong on purpose (set TX 12" off): est should converge back to
//     the true spot using the walls. Proves the update step works.
//  3. Hand-push the robot: est should track the new position.
//  4. Swap M to loc::Method::EKF and repeat — compare EKF vs MCL.
void localization_test() {
    const double TX = 56.5, TY = 24, TDEG = 0;     // measured start pose
    const loc::Method M = loc::Method::MCL;      // swap to ::EKF to test EKF
    loc::start(TX, TY, TDEG, M, /*correct=*/false);

    while (true) {
        lemlib::Pose p = chassis.getPose();
        loc::Estimate e = loc::estimate();
        controller.print(0, 0, "od  %4d %4d %4d", (int)p.x, (int)p.y, (int)p.theta);
        pros::delay(60);
        controller.print(1, 0, "est %4d %4d %4d", (int)e.x, (int)e.y, (int)e.theta_deg);
        pros::delay(60);
        controller.print(2, 0, "var %5.1f n%3d ", e.var_xy, (int)e.extra);
        pros::delay(400);
    }
}


// ── EXAMPLE: normal hand-coded auton WITH MCL correction running ──────
// Everything here is field-absolute (0,0 = red-side bottom-left, 144").
// Coords reference real field landmarks (center goal ~70,70; long goals
// at y≈23 and y≈117). Place the robot at the start pose below, then run.
void corrected_auton() {
    // Seed pose + spawn the MCL correction task. Robot starts at field
    // (24, 24) facing +Y (toward blue side).
    loc::start(56.5, 23, 0);

    wing.set_value(0);
    limiter.set_value(1);
    matchLoad.set_value(false);

    // Plain LemLib moves — MCL trims drift underneath, no special calls.
    chassis.moveToPoint(70, 24, 2000, {.forwards = true}, false);   // to bottom long goal
    chassis.turnToHeading(0, 600, {}, false);
    chassis.moveToPose(70, 60, 0, 2500, {.forwards = true}, false); // up toward center goal
    intakeMotor.move(-200);
    middleMotor.move(-600);
    outtakeMotor.move(-200);
    pros::delay(1000);

    loc::stop();   // always stop the task before the function returns
}


void skills_auton() {
    //Initial bot state
    chassis.setPose(0,0,0);
    // To MCL-correct this auton: (1) replace setPose above with
    //   loc::start(<field start x>, <field start y>, <field heading>);
    // (2) rewrite the waypoints below in FIELD-ABSOLUTE coords; and
    // (3) call loc::stop(); before the function returns.
    // As written (relative coords from 0,0,0) the sensor gate rejects
    // everything, so correction would be a no-op.
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
}
