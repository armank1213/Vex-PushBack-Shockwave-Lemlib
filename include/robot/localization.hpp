#pragma once

// Background localization for hand-coded autons. Runs an MCL particle filter
// or a 3-state EKF as a background task, fusing odometry with the 4 perimeter
// distance sensors, and optionally nudging chassis.setPose toward the estimate
// so normal moveTo*/turnTo* calls land on true field-absolute positions.
//
// Correction ON (production):
//   loc::start(fieldX, fieldY, fieldDeg);
//   chassis.moveToPoint(...); chassis.turnToHeading(...);
//   loc::stop();
//
// Correction OFF (testing): the task only estimates, never touching chassis
// pose. Compare loc::estimate() against odom + ground truth.
//   loc::start(24, 24, 0, loc::Method::MCL, /*correct=*/false);
//
// Coords MUST be field-absolute (0,0 = red-side bottom-left, 144").

namespace loc {

enum class Method { MCL, EKF };

struct Estimate {
    double x        = 0;   // inches, field frame
    double y        = 0;
    double theta_deg = 0;
    double var_xy   = 0;   // position variance proxy (in^2); lower = better
    double extra    = 0;   // MCL: effective sample size. EKF: 0.
    Method method   = Method::MCL;
    bool   corrected = false; // did the last tick nudge chassis pose?
};

void start(double field_x_in, double field_y_in, double field_theta_deg,
           Method method = Method::MCL, bool correct = true);
void startHere(Method method = Method::MCL, bool correct = true);
void stop();
bool active();

// Latest filter summary (updated every task tick).
Estimate estimate();

// Discrete one-shot correction. Call ONLY when the robot is stopped at a
// waypoint (right after a blocking move). It waits settle_ms, checks the
// estimate is confident (var_xy < max_var, and for MCL a healthy sample
// size), then hard-snaps x/y and re-seeds the filter — in-task, so the snap
// isn't mis-read as motion. Heading is left on the IMU. Returns true if a
// correction was applied, false if the filter wasn't confident.
// For this pattern start the task with correct=false so the continuous
// nudger stays out of your moves.
bool snapPose(int settle_ms = 200, double max_var = 4.0);

} // namespace loc
