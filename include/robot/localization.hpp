#pragma once

// Background localization for hand-coded autons. Runs EITHER an MCL
// particle filter or a 3-state EKF as a background task, fusing odometry
// with the 4 perimeter distance sensors, and (optionally) nudging
// chassis.setPose toward the estimate so your normal moveTo*/turnTo*
// calls land on true FIELD-ABSOLUTE positions.
//
// Usage (correction ON — production):
//   loc::start(fieldX, fieldY, fieldDeg);          // MCL, corrects pose
//   chassis.moveToPoint(...); chassis.turnToHeading(...);
//   loc::stop();
//
// Usage (correction OFF — testing): the task only ESTIMATES; it does not
// touch chassis pose. Compare loc::estimate() against LemLib odom and
// physical ground truth to see if the filter tracks reality.
//   loc::start(24, 24, 0, loc::Method::MCL, /*correct=*/false);
//
// Coords MUST be field-absolute (0,0 = red-side bottom-left, 144").

namespace loc {

enum class Method { MCL, EKF };

struct Estimate {
    double x        = 0;   // inches, field frame
    double y        = 0;
    double theta_deg = 0;
    double var_xy   = 0;   // position variance proxy (in^2); lower = more confident
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

} // namespace loc
