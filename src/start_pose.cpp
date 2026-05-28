#include "robot/start_pose.hpp"
#include "robot/hardware.hpp"
#include "lemlib/pose.hpp"

#include <cmath>

namespace {

constexpr double FIELD_IN = 144.0;
constexpr double MM_PER_IN = 25.4;

// Robot center -> sensor face along sensor pointing direction.
constexpr double FRONT_OFFSET = 7.75;
constexpr double BACK_OFFSET  = 9.00;
constexpr double LEFT_OFFSET  = 7.50;
constexpr double RIGHT_OFFSET = 7.50;

// VEX Distance confidence is 0..63. Same gate as the EKF replay uses.
constexpr int CONF_GATE = 40;

// Distance reading validity window (mm).
constexpr double DIST_MIN_MM = 10.0;
constexpr double DIST_MAX_MM = 1900.0;

bool reading_valid(int raw_mm, int confidence) {
    return confidence > CONF_GATE
        && raw_mm > DIST_MIN_MM
        && raw_mm < DIST_MAX_MM;
}

// Convert raw_mm to robot-center -> wall distance (inches).
double to_center_distance_in(int raw_mm, double sensor_offset_in) {
    return (raw_mm / MM_PER_IN) + sensor_offset_in;
}

} // anonymous namespace


lemlib::Pose determine_start_pose(double theta_deg) {
    // Snap heading to nearest cardinal: 0, 90, 180, 270. Reading
    // perimeter walls only makes sense when the robot is square-on.
    // theta_deg input is taken as-is for the returned pose, but the
    // sensor-to-wall mapping uses the snapped quadrant.
    int quadrant = ((int)std::round(theta_deg / 90.0)) & 3;
    // quadrant 0: front=+Y, back=-Y, left=-X, right=+X
    // quadrant 1: front=+X, back=-X, left=+Y, right=-Y
    // quadrant 2: front=-Y, back=+Y, left=+X, right=-X
    // quadrant 3: front=-X, back=+X, left=-Y, right=+Y

    int f_mm   = fdist_sens.get();
    int b_mm   = bdist_sens.get();
    int l_mm   = ldist_sens.get();
    int r_mm   = rdist_sens.get();
    int f_conf = fdist_sens.get_confidence();
    int b_conf = bdist_sens.get_confidence();
    int l_conf = ldist_sens.get_confidence();
    int r_conf = rdist_sens.get_confidence();

    // For each sensor that's valid, compute the field-coord estimate
    // it implies along its pointing axis. Then average within-axis.
    double x_sum = 0, x_count = 0;
    double y_sum = 0, y_count = 0;

    auto fuse_x = [&](double est_x) { x_sum += est_x; x_count += 1; };
    auto fuse_y = [&](double est_y) { y_sum += est_y; y_count += 1; };

    // Helper: a sensor pointing toward "+Y wall" reads (FIELD - y).
    //   y_estimate = FIELD - center_distance
    // A sensor pointing toward "-Y wall" reads y.
    //   y_estimate = center_distance
    // Same pattern on X axis.

    if (reading_valid(f_mm, f_conf)) {
        double d = to_center_distance_in(f_mm, FRONT_OFFSET);
        switch (quadrant) {
            case 0: fuse_y(FIELD_IN - d); break;
            case 1: fuse_x(FIELD_IN - d); break;
            case 2: fuse_y(d);            break;
            case 3: fuse_x(d);            break;
        }
    }
    if (reading_valid(b_mm, b_conf)) {
        double d = to_center_distance_in(b_mm, BACK_OFFSET);
        switch (quadrant) {
            case 0: fuse_y(d);            break;
            case 1: fuse_x(d);            break;
            case 2: fuse_y(FIELD_IN - d); break;
            case 3: fuse_x(FIELD_IN - d); break;
        }
    }
    if (reading_valid(l_mm, l_conf)) {
        double d = to_center_distance_in(l_mm, LEFT_OFFSET);
        switch (quadrant) {
            case 0: fuse_x(d);            break;
            case 1: fuse_y(FIELD_IN - d); break;
            case 2: fuse_x(FIELD_IN - d); break;
            case 3: fuse_y(d);            break;
        }
    }
    if (reading_valid(r_mm, r_conf)) {
        double d = to_center_distance_in(r_mm, RIGHT_OFFSET);
        switch (quadrant) {
            case 0: fuse_x(FIELD_IN - d); break;
            case 1: fuse_y(d);            break;
            case 2: fuse_x(d);            break;
            case 3: fuse_y(FIELD_IN - d); break;
        }
    }

    double x = (x_count > 0) ? (x_sum / x_count) : (FIELD_IN / 2.0);
    double y = (y_count > 0) ? (y_sum / y_count) : (FIELD_IN / 2.0);

    return lemlib::Pose(x, y, theta_deg);
}
