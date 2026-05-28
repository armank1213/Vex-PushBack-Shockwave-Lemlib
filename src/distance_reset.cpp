#include "robot/distance_reset.hpp"
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>

using namespace std;
using namespace lemlib;

double angle_error(double angle_a, double angle_b) {

    double diff = fabs(angle_a - angle_b);

    double mod_diff = fmod(diff, 360.0);
    if (mod_diff < 0) {
        mod_diff += 360.0;
    }

    if (mod_diff > 180.0) {
        return 360.0 - mod_diff;
    } else {
        return abs(mod_diff);
    }
}

namespace {

const double FIELD_WIDTH    = 3657.6; // mm
const double FIELD_HEIGHT   = 3657.6; // mm  (unused, kept for symmetry)
const double ANGLE_TOLERANCE = 0.5;   // degrees
const double DEG_TO_RAD     = M_PI / 180.0;
const double WHEEL_CIRCUMFERENCE = Omniwheel::NEW_2 * M_PI; // mm

struct SensorOffsets {
    double left  = 0.0;
    double right = 0.0;
    double back  = 0.0;
};

const SensorOffsets sensor_offsets;

} // anonymous namespace

// One-pass pose-update step using vertical odom pod + wall correction
// from the three perimeter distance sensors. Intended to be called
// repeatedly by a higher-level task. Returns the new (x, y).
//
// State is held in function-local statics so that hardware (the
// Rotation/IMU/Distance globals) is fully constructed before any
// sensor call happens — avoids the static-init order fiasco.
std::pair<double, double> reset_distance() {
    static bool initialized = false;
    static double x = -54.422;
    static double y =  -9.185;
    static double theta = 0.0;
    static double x_prev = x;
    static double y_prev = y;
    static double prev_rotation = 0.0;

    if (!initialized) {
        prev_rotation = vertical_rotation.get_position();
        initialized   = true;
    }

    double current_rotation = vertical_rotation.get_position();
    double delta_rotation   = current_rotation - prev_rotation;
    prev_rotation           = current_rotation;

    double delta_degrees   = delta_rotation / 100.0;
    double wheel_rotations = delta_degrees / 360.0;
    double delta_s         = wheel_rotations * WHEEL_CIRCUMFERENCE;

    theta = imu.get_heading();

    // C math trig wants radians; imu.get_heading() returns degrees.
    double theta_rad = theta * DEG_TO_RAD;
    x = x + delta_s * cos(theta_rad);
    y = y + delta_s * sin(theta_rad);

    // distance
    double DL = ldist_sens.get_distance();
    double DR = rdist_sens.get_distance();
    double DB = bdist_sens.get_distance();

    // Wall snap — only when heading aligns with that wall's normal,
    // within ANGLE_TOLERANCE degrees.
    if (DL > 0 && angle_error(theta, 90.0) < ANGLE_TOLERANCE) {
        x = DL - sensor_offsets.left;
    }
    if (DR > 0 && angle_error(theta, 270.0) < ANGLE_TOLERANCE) {
        x = FIELD_WIDTH - DR + sensor_offsets.right;
    }
    if (DB > 0 && angle_error(theta, 180.0) < ANGLE_TOLERANCE) {
        y = DB - sensor_offsets.back;
    }

    // in case of jumps or tipping (exponential smoothing)
    x = x_prev + 0.3 * (x - x_prev);
    y = y_prev + 0.3 * (y - y_prev);

    x_prev = x;
    y_prev = y;

    return {x, y};
}
