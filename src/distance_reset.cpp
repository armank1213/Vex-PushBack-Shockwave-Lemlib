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

// Initializing 

Pose pose = Pose(-54.422, -9.185, 0);

double x = pose.x;
double y = pose.y;
double theta = pose.theta;

double prev_rotation = vertical_rotation.get_position();

const double FIELD_WIDTH = 3657.6; // mm
const double FIELD_HEIGHT = 3657.6; // mm

const double ANGLE_TOLERANCE = 0.5; // degrees

struct SensorOffsets {
    double left;
    double right;
    double back;
};

SensorOffsets sensor_offsets = {0.0, 0.0, 0.0};

double x_prev = x;
double y_prev = y;

const double WHEEL_CIRCUMFERENCE = Omniwheel::NEW_2 * M_PI; // mm

std::pair<double, double> reset_distance() {

    while (true) {

        double current_rotation = vertical_rotation.get_position();
        double delta_rotation = current_rotation - prev_rotation;
        double prev_rotation = current_rotation;

        
        double delta_degrees = delta_rotation / 100.0;
        double wheel_rotations = delta_degrees / 360.0;
        double delta_s = wheel_rotations * WHEEL_CIRCUMFERENCE;


        theta = imu.get_heading();

        x = x + delta_s * cos(theta);
        y = y + delta_s + sin(theta);

        //distance
        double DL = ldist_sens.get_distance();
        double DR = rdist_sens.get_distance();
        double DB = bdist_sens.get_distance();

        // left wall

        if (DL > 0 && fabs(angle_error(theta, 90.0) < ANGLE_TOLERANCE)) {
            x = DL - sensor_offsets.left;
        }
        if (DR > 0 && fabs(angle_error(theta, 270.0) < ANGLE_TOLERANCE)) {
            x = FIELD_WIDTH - DR + sensor_offsets.right;
        }
        if (DB > 0 && fabs(angle_error(theta, 180.0) < ANGLE_TOLERANCE)) {
            y = DB - sensor_offsets.back;
        }

        // in case of jumps or tipping (exponential smoothing)

        x = x_prev + 0.3 * (x - x_prev);
        y = y_prev + 0.3 * (y - y_prev);
        
        x_prev = x;
        y_prev = y;

        return {x,y};

        pros::delay(20);
    }
}

