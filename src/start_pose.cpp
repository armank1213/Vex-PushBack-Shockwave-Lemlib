#include "robot/start_pose.hpp"
#include "robot/hardware.hpp"
#include "robot/field_model.hpp"
#include "lemlib/pose.hpp"

#include <cmath>

namespace {

using field::FIELD_IN;
using field::MM_PER_IN;

// Center-to-wall scalar offsets, derived from the measured 2-D sensor
// mounts in field_model.hpp. When the robot is square to a flat wall (the
// assumption of this cardinal-snap solver) only the along-axis component
// matters, which is exactly along_offset(mount).
const double FRONT_OFFSET = field::along_offset(field::FRONT);
const double BACK_OFFSET  = field::along_offset(field::BACK);
const double LEFT_OFFSET  = field::along_offset(field::LEFT);
const double RIGHT_OFFSET = field::along_offset(field::RIGHT);

// VEX V5 Distance sensor spec (kb.vex.com / SIGBots wiki):
//   range 20-2000 mm; get() returns 9999 when NO object is detected.
//   confidence 0..63, but ONLY meaningful when distance > 200 mm.
//   accuracy: +/-15 mm below 200 mm, ~5% above 200 mm.
constexpr int    CONF_GATE    = 40;
constexpr double DIST_MIN_MM  = 20.0;    // sensor minimum
constexpr double DIST_MAX_MM  = 2000.0;  // sensor maximum (9999 => no object)
constexpr double CONF_DIST_MM = 200.0;   // confidence only valid above this

// A reading is usable if it is in physical range AND either close enough
// that the spec guarantees +/-15 mm regardless of the (unavailable)
// confidence, or far enough that the confidence value is meaningful and
// passes the gate. The old code required confidence > gate at ALL ranges,
// which silently discarded any wall closer than ~8" (200 mm).
bool reading_valid(int raw_mm, int confidence) {
    if (raw_mm < DIST_MIN_MM || raw_mm > DIST_MAX_MM) return false;
    if (raw_mm <= CONF_DIST_MM)                       return true;
    return confidence >= CONF_GATE;
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

    // Average the per-axis estimates. With the gate fixed, BOTH the near
    // wall and (when in range) the far wall on each axis contribute, so a
    // robot near a corner gets a 2-sensor average per axis. If an axis has
    // NO valid wall (robot too far from both walls on that axis), we cannot
    // know it from sensors — fall back to field center rather than inventing
    // a number. Caller should place the robot near a corner so this branch
    // does not trigger.
    double x = (x_count > 0) ? (x_sum / x_count) : (FIELD_IN / 2.0);
    double y = (y_count > 0) ? (y_sum / y_count) : (FIELD_IN / 2.0);

    // Clamp into the field so a single bad frame can't return a wild pose.
    if (x < 0.0)        x = 0.0;
    if (x > FIELD_IN)   x = FIELD_IN;
    if (y < 0.0)        y = 0.0;
    if (y > FIELD_IN)   y = FIELD_IN;

    return lemlib::Pose(x, y, theta_deg);
}
