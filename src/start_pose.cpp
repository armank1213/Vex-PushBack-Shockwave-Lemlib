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

    // Read each sensor once (mm + confidence are separate I2C round-trips).
    // Index order: 0=FRONT, 1=BACK, 2=LEFT, 3=RIGHT.
    const field::SensorMount* mounts[4] = { &field::FRONT, &field::BACK,
                                            &field::LEFT,  &field::RIGHT };
    const double offsets[4] = { FRONT_OFFSET, BACK_OFFSET, LEFT_OFFSET, RIGHT_OFFSET };
    const int    mm[4]   = { fdist_sens.get(), bdist_sens.get(),
                             ldist_sens.get(), rdist_sens.get() };
    const int    conf[4] = { fdist_sens.get_confidence(), bdist_sens.get_confidence(),
                             ldist_sens.get_confidence(), rdist_sens.get_confidence() };

    // Per-sensor implied field coordinate under the cardinal-snap assumption.
    // A sensor pointing at the "+Y wall" reads (FIELD - y); at the "-Y wall"
    // reads y; same pattern on X. Which world axis a sensor measures depends
    // on the quadrant.
    bool   valid[4] = { false, false, false, false };
    char   axis[4]  = { 'x', 'x', 'x', 'x' };
    double coord[4] = { 0.0, 0.0, 0.0, 0.0 };

    for (int i = 0; i < 4; ++i) {
        if (!reading_valid(mm[i], conf[i])) continue;
        const double d = to_center_distance_in(mm[i], offsets[i]);
        valid[i] = true;
        switch (i) {
            case 0: // FRONT
                switch (quadrant) {
                    case 0: axis[i] = 'y'; coord[i] = FIELD_IN - d; break;
                    case 1: axis[i] = 'x'; coord[i] = FIELD_IN - d; break;
                    case 2: axis[i] = 'y'; coord[i] = d;            break;
                    case 3: axis[i] = 'x'; coord[i] = d;            break;
                } break;
            case 1: // BACK
                switch (quadrant) {
                    case 0: axis[i] = 'y'; coord[i] = d;            break;
                    case 1: axis[i] = 'x'; coord[i] = d;            break;
                    case 2: axis[i] = 'y'; coord[i] = FIELD_IN - d; break;
                    case 3: axis[i] = 'x'; coord[i] = FIELD_IN - d; break;
                } break;
            case 2: // LEFT
                switch (quadrant) {
                    case 0: axis[i] = 'x'; coord[i] = d;            break;
                    case 1: axis[i] = 'y'; coord[i] = FIELD_IN - d; break;
                    case 2: axis[i] = 'x'; coord[i] = FIELD_IN - d; break;
                    case 3: axis[i] = 'y'; coord[i] = d;            break;
                } break;
            case 3: // RIGHT
                switch (quadrant) {
                    case 0: axis[i] = 'x'; coord[i] = FIELD_IN - d; break;
                    case 1: axis[i] = 'y'; coord[i] = d;            break;
                    case 2: axis[i] = 'x'; coord[i] = d;            break;
                    case 3: axis[i] = 'y'; coord[i] = FIELD_IN - d; break;
                } break;
        }
    }

    // Average the still-valid per-axis estimates. If an axis has NO valid
    // wall, fall back to field center rather than inventing a number (the
    // caller should place the robot near a corner so this never triggers).
    auto solve = [&](char want) {
        double sum = 0.0; int n = 0;
        for (int i = 0; i < 4; ++i)
            if (valid[i] && axis[i] == want) { sum += coord[i]; ++n; }
        return (n > 0) ? (sum / n) : (FIELD_IN / 2.0);
    };

    // Pass 1: provisional pose from every valid reading.
    double x = solve('x');
    double y = solve('y');

    // Pass 2: from that provisional pose, cast each sensor's ray against the
    // walls + static obstacle map. If a beam actually hits a mapped obstacle
    // (long-goal leg, center goal, matchload) it is NOT a wall distance, so
    // drop it before the final solve. Near a corner the provisional pose is
    // accurate enough to tell which beams are looking at a goal.
    const double th = theta_deg * field::PI / 180.0;
    for (int i = 0; i < 4; ++i) {
        if (!valid[i]) continue;
        double sx, sy, sang;
        field::sensor_world(x, y, th, *mounts[i], sx, sy, sang);
        bool hitObs = false;
        field::raycast_map(sx, sy, sang, &hitObs);
        if (hitObs) valid[i] = false;
    }

    // Final pose from the surviving wall readings.
    x = solve('x');
    y = solve('y');

    // Clamp into the field so a single bad frame can't return a wild pose.
    if (x < 0.0)        x = 0.0;
    if (x > FIELD_IN)   x = FIELD_IN;
    if (y < 0.0)        y = 0.0;
    if (y > FIELD_IN)   y = FIELD_IN;

    return lemlib::Pose(x, y, theta_deg);
}
