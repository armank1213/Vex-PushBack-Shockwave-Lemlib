#pragma once

#include <cmath>

// ─────────────────────────────────────────────────────────────────────
// FIELD + SENSOR GEOMETRY — the ONE place to tune localization.
//
// Frame: field-absolute inches, (0,0) = red-side bottom-left corner.
// +X = right, +Y = away from red driver. Heading theta = 0 faces +Y,
// CW positive (LemLib convention); forward unit vector = (sin th, cos th).
//
// Robot BODY frame (used to place the sensors): bx = +right, by = +front.
// At heading 0 the body axes line up with the world axes, so the front
// sensor (by>0) sticks out toward +Y, the right sensor (bx>0) toward +X.
// ─────────────────────────────────────────────────────────────────────

namespace field {

constexpr double PI        = 3.14159265358979323846;
constexpr double MM_PER_IN = 25.4;

// Field interior, wall-inside-face to wall-inside-face. The field is 144"
// outer but the distance sensors see the interior walls ~140.43" apart.
// CANONICAL field size — everything else aliases this.
inline constexpr double FIELD_IN = 140.43;

// A perimeter distance sensor's mounting, MEASURED ON THE REAL ROBOT.
//   bx, by : position of the sensor's FACE (the lens it shoots from),
//            in inches, from the robot's tracking center. bx = +right,
//            by = +front. Measure with a ruler in BOTH axes.
//   dir    : direction the sensor points, RELATIVE to robot heading, in
//            radians. front = 0, right = +PI/2, back = PI, left = -PI/2.
//
// Why both axes: against a flat wall hit head-on, only the along-beam
// distance matters, so the old single offset was fine WHEN the robot was
// square to the wall. The moment the robot is rotated (or a beam grazes a
// corner / clips the long goal), the sensor's sideways position changes
// what it sees. Modeling the true 2-D face position makes the prediction
// exact at any heading — and you tune it by measuring, not guessing.
struct SensorMount {
    double bx;   // inches, +right of center
    double by;   // inches, +front of center
    double dir;  // radians, relative to heading
};

// ===== MEASURE THESE FOUR ON YOUR ROBOT =====
// Replace bx/by with a tape-measure reading from the tracking center to
// each sensor's lens. The by (front/back) and bx (left/right) magnitudes
// below are the old center-offset values as a starting point; the missing
// perpendicular component currently assumed 0 is what you should fill in.
inline constexpr SensorMount FRONT = { 0.00,  7.5, 0.0      };
inline constexpr SensorMount BACK  = { 0.00, -7.75, PI       };
inline constexpr SensorMount LEFT  = {-5.00,  -6.00, -PI / 2.0 };
inline constexpr SensorMount RIGHT = { 5.00,  -6.00,  PI / 2.0 };
// World pose of a sensor face + the world angle it points, given the
// robot pose (X, Y, th[rad]). Rotation of body (bx,by) into world:
//   wx = X + bx*cos th + by*sin th
//   wy = Y - bx*sin th + by*cos th
inline void sensor_world(double X, double Y, double th,
                         const SensorMount& m,
                         double& sx, double& sy, double& sang) {
    const double c = std::cos(th), s = std::sin(th);
    sx   = X + m.bx * c + m.by * s;
    sy   = Y - m.bx * s + m.by * c;
    sang = th + m.dir;
}

// Center-to-wall scalar offset along the sensor's pointing axis. Equals
// the projection of the face position onto the pointing direction. Used
// by the cardinal-snap start-pose solver (where the perpendicular offset
// does not affect a head-on flat-wall reading).
//   measured_center_to_wall = raw_in + along_offset(m)
inline double along_offset(const SensorMount& m) {
    return m.bx * std::sin(m.dir) + m.by * std::cos(m.dir);
}

// Distance from (rx,ry) along world heading `angle` to the nearest wall of
// the [0,FIELD_IN]^2 square. Optionally reports which wall was hit:
// 0 = +X (x=FIELD), 1 = -X (x=0), 2 = +Y (y=FIELD), 3 = -Y (y=0). Inches.
inline double raycast(double rx, double ry, double angle, int* out_wall = nullptr) {
    const double cx = std::sin(angle);
    const double cy = std::cos(angle);
    double t = 1e9;
    int wall = -1;
    if (cx >  1e-6) { double tt = (FIELD_IN - rx) / cx;    if (tt < t) { t = tt; wall = 0; } }
    if (cx < -1e-6) { double tt = (rx)            / (-cx); if (tt < t) { t = tt; wall = 1; } }
    if (cy >  1e-6) { double tt = (FIELD_IN - ry) / cy;    if (tt < t) { t = tt; wall = 2; } }
    if (cy < -1e-6) { double tt = (ry)            / (-cy); if (tt < t) { t = tt; wall = 3; } }
    if (out_wall) *out_wall = wall;
    return t;
}

// Angle (rad) between a sensor ray and the head-on normal of the wall it
// hit. 0 = perpendicular (ideal), large = grazing/unreliable. Head-on ray
// directions per wall, in (sin a, cos a) convention:
//   wall 0 (+X): a = +PI/2   wall 1 (-X): a = -PI/2
//   wall 2 (+Y): a = 0       wall 3 (-Y): a = PI
inline double grazing_angle(double sensor_angle, int wall) {
    double normal = 0.0;
    switch (wall) {
        case 0: normal =  PI / 2.0; break;
        case 1: normal = -PI / 2.0; break;
        case 2: normal =  0.0;      break;
        case 3: normal =  PI;       break;
        default: return 1e9;
    }
    double d = sensor_angle - normal;
    while (d >  PI) d -= 2.0 * PI;
    while (d < -PI) d += 2.0 * PI;
    return std::abs(d);
}

} // namespace field
