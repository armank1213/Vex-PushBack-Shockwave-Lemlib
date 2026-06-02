#pragma once

#include <cmath>

// ─────────────────────────────────────────────────────────────────────
// Field and sensor geometry. The single place to tune localization.
//
// Frame: field-absolute inches, (0,0) = red-side bottom-left corner.
// +X = right, +Y = away from red driver. Heading theta = 0 faces +Y,
// CW positive (LemLib convention); forward unit vector = (sin th, cos th).
//
// Robot body frame: bx = +right, by = +front. At heading 0 the body axes
// align with the world axes.
// ─────────────────────────────────────────────────────────────────────

namespace field {

constexpr double PI        = 3.14159265358979323846;
constexpr double MM_PER_IN = 25.4;

// Field interior, wall-inside-face to wall-inside-face (sensors see the
// ~140.43" interior, not the 144" outer). Canonical size; everything aliases it.
inline constexpr double FIELD_IN = 140.43;

// A perimeter distance sensor's mounting, measured on the real robot.
//   bx, by : sensor face position from the tracking center, inches
//            (bx = +right, by = +front).
//   dir    : pointing direction relative to heading, radians
//            (front = 0, right = +PI/2, back = PI, left = -PI/2).
// Modeling the true 2-D face position keeps the prediction exact at any
// heading, not just when the robot is square to a wall.
struct SensorMount {
    double bx;   // inches, +right of center
    double by;   // inches, +front of center
    double dir;  // radians, relative to heading
};

// Measured per sensor: bx/by are tape-measure readings from the tracking
// center to each sensor lens; dir is the cardinal the sensor faces.
inline constexpr SensorMount FRONT = { 0.00,  7.5, 0.0      };
inline constexpr SensorMount BACK  = { 0.00, -7.75, PI       };
inline constexpr SensorMount LEFT  = {-5.00,  -6.00, -PI / 2.0 };
inline constexpr SensorMount RIGHT = { 5.00,  -6.00,  PI / 2.0 };

// World pose of a sensor face and the world angle it points, given the
// robot pose (X, Y, th[rad]).
inline void sensor_world(double X, double Y, double th,
                         const SensorMount& m,
                         double& sx, double& sy, double& sang) {
    const double c = std::cos(th), s = std::sin(th);
    sx   = X + m.bx * c + m.by * s;
    sy   = Y - m.bx * s + m.by * c;
    sang = th + m.dir;
}

// Center-to-wall scalar offset along the sensor's pointing axis, used by the
// cardinal-snap start-pose solver:
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

// Angle (rad) between a sensor ray and the head-on normal of the wall it hit.
// 0 = perpendicular (ideal), large = grazing/unreliable. Head-on directions:
//   wall 0 (+X): +PI/2   wall 1 (-X): -PI/2   wall 2 (+Y): 0   wall 3 (-Y): PI
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

// ─────────────────────────────────────────────────────────────────────
// Static obstacle map (field-absolute inches).
//
// At the 4-9.25" sensor height the goals are not solid columns: a beam hits
// the long-goal legs, the center-goal support, or a matchload cylinder, and
// otherwise passes under the elevated troughs/arms (12"+). We map only that
// solid cross-section.
//
// These shapes are used for REJECTION, not positive localization. With only
// 4 beams you cannot reliably fix position off a ~2" leg, and the center is a
// keep-out zone. A beam that hits one carries no wall information, so the
// filter ignores it. Position fixes come from the 4 flat perimeter walls.
//
// Geometry: VEX field spec 276-9142 + bench measurements.
//   long-goal center  23.5" in from each side wall, y-centered at 70.2
//   long-goal legs    41.35" apart in Y; footprint 3"(X) x 9"(Y) each
//   center goal       one 22.5"x22.5" keep-out box at field center
//   matchloads        23.5" in, at y = 23.44 / 116.97; cylinder r = 2.25"
// ─────────────────────────────────────────────────────────────────────

struct Rect   { double x_min, y_min, x_max, y_max; };
struct Circle { double cx, cy, r; };

inline constexpr double LG_LX     = 23.5;            // red long-goal center x
inline constexpr double LG_RX     = FIELD_IN - 23.5; // blue long-goal center x
inline constexpr double LG_YC     = 70.2;            // long-goal center y
inline constexpr double LG_LEG_DY = 41.35 / 2.0;     // leg half-separation (y)
inline constexpr double LEG_HX    = 3.0 / 2.0;       // leg half-size (x)
inline constexpr double LEG_HY    = 9.0 / 2.0;       // leg half-size (y)
inline constexpr double CG_HALF   = 22.5 / 2.0;      // center keep-out half-size

inline constexpr Rect OBSTACLE_RECTS[] = {
    // red long-goal legs
    { LG_LX - LEG_HX, LG_YC - LG_LEG_DY - LEG_HY, LG_LX + LEG_HX, LG_YC - LG_LEG_DY + LEG_HY },
    { LG_LX - LEG_HX, LG_YC + LG_LEG_DY - LEG_HY, LG_LX + LEG_HX, LG_YC + LG_LEG_DY + LEG_HY },
    // blue long-goal legs
    { LG_RX - LEG_HX, LG_YC - LG_LEG_DY - LEG_HY, LG_RX + LEG_HX, LG_YC - LG_LEG_DY + LEG_HY },
    { LG_RX - LEG_HX, LG_YC + LG_LEG_DY - LEG_HY, LG_RX + LEG_HX, LG_YC + LG_LEG_DY + LEG_HY },
    // center goal keep-out box
    { LG_YC - CG_HALF, LG_YC - CG_HALF, LG_YC + CG_HALF, LG_YC + CG_HALF },
};

inline constexpr Circle OBSTACLE_CIRCLES[] = {
    { LG_LX, 23.44,  2.25 },
    { LG_LX, 116.97, 2.25 },
    { LG_RX, 23.44,  2.25 },
    { LG_RX, 116.97, 2.25 },
};

// Ray (origin rx,ry; unit dir dx,dy) vs axis-aligned rect (slab method). On a
// t>0 hit, writes the entry distance to t_out and returns true.
inline bool ray_rect(double rx, double ry, double dx, double dy,
                     const Rect& r, double& t_out) {
    double tmin = -1e30, tmax = 1e30;
    if (std::abs(dx) < 1e-12) {
        if (rx < r.x_min || rx > r.x_max) return false;
    } else {
        double t1 = (r.x_min - rx) / dx, t2 = (r.x_max - rx) / dx;
        const double lo = (t1 < t2) ? t1 : t2, hi = (t1 < t2) ? t2 : t1;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
    }
    if (std::abs(dy) < 1e-12) {
        if (ry < r.y_min || ry > r.y_max) return false;
    } else {
        double t1 = (r.y_min - ry) / dy, t2 = (r.y_max - ry) / dy;
        const double lo = (t1 < t2) ? t1 : t2, hi = (t1 < t2) ? t2 : t1;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
    }
    if (tmax < tmin || tmax < 0.0) return false;
    t_out = (tmin > 0.0) ? tmin : 0.0;   // 0 if the origin is inside the rect
    return true;
}

// Ray vs circle. On a t>0 hit, writes nearest positive distance to t_out.
inline bool ray_circle(double rx, double ry, double dx, double dy,
                       const Circle& c, double& t_out) {
    const double ox = rx - c.cx, oy = ry - c.cy;
    const double b  = ox * dx + oy * dy;
    const double cc = ox * ox + oy * oy - c.r * c.r;
    const double disc = b * b - cc;
    if (disc < 0.0) return false;
    const double sq = std::sqrt(disc);
    double t = -b - sq;
    if (t <= 1e-6) t = -b + sq;          // origin inside / first surface behind
    if (t <= 1e-6) return false;
    t_out = t;
    return true;
}

// Raycast against the walls and the static obstacles, returning the nearest
// hit distance. *hit_obstacle is set true when a mapped obstacle is closer
// than the wall (the reading is looking at a goal/matchload and should be
// ignored). *out_wall is the wall index when a wall is nearest, else -1.
inline double raycast_map(double rx, double ry, double angle,
                          bool* hit_obstacle = nullptr, int* out_wall = nullptr) {
    int wall = -1;
    double best = raycast(rx, ry, angle, &wall);
    bool obstacle = false;
    const double dx = std::sin(angle), dy = std::cos(angle);
    for (const Rect& r : OBSTACLE_RECTS) {
        double t;
        if (ray_rect(rx, ry, dx, dy, r, t) && t > 1e-6 && t < best) { best = t; obstacle = true; }
    }
    for (const Circle& c : OBSTACLE_CIRCLES) {
        double t;
        if (ray_circle(rx, ry, dx, dy, c, t) && t > 1e-6 && t < best) { best = t; obstacle = true; }
    }
    if (hit_obstacle) *hit_obstacle = obstacle;
    if (out_wall)     *out_wall = obstacle ? -1 : wall;
    return best;
}

} // namespace field
