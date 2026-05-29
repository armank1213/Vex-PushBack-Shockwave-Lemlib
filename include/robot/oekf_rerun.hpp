#pragma once

#include "robot/field_model.hpp"
#include <fstream>

namespace oekf {

// Field interior size. Canonical value lives in field_model.hpp; aliased
// here so existing oekf::FIELD_IN references keep working.
inline constexpr double FIELD_IN = field::FIELD_IN;   // 140.43

// 3-state EKF: [x_in, y_in, theta_rad]^T.
// P is the 3x3 covariance. Diagonal seed = initial uncertainty.
struct State {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double P[3][3] = {{10.0,  0.0, 0.0},
                      { 0.0, 10.0, 0.0},
                      { 0.0,  0.0, 0.3}};
};

// One distance reading worth fusing. The sensor's body-frame mount (face
// position + pointing direction) is taken from field_model.hpp; the ray is
// cast from the sensor's true world position using the EKF's current pose.
struct WallObs {
    field::SensorMount mount;    // measured sensor geometry
    double raw_mm;               // VEX Distance reading (sensor face -> wall, mm)
    double R_noise;              // measurement variance (in^2)
};

// Predict: integrate odom delta (world-frame inches + radians) into mean,
// inflate P by process noise Q. Caller supplies the deltas this tick.
void predict(State& s,
             double dx_world,
             double dy_world,
             double dtheta_rad,
             const double Q[3][3]);

// Update: fuse one wall-distance reading. Gates on raw range + innovation,
// returns false if reading was rejected.
bool update(State& s, const WallObs& z);

// Top-level entry. Reads /usd/dtData.txt, replays the recorded run with
// EKF correction from the 4 perimeter distance sensors.
void run();

} // namespace oekf

// Flat entry for autonomous() to call.
void oekf_rerun();
