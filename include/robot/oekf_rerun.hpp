#pragma once

#include <fstream>

namespace oekf {

// VEX Pushback field is 144" x 144".
inline constexpr double FIELD_IN = 144.0;

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

// One distance reading worth fusing.
struct WallObs {
    double sensor_world_angle;   // rad, world frame, direction sensor faces
    double sensor_offset_in;     // in, robot center -> sensor face along that direction
    double raw_mm;               // VEX Distance reading
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
