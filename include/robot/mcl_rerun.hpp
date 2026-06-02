#pragma once

#include "robot/field_model.hpp"

// Monte Carlo Localization (particle filter) and the SD-route replay path.
//
// Frame: field-absolute inches, (0,0) = red-side bottom-left of the 144"
// field. +X right, +Y away from red driver. Heading theta=0 = +Y, CW
// positive (LemLib), stored internally in radians.

namespace mcl {

inline constexpr int N = 500;

struct Particle {
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;   // radians
    double w     = 1.0 / N;
};

struct Filter {
    Particle particles[N];

    // Latest summary (recomputed by mean()).
    double x_mean       = 0.0;
    double y_mean       = 0.0;
    double theta_mean   = 0.0;  // radians
    double var_xy       = 0.0;  // trace of x,y covariance, in^2
    double n_eff        = (double)N;
};

// Seed RNG from pros::millis() and scatter cloud around (x0, y0, theta0)
// with Gaussian spread.
void init(Filter& f,
          double x0, double y0, double theta0_rad,
          double sigma_xy_in, double sigma_theta_rad);

// Motion model in robot body frame.
//   d_forward: vertical odom pod delta (in)
//   d_lateral: horizontal odom pod delta (in)
//   dtheta_rad: IMU heading delta (rad)
//   alphas[4]: Thrun motion-model noise coefficients (table 5.5).
//     alphas[0..1] = rotation noise, alphas[2..3] = translation noise.
//     Typical mock values: {0.05, 0.02, 0.02, 0.05}.
void predict(Filter& f,
             double d_forward, double d_lateral, double dtheta_rad,
             const double alphas[4]);

// Pin every particle's heading to a trusted measured heading (the IMU),
// with sigma_rad of jitter. Call once per tick so the distance-sensor
// wall association uses the gyro heading instead of drifting particle
// headings. theta_rad uses the LemLib convention (0 = +Y, CW positive).
void set_heading(Filter& f, double theta_rad, double sigma_rad);

// Fuse one wall-distance reading.
//   mount: sensor body-frame face position + pointing dir (field::FRONT/...).
//          The ray is cast from the sensor's true world position.
//   raw_mm: VEX Distance reading (sensor face -> wall, mm).
//   confidence: VEX get_confidence(), 0..63.
//   R_noise: measurement variance (in^2).
// Returns false if gated out (range, low confidence, grazing, or obstacle).
bool update(Filter& f,
            const field::SensorMount& mount,
            int raw_mm,
            int confidence,
            double R_noise);

// Low-variance resampler (Thrun, p.110). Caller gates on n_eff < N/2.
void resample(Filter& f);

// Recompute mean, n_eff, var_xy from current weights.
void summarize(Filter& f);

// Top-level entry: reads /usd/dtData.txt and replays it with MCL correction,
// starting from the given field-absolute pose.
void rerun(double start_x_in, double start_y_in, double start_theta_deg);

} // namespace mcl

// Flat entry for autonomous(); auto-detects start pose (theta=0 assumption).
void mcl_rerun();
