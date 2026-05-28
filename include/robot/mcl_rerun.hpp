#pragma once

// Monte Carlo Localization for OEKF-Rerun replay path.
//
// Frame convention: field-absolute inches, (0,0) = red-side bottom-left
// corner of the 144" x 144" Pushback field. +X right, +Y away from red
// driver. LemLib heading convention — theta=0 = +Y, CW positive,
// internally stored in radians.

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

// Fuse one wall-distance reading.
//   sensor_world_angle_at_theta0: sensor pointing direction expressed
//     relative to robot heading. front=0, back=PI, left=-PI/2, right=+PI/2.
//   sensor_offset_in: center -> sensor face along that direction.
//   raw_mm: VEX Distance raw reading.
//   confidence: VEX get_confidence(), 0..63.
//   R_noise: measurement variance (in^2).
// Returns true if reading was fused, false if gated out.
bool update(Filter& f,
            double sensor_world_angle_at_theta0,
            double sensor_offset_in,
            int raw_mm,
            int confidence,
            double R_noise);

// Low-variance resampler (Thrun, p.110). Only call when n_eff drops
// below N/2 — caller is responsible for the threshold.
void resample(Filter& f);

// Recompute mean, n_eff, var_xy from current weights.
void summarize(Filter& f);

// Top-level entry. Reads /usd/dtData.txt, replays with MCL correction.
// start_pose_field is the user-supplied (or distance-sensor-inferred)
// field-absolute pose at the start of the run.
void rerun(double start_x_in, double start_y_in, double start_theta_deg);

} // namespace mcl

// Flat entry for autonomous() to call. Auto-detects start pose via
// determine_start_pose() with theta=0 assumption.
void mcl_rerun();
