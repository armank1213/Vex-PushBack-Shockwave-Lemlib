#include "robot/localization.hpp"
#include "robot/mcl_rerun.hpp"
#include "robot/oekf_rerun.hpp"
#include "robot/chassis_config.hpp"
#include "robot/hardware.hpp"

#include "lemlib/api.hpp"
#include "pros/rtos.hpp"

#include <cmath>

namespace {

constexpr double PI = 3.14159265358979323846;

// Sensor mount geometry lives in robot/field_model.hpp.

constexpr int    CONF_GATE = 40;
constexpr double CONF_DIST_MM = 200.0;   // confidence only valid above this
constexpr double DIST_MIN_MM  = 20.0;
constexpr double DIST_MAX_MM  = 2000.0;  // 9999 => no object
constexpr double R_SENSOR  = 3.0;
constexpr double ALPHAS[4] = {0.05, 0.02, 0.02, 0.05};
constexpr double NEFF_THRESH = (double)mcl::N / 2.0;

// IMU is the heading authority; filters only solve x,y. A little jitter
// around the IMU keeps particle diversity.
constexpr double HEADING_JITTER_RAD = 1.0 * PI / 180.0;

// Distance validity gate (EKF path; MCL applies the same rule internally).
bool dist_ok(int raw_mm, int confidence) {
    if (raw_mm < DIST_MIN_MM || raw_mm > DIST_MAX_MM) return false;
    if (raw_mm <= CONF_DIST_MM)                       return true;
    return confidence >= CONF_GATE;
}

// EKF process noise per tick.
constexpr double Q_DIAG[3][3] = {{0.5, 0.0, 0.0}, {0.0, 0.3, 0.0}, {0.0, 0.0, 0.005}};

constexpr double VAR_FULL_TRUST = 4.0;   // in^2
constexpr double CORR_GAIN      = 0.35;

// Shared state for the single background task (LemLib pose access is
// mutex-guarded internally).
mcl::Filter  filter;
oekf::State  ekf;
loc::Method  method = loc::Method::MCL;
bool         correctOn = true;
loc::Estimate latest;

pros::Task*  task    = nullptr;
volatile bool running = false;

// Discrete-correction reseed request. loc::snapPose() fills these and sets
// the flag; the loop applies it at the top of the next tick, in-task, so the
// snap isn't read as a motion delta by the following predict().
volatile bool reseedReq = false;
double        reseedX = 0.0, reseedY = 0.0, reseedDeg = 0.0;

double angDiffDeg(double a, double b) {
    return std::fmod(std::fmod(b - a, 360.0) + 540.0, 360.0) - 180.0;
}

void applyCorrection(const lemlib::Pose& cur, double ex, double ey, double eth_deg, double var) {
    latest.corrected = false;
    if (!correctOn) return;
    if (var >= VAR_FULL_TRUST) return;

    double trust = (1.0 - var / VAR_FULL_TRUST) * CORR_GAIN;
    if (trust < 0.0) trust = 0.0;
    if (trust > 1.0) trust = 1.0;

    const double nx  = cur.x + (ex - cur.x) * trust;
    const double ny  = cur.y + (ey - cur.y) * trust;
    // Heading stays on the IMU: nudging theta toward the filter would inject
    // angle error and make turnToHeading() land short/long.
    chassis.setPose(nx, ny, cur.theta);
    latest.corrected = true;
}

void loop(void*) {
    lemlib::Pose prev = chassis.getPose();
    while (running) {
        // Discrete snap from loc::snapPose(): hard-set x/y, re-seed the
        // filter, resync prev so the jump isn't fed to predict() as motion.
        // Heading is never snapped — keep the IMU angle so a following
        // turnToHeading() targets the trusted heading, not the filter's.
        if (reseedReq) {
            const double curDeg = chassis.getPose().theta; // IMU heading
            const double thr     = curDeg * PI / 180.0;
            chassis.setPose(reseedX, reseedY, curDeg);
            if (method == loc::Method::MCL) {
                mcl::init(filter, reseedX, reseedY, thr, 2.0, 5.0 * PI / 180.0);
            } else {
                ekf.x = reseedX; ekf.y = reseedY; ekf.theta = thr;
            }
            prev = chassis.getPose();
            reseedReq = false;
            pros::delay(20);
            continue;
        }

        lemlib::Pose cur = chassis.getPose();
        const double dx = cur.x - prev.x;
        const double dy = cur.y - prev.y;
        double dth_deg = cur.theta - prev.theta;
        while (dth_deg >  180.0) dth_deg -= 360.0;
        while (dth_deg < -180.0) dth_deg += 360.0;
        const double dth = dth_deg * PI / 180.0;
        prev = cur;

        // Read each sensor once per tick (get() and get_confidence() are
        // separate I2C calls; reading twice risks mismatched frames).
        const int f_mm = fdist_sens.get(),  f_cf = fdist_sens.get_confidence();
        const int b_mm = bdist_sens.get(),  b_cf = bdist_sens.get_confidence();
        const int l_mm = ldist_sens.get(),  l_cf = ldist_sens.get_confidence();
        const int r_mm = rdist_sens.get(),  r_cf = rdist_sens.get_confidence();

        const double imu_rad = cur.theta * PI / 180.0;

        if (method == loc::Method::MCL) {
            // MCL wants body-frame deltas.
            const double th = cur.theta * PI / 180.0;
            const double df = dx * std::sin(th) + dy * std::cos(th);
            const double dl = dx * std::cos(th) - dy * std::sin(th);
            mcl::predict(filter, df, dl, dth, ALPHAS);

            // Pin particle heading to the IMU before the wall updates so
            // ray/wall association uses the trusted heading.
            mcl::set_heading(filter, imu_rad, HEADING_JITTER_RAD);

            // update() applies the gate, obstacle reject, and 2-D sensor
            // geometry internally; just hand it the mount + reading.
            mcl::update(filter, field::FRONT, f_mm, f_cf, R_SENSOR);
            mcl::update(filter, field::BACK,  b_mm, b_cf, R_SENSOR);
            mcl::update(filter, field::LEFT,  l_mm, l_cf, R_SENSOR);
            mcl::update(filter, field::RIGHT, r_mm, r_cf, R_SENSOR);

            mcl::summarize(filter);
            if (filter.n_eff < NEFF_THRESH) { mcl::resample(filter); mcl::summarize(filter); }

            latest.x = filter.x_mean;
            latest.y = filter.y_mean;
            // Report the IMU heading, not filter.theta_mean (which drifts a
            // couple degrees after position-weighted resampling).
            latest.theta_deg = cur.theta;
            latest.var_xy = filter.var_xy;
            latest.extra  = filter.n_eff;
            latest.method = loc::Method::MCL;
            applyCorrection(cur, latest.x, latest.y, latest.theta_deg, latest.var_xy);

        } else {
            // EKF wants world-frame deltas.
            oekf::predict(ekf, dx, dy, dth, Q_DIAG);
            ekf.theta = imu_rad;   // IMU owns heading; walls refine x,y
            if (dist_ok(f_mm, f_cf)) oekf::update(ekf, {field::FRONT, (double)f_mm, R_SENSOR});
            if (dist_ok(b_mm, b_cf)) oekf::update(ekf, {field::BACK,  (double)b_mm, R_SENSOR});
            if (dist_ok(l_mm, l_cf)) oekf::update(ekf, {field::LEFT,  (double)l_mm, R_SENSOR});
            if (dist_ok(r_mm, r_cf)) oekf::update(ekf, {field::RIGHT, (double)r_mm, R_SENSOR});

            latest.x = ekf.x;
            latest.y = ekf.y;
            latest.theta_deg = ekf.theta * 180.0 / PI;
            latest.var_xy = ekf.P[0][0] + ekf.P[1][1];
            latest.extra  = 0;
            latest.method = loc::Method::EKF;
            applyCorrection(cur, latest.x, latest.y, latest.theta_deg, latest.var_xy);
        }

        // Resync prev after a correction so the next delta is odom-only.
        if (latest.corrected) prev = chassis.getPose();

        pros::delay(20);
    }
}

} // anonymous namespace


namespace loc {

void start(double fx, double fy, double fdeg, Method m, bool correct) {
    if (running) stop();
    method = m;
    correctOn = correct;
    chassis.setPose(fx, fy, fdeg);

    if (m == Method::MCL) {
        mcl::init(filter, fx, fy, fdeg * PI / 180.0, 2.0, 5.0 * PI / 180.0);
    } else {
        ekf = oekf::State{};
        ekf.x = fx; ekf.y = fy; ekf.theta = fdeg * PI / 180.0;
    }

    latest = Estimate{};
    latest.x = fx; latest.y = fy; latest.theta_deg = fdeg; latest.method = m;

    running = true;
    task = new pros::Task(loop, nullptr, "loc_task");
}

void startHere(Method m, bool correct) {
    lemlib::Pose p = chassis.getPose();
    start(p.x, p.y, p.theta, m, correct);
}

void stop() {
    running = false;
    pros::delay(40);
    if (task) { delete task; task = nullptr; }
}

bool active() { return running; }

Estimate estimate() { return latest; }

bool snapPose(int settle_ms, double max_var) {
    if (!running) return false;

    // Let the filter converge on the stationary readings. The robot must
    // actually be stopped when this is called.
    if (settle_ms > 0) pros::delay(settle_ms);

    const Estimate e = latest;
    const bool confident =
        e.var_xy < max_var &&
        (e.method != Method::MCL || e.extra > (double)mcl::N * 0.5);
    if (!confident) return false;

    // Hand the snap to the task and wait (bounded) for it to apply.
    reseedX   = e.x;  // consumed in-task at the next tick

    reseedY   = e.y;
    reseedDeg = e.theta_deg;
    reseedReq = true;
    for (int waited = 0; reseedReq && waited < 200; waited += 5) pros::delay(5);
    return !reseedReq;
}

} // namespace loc
