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

// Sensor mount geometry lives in robot/field_model.hpp (field::FRONT, ...).

constexpr int    CONF_GATE = 40;
constexpr double CONF_DIST_MM = 200.0;   // VEX: confidence only valid above this
constexpr double DIST_MIN_MM  = 20.0;
constexpr double DIST_MAX_MM  = 2000.0;  // get()==9999 => no object
constexpr double R_SENSOR  = 3.0;
constexpr double ALPHAS[4] = {0.05, 0.02, 0.02, 0.05};
constexpr double NEFF_THRESH = (double)mcl::N / 2.0;

// Heading slack around the IMU. The IMU is the heading authority; the
// filters only solve x,y. 1 degree of jitter keeps a little diversity.
constexpr double HEADING_JITTER_RAD = 1.0 * PI / 180.0;

// Spec-correct distance validity (used for the EKF path, which gates here;
// the MCL update() applies the same rule internally).
bool dist_ok(int raw_mm, int confidence) {
    if (raw_mm < DIST_MIN_MM || raw_mm > DIST_MAX_MM) return false;
    if (raw_mm <= CONF_DIST_MM)                       return true;
    return confidence >= CONF_GATE;
}

// EKF process noise per tick.
constexpr double Q_DIAG[3][3] = {{0.5, 0.0, 0.0}, {0.0, 0.3, 0.0}, {0.0, 0.0, 0.005}};

constexpr double VAR_FULL_TRUST = 4.0;   // in^2
constexpr double CORR_GAIN      = 0.35;

// Shared state (single background task — plain globals, races acceptable
// for a mock; LemLib pose access is mutex-guarded internally).
mcl::Filter  filter;
oekf::State  ekf;
loc::Method  method = loc::Method::MCL;
bool         correctOn = true;
loc::Estimate latest;

pros::Task*  task    = nullptr;
volatile bool running = false;

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
    const double nth = cur.theta + angDiffDeg(cur.theta, eth_deg) * trust;
    chassis.setPose(nx, ny, nth);
    latest.corrected = true;
}

void loop(void*) {
    lemlib::Pose prev = chassis.getPose();
    while (running) {
        lemlib::Pose cur = chassis.getPose();
        const double dx = cur.x - prev.x;
        const double dy = cur.y - prev.y;
        double dth_deg = cur.theta - prev.theta;
        while (dth_deg >  180.0) dth_deg -= 360.0;
        while (dth_deg < -180.0) dth_deg += 360.0;
        const double dth = dth_deg * PI / 180.0;
        prev = cur;

        // Read every sensor ONCE per tick (get() and get_confidence() are
        // separate I2C round-trips; calling them twice can return mismatched
        // frames). raw is mm; confidence is 0..63.
        const int f_mm = fdist_sens.get(),  f_cf = fdist_sens.get_confidence();
        const int b_mm = bdist_sens.get(),  b_cf = bdist_sens.get_confidence();
        const int l_mm = ldist_sens.get(),  l_cf = ldist_sens.get_confidence();
        const int r_mm = rdist_sens.get(),  r_cf = rdist_sens.get_confidence();

        // The IMU is the heading authority; the filters only solve x,y.
        const double imu_rad = cur.theta * PI / 180.0;

        if (method == loc::Method::MCL) {
            // MCL wants body-frame deltas.
            const double th = cur.theta * PI / 180.0;
            const double df = dx * std::sin(th) + dy * std::cos(th);
            const double dl = dx * std::cos(th) - dy * std::sin(th);
            mcl::predict(filter, df, dl, dth, ALPHAS);

            // Pin particle orientation to the IMU before the wall updates so
            // ray/wall association uses the trusted heading, not drift.
            mcl::set_heading(filter, imu_rad, HEADING_JITTER_RAD);

            // update() applies the spec-correct gate, the obstacle reject,
            // and the true 2-D sensor geometry internally (field_model.hpp),
            // so we just hand it the mount + reading.
            mcl::update(filter, field::FRONT, f_mm, f_cf, R_SENSOR);
            mcl::update(filter, field::BACK,  b_mm, b_cf, R_SENSOR);
            mcl::update(filter, field::LEFT,  l_mm, l_cf, R_SENSOR);
            mcl::update(filter, field::RIGHT, r_mm, r_cf, R_SENSOR);

            mcl::summarize(filter);
            if (filter.n_eff < NEFF_THRESH) { mcl::resample(filter); mcl::summarize(filter); }

            latest.x = filter.x_mean;
            latest.y = filter.y_mean;
            latest.theta_deg = filter.theta_mean * 180.0 / PI;
            latest.var_xy = filter.var_xy;
            latest.extra  = filter.n_eff;
            latest.method = loc::Method::MCL;
            applyCorrection(cur, latest.x, latest.y, latest.theta_deg, latest.var_xy);

        } else {
            // EKF wants WORLD-frame deltas.
            oekf::predict(ekf, dx, dy, dth, Q_DIAG);
            // Trust the IMU for heading; the walls only refine x,y.
            ekf.theta = imu_rad;
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

        // Resync prev if we corrected, so next delta is odom-only.
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

} // namespace loc
