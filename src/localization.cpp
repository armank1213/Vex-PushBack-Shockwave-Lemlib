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

constexpr double FRONT_OFFSET = 7.75;
constexpr double BACK_OFFSET  = 9.00;
constexpr double LEFT_OFFSET  = 7.50;
constexpr double RIGHT_OFFSET = 7.50;

constexpr int    CONF_GATE = 40;
constexpr double R_SENSOR  = 3.0;
constexpr double ALPHAS[4] = {0.05, 0.02, 0.02, 0.05};
constexpr double NEFF_THRESH = (double)mcl::N / 2.0;

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
        const double dth = (cur.theta - prev.theta) * PI / 180.0;
        prev = cur;

        if (method == loc::Method::MCL) {
            // MCL wants body-frame deltas.
            const double th = cur.theta * PI / 180.0;
            const double df = dx * std::sin(th) + dy * std::cos(th);
            const double dl = dx * std::cos(th) - dy * std::sin(th);
            mcl::predict(filter, df, dl, dth, ALPHAS);

            if (fdist_sens.get_confidence() > CONF_GATE)
                mcl::update(filter, 0.0,       FRONT_OFFSET, fdist_sens.get(), fdist_sens.get_confidence(), R_SENSOR);
            if (bdist_sens.get_confidence() > CONF_GATE)
                mcl::update(filter, PI,        BACK_OFFSET,  bdist_sens.get(), bdist_sens.get_confidence(), R_SENSOR);
            if (ldist_sens.get_confidence() > CONF_GATE)
                mcl::update(filter, -PI / 2.0, LEFT_OFFSET,  ldist_sens.get(), ldist_sens.get_confidence(), R_SENSOR);
            if (rdist_sens.get_confidence() > CONF_GATE)
                mcl::update(filter, +PI / 2.0, RIGHT_OFFSET, rdist_sens.get(), rdist_sens.get_confidence(), R_SENSOR);

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
            if (fdist_sens.get_confidence() > CONF_GATE)
                oekf::update(ekf, {ekf.theta,            FRONT_OFFSET, (double)fdist_sens.get(), R_SENSOR});
            if (bdist_sens.get_confidence() > CONF_GATE)
                oekf::update(ekf, {ekf.theta + PI,       BACK_OFFSET,  (double)bdist_sens.get(), R_SENSOR});
            if (ldist_sens.get_confidence() > CONF_GATE)
                oekf::update(ekf, {ekf.theta - PI / 2.0, LEFT_OFFSET,  (double)ldist_sens.get(), R_SENSOR});
            if (rdist_sens.get_confidence() > CONF_GATE)
                oekf::update(ekf, {ekf.theta + PI / 2.0, RIGHT_OFFSET, (double)rdist_sens.get(), R_SENSOR});

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
    if (running) return;
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
