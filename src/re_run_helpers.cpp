#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp" // IWYU pragma: keep

#include <cstdio> // IWYU pragma: keep
#include <iostream> // IWYU pragma: keep
#include <fstream> // IWYU pragma: keep

void voltage_recording(std::ofstream& ofs, bool open) {
    if (!open) return;

    // battery normalization during recording so playback voltages are
    // hardware-independent and match what voltage_re_run expects
    double battery = pros::battery::get_voltage() / 12000.0;
    double l1 = leftMotors.get_voltage()  / battery;
    double r1 = rightMotors.get_voltage() / battery;

    double x     = chassis.getPose().x;
    double y     = chassis.getPose().y;
    double theta = chassis.getPose().theta;

    ofs << l1 << " " << r1 << " " << x << " " << y << " " << theta << "\n";
    ofs.flush();
}


void pose_recording(std::ofstream& ofs, bool open) {
    if (!open) return;

    int x = chassis.getPose().x;
    int y = chassis.getPose().y;
    int t = chassis.getPose().theta;

    ofs << x << " " << y << " " << t << "\n";
    ofs.flush();
}


void dist_sens_angle_correction(double t, double quadrant, bool fwall, bool bwall, bool lwall, bool rwall) {

}