#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp" // IWYU pragma: keep

#include <cstdio> // IWYU pragma: keep
#include <iostream> // IWYU pragma: keep
#include <fstream> // IWYU pragma: keep

void voltage_recording(std::ofstream& ofs, bool open) {
    if (!open) return;

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

    double x = chassis.getPose().x;
    double y = chassis.getPose().y;
    double t = chassis.getPose().theta;

    // Record all button states (0 or 1) after the pose
    int r1    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)    ? 1 : 0;
    int r2    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)    ? 1 : 0;
    int y_btn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)     ? 1 : 0;
    int b_btn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)     ? 1 : 0;
    int l2    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)    ? 1 : 0;
    int l1    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)    ? 1 : 0;
    int right = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ? 1 : 0;
    int x_btn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)     ? 1 : 0;

    // Format: x y theta r1 r2 y b l2 l1 right x
    ofs << x     << " " << y     << " " << t     << " "
        << r1    << " " << r2    << " " << y_btn << " "
        << b_btn << " " << l2    << " " << l1    << " "
        << right << " " << x_btn << "\n";
    ofs.flush();
}

void dist_sens_angle_correction(double t, double quadrant, bool fwall, bool bwall, bool lwall, bool rwall) {

}