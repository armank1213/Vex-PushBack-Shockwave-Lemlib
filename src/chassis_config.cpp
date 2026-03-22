#include "robot/chassis_config.hpp"
#include "robot/hardware.hpp"

// Tracking wheels
// 11 inch long, 13.5 inch wide
// Tracking center (6.875, 5.5)
// Horizontal tracking wheel. 2" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, -5.5);
// Vertical tracking wheel. 2" diameter, 0.37" offset, left of the robot (negative)
lemlib::TrackingWheel vertical_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, 1.5);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.25, // 11 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 for now
);

// Lateral PID controller
lemlib::ControllerSettings lateralController(10.25, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small errsor range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Angular PID controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Sensors for odometry
// Tracking center: (4,6.3125)
// Vertical Tracking Wheel Offset: 6.3125-6.3125 = 0.000
// Horizontal Tracking Wheel Offset: 4-(-1.5) = 5.5 (this value 5.5 should be negative since its at the back of the robot)
lemlib::OdomSensors sensors(&vertical_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// Input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// Input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

void initializeChassis() {
    // Calibration is done in main.cpp initialize() function
}

