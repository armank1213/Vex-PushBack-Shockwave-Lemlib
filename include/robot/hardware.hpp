#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"

// Controller
extern pros::Controller controller;

// Motor Groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Individual Motors
extern pros::Motor outtakeMotor;
extern pros::Motor intakeMotor;
extern pros::Motor middleMotor;

// Sensors
extern pros::Imu imu;
extern pros::Optical colorSensor;
extern pros::Distance distanceSensor;
extern pros::Optical limiter_light;

extern pros::Distance ldist_sens;
extern pros::Distance rdist_sens;
extern pros::Distance fdist_sens;
extern pros::Distance bdist_sens;


// Pneumatics
extern pros::adi::Pneumatics matchLoad;
extern pros::adi::Pneumatics limiter;
extern pros::adi::Pneumatics wing;

// Tracking Wheels
extern pros::Rotation horizontal_rotation;
extern pros::Rotation vertical_rotation;

