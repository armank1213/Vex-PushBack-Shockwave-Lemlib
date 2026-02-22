#include "robot/hardware.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


pros::MotorGroup leftMotors({-19, -13, -17}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({12, 18, 14}, pros::MotorGearset::blue);

// Individual Motors
pros::Motor outtakeMotor(10, pros::v5::MotorGears::green); // outtake motor on port 15
pros::Motor intakeMotor(15, pros::v5::MotorGears::green); // intake motor on port 2
pros::Motor middleMotor(7, pros::v5::MotorGears::blue); // middletake motor on port 9

// Sensors
pros::Optical colorSensor(8); // Optical Sensor on port 19
pros::Distance distanceSensor(2); // Distance Sensor on port 14
pros::Optical limiter_light(6); // Synesthesia sensor on port 6
pros::Imu imu(3);
pros::Rotation vertical_rotation(-11);
pros::Rotation horizontal_rotation(20);
pros::Distance LeftDistanceSensor(21);
pros::Distance RightDistanceSensor(21); 
pros::Distance BackDistanceSensor(21);

// Pneumatics
pros::adi::Pneumatics matchLoad('E', false);
pros::adi::Pneumatics limiter('A', false);
pros::adi::Pneumatics wing('C', false);