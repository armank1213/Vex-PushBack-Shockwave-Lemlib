#include "robot/hardware.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


pros::MotorGroup leftMotors({-19, -13, -17}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({12, 18, 14}, pros::MotorGearset::blue);

// Individual Motors
pros::Motor outtakeMotor(10, pros::v5::MotorGears::green); 
pros::Motor intakeMotor(15, pros::v5::MotorGears::green); 
pros::Motor middleMotor(7, pros::v5::MotorGears::blue); 

// Sensors
pros::Optical colorSensor(8); 
pros::Distance distanceSensor(2); // Also used for color sorting
pros::Optical limiter_light(6); // Used for limiter state indication
pros::Imu imu(3);
pros::Rotation vertical_rotation(-11);
pros::Rotation horizontal_rotation(20);

//Trilateral Sensors
pros::Distance ldist_sens(9); 
pros::Distance rdist_sens(4); 
pros::Distance fdist_sens(5);
pros::Distance bdist_sens(6);

// Pneumatics
pros::adi::Pneumatics matchLoad('E', false);
pros::adi::Pneumatics limiter('A', false);
pros::adi::Pneumatics wing('C', false);