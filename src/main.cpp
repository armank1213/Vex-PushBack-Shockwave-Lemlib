#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/vision.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({18, -19, 20}, pros::MotorGearset::green); // left motor group - ports 18, 19 (reversed), 20
pros::MotorGroup rightMotors({-11, 12, 13}, pros::MotorGearset::green); // right motor group - ports 11 (reversed), 12, 13

// Inertial Sensor on port 10
pros::Imu imu(1);

// Motors
pros::Motor intakeMotor(17, pros::v5::MotorGears::green); // intake motor on port 17
pros::Motor outtakeMotor(6, pros::v5::MotorGears::green); // outtake motor on port 6
pros::Motor sortMotor(9, pros::v5::MotorGears::green); // sorting motor on port 9

// intake and outtake motor group
pros::MotorGroup in_outGroup({17, 6}); 

// Vision & Signatures
// vision sensor signature IDs
pros::Vision visionSensor(8);
pros::vision_signature_s_t BLUE_SIG  = pros::Vision::signature_from_utility(1, -4089, -2329, -3210, 2711, 4961, 3836, 2.100, 0);
pros::vision_signature_s_t RED_SIG  = pros::Vision::signature_from_utility(2, 4861, 11873, 8368, -1889, -225, -1058, 1.300, 0);

// Pneumatics
pros::adi::Pneumatics matchLoad('H', false);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation vertical_rotation(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&vertical_rotation, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
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

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis sensors.

    /*
    while (true) {
        // print measurements from the rotation sensor
        pros::lcd::print(1, "Rotation Sensor: %i", vertical_rotation.get_position());
        pros::delay(10); // delay to save resources. DO NOT REMOVE
    }
    */

    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during autonomous
 */
void autonomous() {

    // Angular PID Tuning
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000);

}


void opcontrol() {
    
	void manual_in_out();
	void manual_sort();
	void colorSort();

    bool pistonToggle = false;

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
		// chassis.tank(leftY, rightX);
        // chassis.curvature(-rightX, -leftY, false);
        chassis.arcade(-rightX, -leftY, false, .3);
        // delay to save resources
        pros::delay(25);


        manual_in_out();

        // manual_sort();

        colorSort();

        // Pneumatics Toggle
        if (controller.get_digital(DIGITAL_X)) {
            if (pistonToggle == false) {
                matchLoad.extend();
                pros::delay(250);
                pistonToggle = true;
            } else {
                matchLoad.retract();
                pros::delay(250);
                pistonToggle = false;
            }
        }
    }
}

// Intake/Outtake Motor Function
void in_out(int in_out_power) {
	in_outGroup.move(in_out_power);
}

// Sort Motor Function
void sort(int sortPower) {
	sortMotor.move(sortPower);
}

// Manual Intake/Outtake
void manual_in_out() {
	if (controller.get_digital(DIGITAL_R1)) {
		in_out(127);
	}
	else if (controller.get_digital(DIGITAL_R2)) {
      in_out(-127);
    }
	else {
      in_out(0);
    }
}

// Manual Sorting
void manual_sort() {
	if (controller.get_digital(DIGITAL_L1)) {
      sort(-127);
    }
	else if (controller.get_digital(DIGITAL_L2)) {
      sort(127);
    }
	else {
      sort(0);
    }
}

// Color Sorting 
void colorSort() {
   pros::vision_object_s_t block = visionSensor.get_by_size(0);

    if (block.signature == RED_SIG.id && block.width > 100 && block.height > 95) {
        sort(127);
        pros::delay(200);
    }
    else if (block.signature == BLUE_SIG.id && block.width > 100) {
        sort(-127);
        pros::delay(100);
    }
    else {
        sort(0);
    }
}