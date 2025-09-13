#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({18, -19, 20},
                            pros::MotorGearset::green); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-11, 12, 13}, pros::MotorGearset::green); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(1);

// Motors
pros::Motor intakeMotor(17, pros::v5::MotorGears::green); // intake motor on port 17
pros::Motor outtakeMotor(6, pros::v5::MotorGears::green); // outtake motor on port 6
pros::Motor sortMotor(9, pros::v5::MotorGears::green); // sorting motor on port 9

// intake and outtake motor group
pros::MotorGroup int_outGroup({17, 6}); 

// Vision & Signatures
// vision sensor signature IDs
pros::Vision visionSensor(8);
pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(2, -2867, -2599, -2733, 8151, 8837, 8494, 4.3, 0);
pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(3, 6769, 9551, 8160, -1367, -449, -908, 2.2, 0);

// Pneumatics
pros::adi::Pneumatics matchLoad('H', false);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              352.94, // drivetrain rpm is 352.94
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
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
                            &horizontal, // horizontal tracking wheel
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
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
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
	/*
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
	*/
	/*
	chassis.moveToPoint(10, 10, 1000, {.forwards = false, .maxSpeed = 127}, true);

	chassis.moveToPose(10, 10, 90, 1000); // move to (10, 10) facing 90 degrees

	chassis.turnToHeading(90, 1000); // turn to face 90 degrees

	chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 1000); // swing left to face 90 degrees

	chassis.follow(example_txt, 10, 1000);
	*/

	// chassis.setPose(0, 0, 0);

	// chassis.turnToHeading(90, 1000);
}


void opcontrol() {
    // controller
    // loop to continuously update motors
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
        chassis.arcade(-rightX, -leftY);
        // delay to save resources
        pros::delay(10);


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
	intakeMotor.move(in_out_power);
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

    //pros::vision_object_s_t redBlock = visionSensor.get_by_sig(0,RED_SIG_ID);
   // pros::vision_object_s_t blueBlock = visionSensor.get_by_sig(0, BLUE_SIG_ID);
   pros::vision_object_s_t block = visionSensor.get_by_size(0);

    if (block.signature == RED_SIG.id && block.width > 100) {
        sort(127);
        pros::delay(500);
    }
    else if (block.signature == BLUE_SIG.id && block.width > 100) {
        sort(-127);
        pros::delay(500);
    }
    else {
        sort(0);
    }
}
