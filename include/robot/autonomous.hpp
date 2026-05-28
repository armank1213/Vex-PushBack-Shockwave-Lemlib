#pragma once

// PID tuning
void angular_tuning();
void lateral_tuning();

// Autonomous routines
void oekf_rerun();
void mcl_rerun();
void corrected_auton();   // hand-coded auton + background MCL correction
void localization_test(); // live readout: odom vs filter estimate vs variance
void left_auton();
void right_auton();
void skills_auton();
void park_auton();

