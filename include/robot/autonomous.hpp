#pragma once

// PID tuning
void angular_tuning();
void lateral_tuning();

// Autonomous routines
void voltage_re_run();
void odom_ekf_run();
void left_auton();
void right_auton();
void skills_auton();

