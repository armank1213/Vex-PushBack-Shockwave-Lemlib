#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep


// Autonomous helper functions

void correct_y(int target_y, int final_x, int final_t, int timeout, bool reversed, bool async);
void correct_x(int target_x, int final_y, int final_t, int timeout, bool reversed, bool async);
void correct_t(int target_t, int final_x, int final_y, int timeout, bool reversed, bool async);

// distance reset

double angle_error(double angle_a, double angle_b);
std::pair<double,double> reset_distance();

void dReset(int ftarget, int btarget, int ltarget, int rtarget);