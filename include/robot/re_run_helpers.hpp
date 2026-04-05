#include <cstdio> // IWYU pragma: keep
#include <iostream> // IWYU pragma: keep
#include <fstream> // IWYU pragma: keep


#pragma once

void voltage_recording(std::ofstream& ofs, bool open);

void pose_recording(std::ofstream& ofs, bool open);

void dist_sens_angle_correction(double t, double quadrant, bool fwall, bool bwall, bool lwall, bool rwall);