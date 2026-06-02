#pragma once

#include "lemlib/pose.hpp"

// Field-absolute frame: (0,0) = red-side bottom-left of a 144"x144" field.
// +X right, +Y away from red driver. LemLib heading (theta=0 = +Y, CW+).

// Infer a field-absolute start pose from the 4 perimeter distance sensors,
// given the heading the robot is physically set to. Place the robot close
// enough to the walls that at least one sensor per axis sees a wall.
//
// theta_deg: heading the robot is physically pointing (e.g. 0 = facing +Y).
//
// Returns (x, y, theta_deg) in inches, clamped to [0, FIELD_IN]. Opposite
// walls on an axis are averaged when both are valid; an axis with no valid
// reading falls back to field-center (FIELD_IN / 2).
lemlib::Pose determine_start_pose(double theta_deg = 0.0);
