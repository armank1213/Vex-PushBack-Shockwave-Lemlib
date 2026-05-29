#pragma once

#include "lemlib/pose.hpp"

// Field-absolute frame: (0,0) = red-side bottom-left corner of a
// 144"x144" Pushback field. +X right, +Y away from red driver. LemLib
// heading convention (theta=0 = +Y, CW positive).

// Read the 4 perimeter distance sensors and infer a field-absolute
// starting pose, given the heading the robot is *physically* set to.
//
// Assumes the robot has been placed close enough to the field walls
// that at least one sensor on each axis sees a wall with usable
// confidence. Averages opposite-axis sensors when both are valid.
//
// theta_deg: heading the robot is physically pointing (driver knows
//            this — e.g. 0 = facing +Y / blue side, 90 = facing +X).
//
// Returns the inferred field-absolute pose (x, y, theta_deg). x and y
// in inches, clamped into [0, FIELD_IN]. When BOTH walls on an axis are
// in range it averages them; otherwise it uses whichever wall is valid.
// If a sensor pair has no valid reading on an axis, that axis falls back
// to field-center (FIELD_IN / 2).
lemlib::Pose determine_start_pose(double theta_deg = 0.0);
