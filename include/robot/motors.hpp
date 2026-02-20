#pragma once 

// Basic motor control functions
void intake(int intakePower);
void outtake(int outtakePower);
void middleTake(int middlePower);
void sort(int sortPower);
void fullMotorControl();
void halfMotorControl();

// Control functions
void intakeControl();
void outtakeControl();
void manual_colorSort();