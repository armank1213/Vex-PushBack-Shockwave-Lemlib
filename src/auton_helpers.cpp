#include "robot/autonomous.hpp" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include <cmath>


void correct_y(int Ytarget, int Xfinal, int Tfinal, int timeout, bool reversed, bool async) {
    long startTime = pros::millis();
    long elapsedTime = 0;

    int Ydelta = chassis.getPose().y;
    while (elapsedTime < 4000) {
        if (reversed) {
            chassis.moveToPose(Xfinal, Ytarget-Ydelta, Tfinal, timeout, {.forwards=true}, async);
        }
        else {
            chassis.moveToPose(Xfinal, Ytarget-Ydelta, Tfinal, timeout, {}, async);
        }
        elapsedTime = pros::millis() - startTime;
    }   
}
void correct_x(int Xtarget, int Yfinal, int Tfinal, int timeout, bool reversed, bool async) {
    long startTime = pros::millis();
    long elapsedTime = 0;
    
    int Xdelta = chassis.getPose().x;
    while (elapsedTime < 4000) {
        if (reversed) {
            chassis.moveToPose(Xtarget-Xdelta, Yfinal, Tfinal, timeout, {.forwards=true}, async);
        }
        else {
            chassis.moveToPose(Xtarget-Xdelta, Yfinal, Tfinal, timeout, {}, async);
        }
        elapsedTime = pros::millis() - startTime;
    }
}
void correct_t(int Ttarget, int Xfinal, int Yfinal, int timeout, bool reversed, bool async) {
    long startTime = pros::millis();
    long elapsedTime = 0;    
    
    int Tdelta = chassis.getPose().theta;
    while (elapsedTime < 4000) {
        if (reversed) {
            chassis.moveToPose(Xfinal, Yfinal, Ttarget-Tdelta, timeout, {.forwards=false}, async);
        }
        else {
            chassis.moveToPose(Xfinal, Yfinal, Ttarget-Tdelta, timeout, {}, async);
        }
        elapsedTime = pros::millis() - startTime;
    }
}

void midGoalReset() {
    int f = fdist_sens.get_distance();
    int b = bdist_sens.get_distance();
    int l = ldist_sens.get_distance();
    int r = rdist_sens.get_distance();

}