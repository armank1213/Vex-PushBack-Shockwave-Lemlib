#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/ui.hpp" // IWYU pragma: keep

// pneumatic mech functions

void matchloadToggle() {

    // Matchload piston variables
    static bool matchloadPistonToggle = false;
    static bool lastRightButtonState = false;

    // Matchload Pneumatics Toggle
    bool currentRight = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (currentRight && !lastRightButtonState) {
        matchloadPistonToggle = !matchloadPistonToggle;
        matchLoad.set_value(matchloadPistonToggle);
    }
    lastRightButtonState = currentRight;
}
void limiterToggle() {

    // Limiter piston variables
    static bool limiterPistonToggle = true;
    static bool lastYButtonState = false;
    
    // Limiter Pneumatics Toggle
    bool currentY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    if (currentY && !lastYButtonState) {
        limiterPistonToggle = !limiterPistonToggle;
        limiter.set_value(limiterPistonToggle);
        if (limiterPistonToggle == true) {
            limiter_light.set_led_pwm(100);
        } else {
            limiter_light.set_led_pwm(0);
        }
    }
    lastYButtonState = currentY;
}
void wingToggle() {

    static bool wingPistonToggle = false;
    static bool lastL1ButtonState = false;

    bool currentL1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        
    if (currentL1 && !lastL1ButtonState) {
        wingPistonToggle = !wingPistonToggle;
        wing.set_value(wingPistonToggle);
    }
    
    lastL1ButtonState = currentL1;
}

