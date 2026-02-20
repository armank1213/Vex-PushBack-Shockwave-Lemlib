#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/ui.hpp" // IWYU pragma: keep

// pneumatic mech functions

void matchloadToggle() {

    // Matchload piston variables
    static bool matchloadPistonToggle = false;
    static bool lastAButtonState = false;

    // Matchload Pneumatics Toggle
    bool currentA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (currentA && !lastAButtonState) {
        matchloadPistonToggle = !matchloadPistonToggle;
        matchLoad.set_value(matchloadPistonToggle);
    }
    lastAButtonState = currentA;
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
    static bool lastXButtonState = false;

    bool currentX = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        
    if (currentX && !lastXButtonState) {
        wingPistonToggle = !wingPistonToggle;
        wing.set_value(wingPistonToggle);
    }
    
    lastXButtonState = currentX;
}

