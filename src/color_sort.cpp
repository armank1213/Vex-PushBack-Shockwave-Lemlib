#include "robot/color_sort.hpp"
#include "robot/motors.hpp"
#include "robot/hardware.hpp" // IWYU pragma: keep
#include <cstdlib>

void red_colorSort(int distance, double hue) {
    if (distance < 95) { // if an object is detected within 95 mm
/*      if ((hue >= 0 && hue <= 30)) {
            outtake(127);
        } else */
        if (hue >= 220 && hue <= 360) {
            outtake(127);
        }
    }
}

void blue_colorSort(int distance, double hue) {
    if (distance < 95) { // if an object is detected within 95 mm
            /*if ((hue >= 220 && hue <= 360)) {
                outtake(127);
            } else*/
            if (hue >= 0 && hue <= 30) {
                outtake(-127);
            }
    }
}