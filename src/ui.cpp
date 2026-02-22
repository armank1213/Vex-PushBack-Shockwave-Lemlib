#include "robot/ui.hpp"
#include "liblvgl/misc/lv_event.h"
#include "robot/chassis_config.hpp"
#include "pros/rtos.hpp"

// UI state variables
int autonSelection = 0;
int colorSortMode = 0; // 0 for red, 1 for blue

static int left_count = 0;
static int right_count = 0;
static int skills_count = 0;

// UI label objects
lv_obj_t *xy_label = NULL; // x and y coords
lv_obj_t *t_label = NULL; // theta (direction robot is facing)

// Button event handlers

void left_buttonEvent(lv_event_t *e) {
    autonSelection = 0;
}

void right_buttonEvent(lv_event_t *e) {
    autonSelection = 1;
}

void skills_buttonEvent(lv_event_t *e) {
    autonSelection = 2;
}

// switch functions
void left_button(void) {
    lv_obj_t *leftbutton = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(leftbutton, left_buttonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *leftbutton_label = lv_label_create(leftbutton);
    lv_label_set_text(leftbutton_label, "Left Auton");
    lv_obj_align(leftbutton, LV_ALIGN_TOP_LEFT, 0,30);
}

void right_button(void) {
    lv_obj_t *rightbutton = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(rightbutton, right_buttonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *rightbutton_label = lv_label_create(rightbutton);
    lv_label_set_text(rightbutton_label, "Right Auton");
    lv_obj_align(rightbutton, LV_ALIGN_TOP_RIGHT,0,80);
    lv_obj_add_flag(rightbutton, LV_OBJ_FLAG_EVENT_BUBBLE);

}

void skills_button(void) {
    lv_obj_t *skillsbutton= lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(skillsbutton, skills_buttonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *skillsbutton_label = lv_label_create(skillsbutton);
    lv_label_set_text(skillsbutton_label, "Skills Auton");
    lv_obj_align(skillsbutton, LV_ALIGN_BOTTOM_MID,0,-60);
    lv_obj_add_flag(skillsbutton, LV_OBJ_FLAG_EVENT_BUBBLE);

}

// Label creation functions
void xyDisplay(void) {
    xy_label = lv_label_create(lv_screen_active());
    lv_obj_align(xy_label,LV_ALIGN_TOP_MID,0,30);
}

void tDisplay(void) {
    t_label = lv_label_create(lv_screen_active());
    lv_obj_align(t_label,LV_ALIGN_TOP_MID,0,60);
}

// Label update task
void update_xyt_labels(void* param) {
    while (true) {
        if (xy_label != NULL) {
            lv_label_set_text_fmt(xy_label, "Pose: (%i, %i)", (int)chassis.getPose().x, (int)chassis.getPose().y);
        }
        if (t_label != NULL) {
            lv_label_set_text_fmt(t_label, "Theta: %i", (int)chassis.getPose().theta);
        }
        pros::delay(50);
    }
}

// Initialize UI
void initializeUI() {

    lv_obj_t *brain_screen = lv_obj_create(NULL);
    lv_screen_load(brain_screen);

    xyDisplay();
    tDisplay();
    left_button();
    right_button();
    skills_button();

    // Create a pros task that updates the vertical and horizontal rotation sensor measurements on the brain screen
    pros::Task xyt_update(update_xyt_labels, nullptr, (uint32_t)TASK_PRIORITY_MAX, (uint16_t)TASK_STACK_DEPTH_DEFAULT); // prev task priority default
}

