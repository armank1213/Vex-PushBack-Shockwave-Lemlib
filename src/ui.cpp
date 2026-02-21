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

void left_switchEvent(lv_event_t *e) {
    
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *leftobj = lv_event_get_target_obj(e);
    LV_UNUSED(leftobj);
    if (code == LV_EVENT_VALUE_CHANGED) {
        left_count++;
    }
}

void right_switchEvent(lv_event_t *e) {
    
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *rightobj = lv_event_get_target_obj(e);
    LV_UNUSED(rightobj);
    if (code == LV_EVENT_VALUE_CHANGED) {
        right_count++;
    }
}

void skills_switchEvent(lv_event_t *e) {
    
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *skillsobj = lv_event_get_target_obj(e);
    LV_UNUSED(skillsobj);
    if (code == LV_EVENT_VALUE_CHANGED) {
        skills_count++;
    } 
}


// switch functions
void left_switch(void) {
    lv_obj_t *leftswitch = lv_switch_create(lv_screen_active());
    lv_obj_add_event_cb(leftswitch, left_switchEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *leftswitch_label = lv_label_create(leftswitch);
    lv_label_set_text(leftswitch_label, "Left Auton");
    lv_obj_align(leftswitch, LV_ALIGN_TOP_LEFT, 0,30);
}

void right_switch(void) {
    lv_obj_t *rightswitch = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(rightswitch, right_switchEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *rightswitch_label = lv_label_create(rightswitch);
    lv_label_set_text(rightswitch_label, "Right Auton");
    lv_obj_align(rightswitch, LV_ALIGN_TOP_RIGHT,0,80);
    lv_obj_add_flag(rightswitch, LV_OBJ_FLAG_EVENT_BUBBLE);

}

void skills_switch(void) {
    lv_obj_t *skillsswitch= lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(skillsswitch, skills_switchEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *skillsswitch_label = lv_label_create(skillsswitch);
    lv_label_set_text(skillsswitch_label, "Skills Auton");
    lv_obj_align(skillsswitch, LV_ALIGN_BOTTOM_MID,0,-60);
    lv_obj_add_flag(skillsswitch, LV_OBJ_FLAG_EVENT_BUBBLE);

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
    left_Button();
    right_Button();
    skills_switch();

    // Create a pros task that updates the vertical and horizontal rotation sensor measurements on the brain screen
    pros::Task xyt_update(update_xyt_labels, nullptr, (uint32_t)TASK_PRIORITY_MAX, (uint16_t)TASK_STACK_DEPTH_DEFAULT); // prev task priority default
}

