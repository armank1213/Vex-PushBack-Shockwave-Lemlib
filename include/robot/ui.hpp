#pragma once

#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep

// UI state variables
extern int autonSelection;
extern int colorSortMode;

// UI label objects
extern lv_obj_t *poseXYlabel;
extern lv_obj_t *poseTlabel;

// Button event handlers);
void left_ButtonEvent(lv_event_t *e);
void right_ButtonEvent(lv_event_t *e);
void red_colorSort(lv_event_t *e);
void blue_colorSort(lv_event_t *e);

// Button creation functions
void left_Button(void);
void right_Button(void);
void skillsButton(void);
void bluecolorSortButton(void);

// Label creation functions
void poseXY_Label(void);
void poseT_Label(void);

// Label update task
void update_VHP_Labels(void* param);

// Initialize UI
void initializeUI();

