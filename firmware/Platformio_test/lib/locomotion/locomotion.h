#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

#include <Arduino.h>

#include "pinout.h"


#define FRFOR axis_forward(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN)
#define FLFOR axis_forward(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN)
#define BRFOR axis_forward(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN)
#define BLFOR axis_forward(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN)
#define FRBACK axis_backward(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN)
#define FLBACK axis_backward(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN)
#define BRBACK axis_backward(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN)
#define BLBACK axis_backward(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN)
#define BLON axis_on(BL_EN_PIN)
#define BRON axis_on(BR_EN_PIN)

#define FREQ 1000

void init_axis(uint8_t in1, uint8_t in2, uint8_t en);
void axis_forward(uint8_t in1, uint8_t in2, uint8_t en);
void axis_backward(uint8_t in1, uint8_t in2, uint8_t en);
void axis_off(uint8_t in1, uint8_t in2, uint8_t en);
void axis_on(uint8_t en);

void drive_left();
void drive_right();
void drive_diag_FL();
void drive_diag_FR();
void lateral_arc_CW();
void lateral_arc_CCW();
void drive_forward();
void drive_backward();
void all_axis_off();

void locomotion_init();
void locomotion_run();


#endif