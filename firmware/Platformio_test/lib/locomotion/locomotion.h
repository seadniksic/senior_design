#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

#include <Arduino.h>

//Notes regarding PWM https://www.pjrc.com/teensy/td_pulse.html
// Looks pins 2 and 3 are on the same time timer
// and so are 28 and 29
// all of them have a default frequency of 4.482 kHz


// BR = Back Right
#define BR_IN1_PIN 30
#define BR_IN2_PIN 31
#define BR_EN_PIN 2

// BL = Back Left
#define BL_IN3_PIN 26
#define BL_IN4_PIN 27
#define BL_EN_PIN 3

// FR = Front Right
#define FR_IN3_PIN 34
#define FR_IN4_PIN 35
#define FR_EN_PIN 29

// FL = Front Left
#define FL_IN1_PIN 36
#define FL_IN2_PIN 37
#define FL_EN_PIN 28

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
void drive_diag_FL();
void drive_diag_FR();
void lateral_arc_CW();
void lateral_arc_CCW();
void drive_forward();
void all_axis_off();

void locomotion_init();
void locomotion_run();


#endif