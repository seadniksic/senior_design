#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

#include <Arduino.h>
#include "pinout.h"

#define ANALOG_WRITE_VAL 200
#define FREQ 1000

// Function Macros
#define AXIS_FORWARD(in1, in2, en, pwm) \
  analogWrite(en, pwm); \
  digitalWrite(in1, LOW); \
  digitalWrite(in2, HIGH);

#define AXIS_BACKWARD(in1, in2, en, pwm) \
    analogWrite(en, pwm); \
    digitalWrite(in1, HIGH); \
    digitalWrite(in2, LOW); 

#define AXIS_OFF(in1, in2, en) \
    digitalWrite(en, LOW); \
    digitalWrite(in1, LOW); \
    digitalWrite(in2, LOW);

// Wheel directions
#define FRFOR(PWM) AXIS_FORWARD(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN, PWM)
#define FLFOR(PWM) AXIS_FORWARD(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN, PWM)
#define BRFOR(PWM) AXIS_FORWARD(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN, PWM)
#define BLFOR(PWM) AXIS_FORWARD(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN, PWM)
#define FRBACK(PWM) AXIS_BACKWARD(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN, PWM)
#define FLBACK(PWM) AXIS_BACKWARD(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN, PWM)
#define BRBACK(PWM) AXIS_BACKWARD(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN, PWM)
#define BLBACK(PWM) AXIS_BACKWARD(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN, PWM)

// Wheel EN
#define BLON digitalWrite(BL_EN_PIN, HIGH);
#define BRON digitalWrite(BR_EN_PIN, HIGH);
#define FRON digitalWrite(FR_EN_PIN, HIGH);
#define FLON digitalWrite(FL_EN_PIN, HIGH);

void Locomotion_Init_Axis(uint8_t in1, uint8_t in2, uint8_t en);

void Locomotion_Drive_Left();
void Locomotion_Drive_Left(const uint8_t &val);

void Locomotion_Drive_Right();
void Locomotion_Drive_Right(const uint8_t &val);

void Locomotion_Drive_Forward();
void Locomotion_Drive_Forward(const uint8_t &val);
void Locomotion_Differential_Drive_Forward(const uint8_t &pwm_left, const uint8_t &pwm_right);

void Locomotion_Drive_Backward();
void Locomotion_Drive_Backward(const uint8_t &val);
void Locomotion_Differential_Drive_Backward(const uint8_t &pwm_left, const uint8_t &pwm_right);

void Locomotion_Drive_Diag_FL();
void Locomotion_Drive_Diag_FL(const uint8_t &val);

void Locomotion_Drive_Diag_FR();
void Locomotion_Drive_Diag_FR(const uint8_t &val);

void Locomotion_Drive_Diag_BL();
void Locomotion_Drive_Diag_BL(const uint8_t &val);

void Locomotion_Drive_Diag_BR();
void Locomotion_Drive_Diag_BR(const uint8_t &val);

void Locomotion_Rotate_CW();
void Locomotion_Rotate_CW(const uint8_t &val);

void Locomotion_Rotate_CCW();
void Locomotion_Rotate_CCW(const uint8_t &val);


void Locomotion_Lateral_Arc_CW();
void Locomotion_Lateral_Arc_CCW();
void Locomotion_All_Axis_Off();

void Locomotion_Init();
void Locomotion_Run();


#endif