#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

#include <Arduino.h>
#include "pinout.h"

#define ANALOG_WRITE_VAL 200
#define FREQ 1000

// Function Macros
#define AXIS_FORWARD(in1, in2, pwm) \
    analogWrite(in1, 255-pwm); \
    digitalWrite(in2, HIGH);

#define AXIS_BACKWARD(in1, in2, pwm) \
    analogWrite(in1, pwm); \
    digitalWrite(in2, LOW); 

#define AXIS_FORWARD_FLIPPED(in1, in2, pwm) \
    analogWrite(in1, pwm); \
    digitalWrite(in2, LOW);

#define AXIS_BACKWARD_FLIPPED(in1, in2, pwm) \
    analogWrite(in1, 255-pwm); \
    digitalWrite(in2, HIGH); 

#define AXIS_OFF(in1, in2) \
    analogWrite(in1, 0); \
    digitalWrite(in2, LOW);

// Wheel directions
#define FRFOR(PWM) AXIS_FORWARD_FLIPPED(FR_IN3_PIN, FR_IN4_PIN, PWM+10) //this is correct
#define FLFOR(PWM) AXIS_FORWARD(FL_IN1_PIN, FL_IN2_PIN, PWM+25) //this is correct
#define BRFOR(PWM) AXIS_FORWARD(BR_IN1_PIN, BR_IN2_PIN, PWM+25)
#define BLFOR(PWM) AXIS_FORWARD_FLIPPED(BL_IN3_PIN, BL_IN4_PIN, PWM)
#define FRBACK(PWM) AXIS_BACKWARD_FLIPPED(FR_IN3_PIN, FR_IN4_PIN, PWM+10)
#define FLBACK(PWM) AXIS_BACKWARD(FL_IN1_PIN, FL_IN2_PIN, PWM+25)
#define BRBACK(PWM) AXIS_BACKWARD(BR_IN1_PIN, BR_IN2_PIN, PWM+25)
#define BLBACK(PWM) AXIS_BACKWARD_FLIPPED(BL_IN3_PIN, BL_IN4_PIN, PWM)

void Locomotion_Init_Axis(uint8_t in1, uint8_t in2);

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