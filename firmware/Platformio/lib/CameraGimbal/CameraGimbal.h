#ifndef CAMERAGIMBAL_H_
#define CAMERAGIMBAL_H_

#include <Arduino.h>
#include <Servo.h>

#define ANGLE_MAX 180
#define ANGLE_MIN 0
#define ANGLE_CENTER 90

typedef struct {
    Servo *s_pan;
    Servo *s_tilt;
    uint8_t pan_pos;
    uint8_t tilt_pos;
} CameraGimbal_t;

void CameraGimbal_Init();
void CameraGimbal_Set_Pan(const uint8_t &val);
void CameraGimbal_Set_Tilt(const uint8_t &val);
void CameraGimbal_Set_PanTilt(const uint8_t &val);
uint8_t CameraGimbal_Get_Pan();
uint8_t CameraGimbal_Get_Tilt();



#endif