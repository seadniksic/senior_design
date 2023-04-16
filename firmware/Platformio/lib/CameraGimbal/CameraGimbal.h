#ifndef CAMERAGIMBAL_H_
#define CAMERAGIMBAL_H_

#include <Arduino.h>
#include <Servo.h>
#include <UartComms.h>

#define ANGLE_MAX 180
#define ANGLE_MIN 0
#define ANGLE_CENTER 90

//axis is flipped, 0 is straight up
#define MIN_TILT 40 //180-140
#define MAX_TILT 110 //180-70

#define START_SPEED 4
#define MAX_SERVO_SPEED 15
#define MIN_SERVO_SPEED 1

#define NEW_VAL_PERCENT (float)0.05
#define OLD_VAL_PERCENT (1.0f-NEW_VAL_PERCENT)

typedef struct {
    Servo *s_pan;
    Servo *s_tilt;
    uint8_t pan_pos;
    float pan_pos_f;
    float prev_pan_pos;
    uint8_t tilt_pos;
    float tilt_pos_f;
    float prev_tilt_pos;
    uint8_t servo_speed;
    uint8_t pan_home_pos;
    uint8_t tilt_home_pos;
} CameraGimbal_t;

void CameraGimbal_Init();
void CameraGimbal_Set_Pan(const uint8_t &val);
void CameraGimbal_Set_Tilt(const uint8_t &val);
void CameraGimbal_Set_PanTilt(const uint8_t &val);
void CameraGimbal_Run();
uint8_t CameraGimbal_Get_Pan();
uint8_t CameraGimbal_Get_Tilt();
void CameraGimbal_Increment_Pan();
void CameraGimbal_Increment_Tilt();
void CameraGimbal_Decrement_Pan();
void CameraGimbal_Decrement_Tilt();
void CameraGimbal_Increment_Speed();
void CameraGimbal_Decrement_Speed();
uint8_t CameraGimbal_Get_Speed();
void CameraGimbal_Home();
void CameraGimbal_SetHome();
void CameraGimbal_StoreAngles(SLAM_Data * sd);
void CameraGimbal_StoreAngles(GUI_Data * gd);
void CameraGimbal_StoreHomeAngles(GUI_Data * gd);



#endif