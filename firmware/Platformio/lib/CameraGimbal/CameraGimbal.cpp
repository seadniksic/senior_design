#include "CameraGimbal.h"

#include "pinout.h"

static Servo Servo_Pan;
static Servo Servo_Tilt;
static CameraGimbal_t cameraGimbal = {0};

void CameraGimbal_Init()
{
    cameraGimbal.s_pan = &Servo_Pan;
    cameraGimbal.s_pan->attach(PAN_SERVO_PIN);
    CameraGimbal_Set_Pan(ANGLE_CENTER);

    cameraGimbal.s_tilt = &Servo_Tilt;
    cameraGimbal.s_tilt->attach(TILT_SERVO_PIN);
    CameraGimbal_Set_Tilt(ANGLE_CENTER);
    
}

void CameraGimbal_Set_Pan(const uint8_t &val)
{
    cameraGimbal.s_pan->write(val);
    cameraGimbal.pan_pos=val;
}

void CameraGimbal_Set_Tilt(const uint8_t &val)
{
    cameraGimbal.s_tilt->write(val);
    cameraGimbal.tilt_pos=val;
}

uint8_t CameraGimbal_Get_Pan()
{
    return cameraGimbal.pan_pos;
}

uint8_t CameraGimbal_Get_Tilt()
{
    return cameraGimbal.tilt_pos;
}

void CameraGimbal_Set_PanTilt(const uint8_t &val)
{
    CameraGimbal_Set_Pan(val);
    CameraGimbal_Set_Tilt(val);
}