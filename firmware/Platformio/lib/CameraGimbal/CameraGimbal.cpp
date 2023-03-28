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

    cameraGimbal.servo_speed = START_SPEED;
    cameraGimbal.pan_home_pos = ANGLE_CENTER;
    cameraGimbal.tilt_home_pos = ANGLE_CENTER;
}

void CameraGimbal_Set_Pan(const uint8_t &val)
{
    if(val > 180)
        return;
    cameraGimbal.s_pan->write(val);
    cameraGimbal.pan_pos=val;
}

void CameraGimbal_Set_Tilt(const uint8_t &val)
{
    if(val > 180 || val < MIN_TILT)
        return;
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

void CameraGimbal_Increment_Pan()
{
    CameraGimbal_Set_Pan(cameraGimbal.pan_pos + cameraGimbal.servo_speed);
}

void CameraGimbal_Increment_Tilt()
{
    CameraGimbal_Set_Tilt(cameraGimbal.tilt_pos + cameraGimbal.servo_speed);
}

void CameraGimbal_Decrement_Pan()
{
    CameraGimbal_Set_Pan(cameraGimbal.pan_pos - cameraGimbal.servo_speed);
}

void CameraGimbal_Decrement_Tilt()
{
    CameraGimbal_Set_Tilt(cameraGimbal.tilt_pos - cameraGimbal.servo_speed);
}

void CameraGimbal_Increment_Speed()
{
    if((cameraGimbal.servo_speed+1) <= MAX_SERVO_SPEED)
    {
        cameraGimbal.servo_speed++;
    }
    
}

void CameraGimbal_Decrement_Speed()
{
    if((cameraGimbal.servo_speed-1) >= MIN_SERVO_SPEED)
    {
        cameraGimbal.servo_speed--;
    }
}

uint8_t CameraGimbal_Get_Speed()
{
    return cameraGimbal.servo_speed;
}

void CameraGimbal_Home()
{
    Serial.println("thing!!");
    Serial.println(cameraGimbal.pan_home_pos);
    Serial.println(cameraGimbal.tilt_home_pos);
    CameraGimbal_Set_Pan(cameraGimbal.pan_home_pos);
    CameraGimbal_Set_Tilt(cameraGimbal.tilt_home_pos);
}

void CameraGimbal_SetHome()
{
    cameraGimbal.pan_home_pos = cameraGimbal.pan_pos;
    cameraGimbal.tilt_home_pos = cameraGimbal.tilt_pos;
}