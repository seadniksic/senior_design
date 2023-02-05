#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <Arduino.h>

#include "pinout.h"

void joystick_init();

inline uint8_t dpad_up_pressed()
{
    return digitalRead(DPAD_UP_PIN);
}

inline uint8_t dpad_down_pressed()
{
    return digitalRead(DPAD_DOWN_PIN);
}

inline uint8_t dpad_right_pressed()
{
    return digitalRead(DPAD_RIGHT_PIN);
}

inline uint8_t dpad_left_pressed()
{
    return digitalRead(DPAD_LEFT_PIN);
}


#endif