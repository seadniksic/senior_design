#ifndef LIGHTBAR_H_
#define LIGHTBAR_H_

#include <cstdint>
#include "pinout.h"
#include <Arduino.h>

#define BRIGHTNESS_STEP 10

inline void LightBar_Init()
{
    pinMode(LIGHTBAR_PIN, OUTPUT);
    analogWrite(LIGHTBAR_PIN, 0);
}

inline void LightBar_Brightness(const uint8_t &val)
{
    analogWrite(LIGHTBAR_PIN, val);
}

inline void LightBar_State(const bool &state)
{
    analogWrite(LIGHTBAR_PIN, state == true ? 255 : 0);
}

#endif
