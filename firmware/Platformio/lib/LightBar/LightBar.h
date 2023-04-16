#ifndef LIGHTBAR_H_
#define LIGHTBAR_H_

#include <cstdint>
#include "pinout.h"
#include <Arduino.h>

#define LIGHTBAR_BRIGHTNESS_STEP 7
#define LIGHTBAR_MIN_BRIGHTNESS 5
#define LIGHTBAR_PWM_FREQ 2000

inline void LightBar_Init()
{
    pinMode(LIGHTBAR_PIN, OUTPUT);
    analogWriteFrequency(LIGHTBAR_PIN, LIGHTBAR_PWM_FREQ);
    analogWrite(LIGHTBAR_PIN, LIGHTBAR_MIN_BRIGHTNESS);
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
