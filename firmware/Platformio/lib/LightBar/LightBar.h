#ifndef LIGHTBAR_H_
#define LIGHTBAR_H_

#include <cstdint>
#include "pinout.h"
#include <Arduino.h>

inline void LightBar_Init()
{
    pinMode(LIGHTBAR_PIN, OUTPUT);
}

inline void LightBar_Brightness(const uint8_t &val)
{
    analogWrite(LIGHTBAR_PIN, val);
}

inline void LightBar_State(const bool &state)
{
    digitalWrite(LIGHTBAR_PIN, state);
}

#endif
