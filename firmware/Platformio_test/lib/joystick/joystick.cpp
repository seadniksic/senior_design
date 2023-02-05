#include "joystick.h"

void joystick_init()
{
    pinMode(DPAD_DOWN_PIN, INPUT);
    pinMode(DPAD_LEFT_PIN, INPUT);
    pinMode(DPAD_RIGHT_PIN, INPUT);
    pinMode(DPAD_UP_PIN, INPUT);
    pinMode(JOY_LED_PIN, OUTPUT);

}


