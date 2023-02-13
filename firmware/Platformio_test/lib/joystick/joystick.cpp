#include "joystick.h"
#include <String>
#include "locomotion.h"

void joystick_init()
{
#ifdef INIT_DPAD
    pinMode(DPAD_DOWN_PIN, INPUT);
    pinMode(DPAD_LEFT_PIN, INPUT);
    pinMode(DPAD_RIGHT_PIN, INPUT);
    pinMode(DPAD_UP_PIN, INPUT);
#endif
#ifdef INIT_AUX
    pinMode(JOY_LED_PIN, OUTPUT);
    pinMode(JOY_HOMEBTN_PIN, INPUT); // need to put the pins in for this
#endif

    //analog pins need no configuration

}

void joystick_print()
{
#ifdef PRINT_DPAD
    Serial.print("[DPAD]: ");
    Serial.print("U: " );
    Serial.print(dpad_up_pressed());
    Serial.print(", D: ");
    Serial.print(dpad_down_pressed());
    Serial.print(", R: ");
    Serial.print(dpad_right_pressed());
    Serial.print(", L: ");
    Serial.print(dpad_left_pressed());
    Serial.print(";\t");
#endif
#ifdef PRINT_AUX
    Serial.print("[AUX]: BTN: ");
    Serial.print(homebtn_pressed());
    Serial.print(";\t");
#endif
#ifdef PRINT_RJOY
    Serial.print("[RJOY]: ");
    Serial.print("X: ");
    Serial.print(analogRead(JOY_RY_PIN)); //because theyre physically switched
    Serial.print(", Y: ");
    Serial.print(analogRead(JOY_RX_PIN));
    Serial.print(";\t");
#endif
#ifdef PRINT_LJOY
    Serial.print("[LJOY]: ");
    Serial.print("X: ");
    Serial.print(analogRead(JOY_LY_PIN)); //because the switches are rotated
    Serial.print(", Y: ");
    Serial.print(analogRead(JOY_LX_PIN));
    Serial.print(";\t");
#endif
    Serial.print("\n");
}

void joystick_run()
{

    

    //check for controller input
    if(ljoy_deadzone() && !dpad_up_pressed() && !dpad_left_pressed() && !dpad_right_pressed() && !dpad_down_pressed())
    {
        all_axis_off();
        return;
    }

    /* check left joystick */

    // check for compound motions first
    if(ljoy_up() && ljoy_right())
    {
        Serial.println("diag fr");
        drive_diag_FR();
        return;
    }
    if(ljoy_up() && ljoy_left())
    {
        Serial.println("diag fl");
        drive_diag_FL();
        return;
    }
    if(ljoy_down() && ljoy_right())
    {
        Serial.println("diag br");
        drive_diag_BR();
        return;
    }
    if(ljoy_down() && ljoy_left())
    {
        Serial.println("diag bl");
        drive_diag_BL();
        return;
    }


    // check for simple motions next
    if(ljoy_up())
    {
        Serial.println("forward");
        drive_forward();
    }
    else if(ljoy_down())
    {
        Serial.println("back");
        drive_backward();
    }
    else if(ljoy_right())
    {
        Serial.println("right");
        drive_right();
    }
    else if(ljoy_left())
    {
        Serial.println("left");
        drive_left();
    }
    

    /* D-PAD */

    // check for no motion

    if(dpad_up_pressed())
    {
        Serial.println("CW");
        rotate_CW();
    }
    else if(dpad_down_pressed())
    {
        Serial.println("CWW");
        rotate_CCW();
    }

}
