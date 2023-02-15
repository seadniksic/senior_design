#include "joystick.h"
#include <String>
#include "locomotion.h"

joy_state_t joy_state = {0};


void joystick::init()
{
#ifdef INIT_DPAD
    pinMode(DPAD_DOWN_PIN, INPUT);
    pinMode(DPAD_LEFT_PIN, INPUT);
    pinMode(DPAD_RIGHT_PIN, INPUT);
    pinMode(DPAD_UP_PIN, INPUT);
#endif
#ifdef INIT_AUX
    pinMode(JOY_LED_PIN, OUTPUT);
    pinMode(JOY_FNCBTN_PIN, INPUT); // need to put the pins in for this
#endif

    //analog pins need no configuration

}

void joystick::print()
{
#ifdef PRINT_DPAD
    Serial.print("[DPAD]: ");
    Serial.print("U: " );
    Serial.print(joy_state.dpad_up);
    Serial.print(", D: ");
    Serial.print(joy_state.dpad_down);
    Serial.print(", R: ");
    Serial.print(joy_state.dpad_right);
    Serial.print(", L: ");
    Serial.print(joy_state.dpad_left);
    Serial.print(";\t");
#endif
#ifdef PRINT_AUX
    Serial.print("[AUX]: BTN: ");
    Serial.print(fncbtn_pressed());
    Serial.print(";\t");
#endif
#ifdef PRINT_RJOY
    Serial.print("[RJOY]: ");
    Serial.print("X: ");
    Serial.print(joy_state.rjoy_x); 
    Serial.print(", Y: ");
    Serial.print(joy_state.rjoy_y);
    Serial.print(";\t");
#endif
#ifdef PRINT_LJOY
    Serial.print("[LJOY]: ");
    Serial.print("X: ");
    Serial.print(joy_state.ljoy_x); 
    Serial.print(", Y: ");
    Serial.print(joy_state.ljoy_y);
    Serial.print(";\t");
#endif
    Serial.print("\n");
}

void joystick::store_joy_state()
{
    joy_state.dpad_down = dpad_down_pressed();
    joy_state.dpad_left = dpad_left_pressed();
    joy_state.dpad_right = dpad_right_pressed();
    joy_state.dpad_up = dpad_up_pressed();
    joy_state.ljoy_x = analogRead(JOY_LY_PIN); //intentionally switched because joystick is rotated.
    joy_state.ljoy_y = analogRead(JOY_LX_PIN);
    joy_state.rjoy_x = analogRead(JOY_RY_PIN);
    joy_state.rjoy_y = analogRead(JOY_RX_PIN);
}

void joystick::run()
{
    //check for controller input
    if(ljoy_deadzone() && !joy_state.dpad_up && !joy_state.dpad_left && !joy_state.dpad_right && !joy_state.dpad_down)
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

    if(joy_state.dpad_up)
    {
        Serial.println("CW");
        rotate_CW();
    }
    else if(joy_state.dpad_down)
    {
        Serial.println("CWW");
        rotate_CCW();
    }

}


