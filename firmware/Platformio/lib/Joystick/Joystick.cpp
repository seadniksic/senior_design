#include "Joystick.h"
#include <string.h>
#include "Locomotion.h"

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
    Serial.print(joy_state.fnc_btn);
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

    Serial.print("[DRIVE MODE]: ");
    Serial.print(joy_state.drive_mode);
    Serial.print("\n");
}

void joystick::store_joy_state()
{
    //store prev state
    joy_state.dpad_down_prev = joy_state.dpad_down;
    joy_state.fnc_btn_prev = joy_state.fnc_btn;

    joy_state.dpad_down = dpad_down_pressed();
    joy_state.dpad_left = dpad_left_pressed();
    joy_state.dpad_right = dpad_right_pressed();
    joy_state.dpad_up = dpad_up_pressed();
    joy_state.fnc_btn = fncbtn_pressed();
    joy_state.ljoy_x = analogRead(JOY_LY_PIN); //intentionally switched because joystick is rotated.
    joy_state.ljoy_y = analogRead(JOY_LX_PIN);
    joy_state.rjoy_x = analogRead(JOY_RY_PIN);
    joy_state.rjoy_y = analogRead(JOY_RX_PIN);


}

void joystick::run()
{

#ifdef DRIVE_MODE_1

    //check for controller input
    if(ljoy_deadzone() && !joy_state.dpad_up && !joy_state.dpad_left && !joy_state.dpad_right && !joy_state.dpad_down && !joy_state.fnc_btn)
    {
        all_axis_off();
        return;
    }

    /* check left joystick */

    // check for compound motions first
    if(joy_state.drive_mode == DIAG_DRIVE_MODE)
    {
        if(ljoy_up() && ljoy_right())
        {
            Serial.println("diag fr");
            drive_diag_FR();
            return;
        }
        else if(ljoy_up() && ljoy_left())
        {
            Serial.println("diag fl");
            drive_diag_FL();
            return;
        }
        else if(ljoy_down() && ljoy_right())
        {
            Serial.println("diag br");
            drive_diag_BR();
            return;
        }
        else if(ljoy_down() && ljoy_left())
        {
            Serial.println("diag bl");
            drive_diag_BL();
            return;
        }
    }


    if(joy_state.drive_mode == STRAIGHT_DRIVE_MODE)
    {
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
    }
    

    /* D-PAD */

    // check for no motion

    // complex motions must go first
    if(joy_state.dpad_right && ljoy_up())
    {
        Serial.println("drive forward and turn right");
        FLFOR(ANALOG_WRITE_VAL);
        BLFOR(ANALOG_WRITE_VAL);
        FRFOR(ANALOG_WRITE_VAL - 100);
        BRFOR(ANALOG_WRITE_VAL - 100);

    }
    else if(joy_state.dpad_left && ljoy_up())
    {
        Serial.println("drive forward and turn right");
        FLFOR(ANALOG_WRITE_VAL - 100);
        BLFOR(ANALOG_WRITE_VAL - 100);
        FRFOR(ANALOG_WRITE_VAL);
        BRFOR(ANALOG_WRITE_VAL);
    }
    else if(joy_state.dpad_right)
    {
        Serial.println("CW");
        rotate_CW();
    }
    else if(joy_state.dpad_left)
    {
        Serial.println("CWW");
        rotate_CCW();
    }

    // check function inputs
    // need some additional logic to only detect edge changes
    if(joy_state.dpad_down && (joy_state.dpad_down_prev == false))
    {
        Serial.println("changing drive modes");
        joy_state.drive_mode = joy_state.drive_mode == \
                STRAIGHT_DRIVE_MODE ? DIAG_DRIVE_MODE : STRAIGHT_DRIVE_MODE;
    }

    if(joy_state.fnc_btn && (joy_state.fnc_btn_prev == false))
    {
        Serial.println("resetting camera pos");
        //TODO: reset camera pos
    }
    


#endif

}


