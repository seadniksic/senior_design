#include "Joystick.h"
#include <string.h>
#include "Locomotion.h"

joy_state_t joy_state = {0};


void Joystick::init()
{

}

void Joystick::print()
{
    Serial.print(joy_state.buttons.button_bits.BTN_A);
    Serial.print(joy_state.buttons.button_bits.BTN_B);
    Serial.print(joy_state.buttons.button_bits.BTN_X);
    Serial.print(joy_state.buttons.button_bits.BTN_Y);
    Serial.print(joy_state.buttons.button_bits.BTN_START);
    Serial.print(joy_state.buttons.button_bits.BTN_SELECT);
    Serial.print(joy_state.buttons.button_bits.DPAD_UP);
    Serial.print(joy_state.buttons.button_bits.DPAD_DOWN);
    Serial.print(joy_state.buttons.button_bits.DPAD_LEFT);
    Serial.print(joy_state.buttons.button_bits.DPAD_RIGHT);
    Serial.print(joy_state.buttons.button_bits.BTN_THUMBR);
    Serial.print(joy_state.buttons.button_bits.BTN_THUMBL);
    Serial.print(joy_state.buttons.button_bits.BTN_TR);
    Serial.print(joy_state.buttons.button_bits.BTN_TL);
    Serial.printf(", %d", joy_state.ljoy_x);
    Serial.printf(", %d", joy_state.ljoy_y);
    Serial.printf(", %d", joy_state.rjoy_x);
    Serial.printf(", %d", joy_state.rjoy_y);
    Serial.printf(", %d", joy_state.tl);
    Serial.printf(", %d", joy_state.tr);
    Serial.println("");

}

void Joystick::store_joy_state(Joystick_Input &js_in)
{
    uint32_t buttons_in = (uint32_t)js_in.get_button();
    joy_state.buttons.button_state = buttons_in;
    joy_state.ljoy_x = js_in.get_LJOY_X();
    joy_state.ljoy_y = js_in.get_LJOY_Y();
    joy_state.rjoy_x = js_in.get_RJOY_X();
    joy_state.rjoy_y = js_in.get_RJOY_Y();
    joy_state.tl = js_in.get_TL();
    joy_state.tr = js_in.get_TR();
}

void Joystick::run()
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


