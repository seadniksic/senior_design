#include "Joystick.h"
#include <string.h>
#include "Locomotion.h"

static joy_state_t joy_state = {0};
static control_state_t control_state = {0}; 


void Joystick_Init()
{
    control_state.control.bits.turn_off = 1;
}

void Joystick_Print()
{
    Serial.print(joy_state.buttons.bits.BTN_A);
    Serial.print(joy_state.buttons.bits.BTN_B);
    Serial.print(joy_state.buttons.bits.BTN_X);
    Serial.print(joy_state.buttons.bits.BTN_Y);
    Serial.print(" ");
    Serial.print(joy_state.buttons.bits.BTN_START);
    Serial.print(joy_state.buttons.bits.BTN_SELECT);
    Serial.print(joy_state.buttons.bits.DPAD_UP);
    Serial.print(joy_state.buttons.bits.DPAD_DOWN);
    Serial.print(" ");
    Serial.print(joy_state.buttons.bits.DPAD_LEFT);
    Serial.print(joy_state.buttons.bits.DPAD_RIGHT);
    Serial.print(joy_state.buttons.bits.BTN_THUMBR);
    Serial.print(joy_state.buttons.bits.BTN_THUMBL);
    Serial.print(" ");
    Serial.print(joy_state.buttons.bits.BTN_TR);
    Serial.print(joy_state.buttons.bits.BTN_TL);
    Serial.print(" ");
    Serial.printf(", %d", joy_state.ljoy_x);
    Serial.printf(", %d", joy_state.ljoy_y);
    Serial.printf(", %d", joy_state.rjoy_x);
    Serial.printf(", %d", joy_state.rjoy_y);
    Serial.printf(", %d", joy_state.tl);
    Serial.printf(", %d", joy_state.tr);
    Serial.print(" ");
    Serial.print(control_state.control.bits.drive_mode);
    Serial.print(control_state.control.bits.turn_off);
    Serial.println("");

}

void Joystick_Store_State(Joystick_Input &js_in)
{
    // Store the previous buttons states in prev_buttons
    joy_state.prev_buttons.state = joy_state.buttons.state;
    
    // Store current state of the contoller
    joy_state.buttons.state = (uint32_t)js_in.get_button();;
    joy_state.ljoy_x = js_in.get_LJOY_X();
    joy_state.ljoy_y = js_in.get_LJOY_Y();
    joy_state.rjoy_x = js_in.get_RJOY_X();
    joy_state.rjoy_y = js_in.get_RJOY_Y();
    joy_state.tl = js_in.get_TL();
    joy_state.tr = js_in.get_TR();

    // Perform processing for control bits
    // Capture on falling edge
    if(joy_state.prev_buttons.bits.BTN_A == 1 && \
        joy_state.buttons.bits.BTN_A == 0)
    {
        control_state.control.bits.drive_mode = !control_state.control.bits.drive_mode;
    }

    if(joy_state.prev_buttons.bits.BTN_X == 1 && \
        joy_state.buttons.bits.BTN_X == 0)
    {
        control_state.control.bits.turn_off = !control_state.control.bits.turn_off;
    }


}

void Joystick_Run()
{
    if(!Joystick_Input_Present() || IN_POWER_DOWN_MODE)
    {
        Locomotion_All_Axis_Off();
        return;
    }

    if(IN_STRAIGHT_DRIVE_MODE)
    {
        if(LJOY_UP)
        {
            Locomotion_Drive_Forward(Joystick_Map(joy_state.ljoy_y));
        }
        else if(LJOY_DOWN)
        {
            Locomotion_Drive_Backward(Joystick_Map(joy_state.ljoy_y));
        }
        else if(LJOY_RIGHT)
        {
            Locomotion_Drive_Right();
        }
        else if(LJOY_LEFT)
        {
            Locomotion_Drive_Left();
        }
    }
    else
    {
        if(LJOY_UR)
        {
            Locomotion_All_Axis_Off();
            Locomotion_Drive_Diag_FR();
        }
        else if(LJOY_UL)
        {
            Locomotion_All_Axis_Off();
            Locomotion_Drive_Diag_FL();
        }
        else if(LJOY_DR)
        {
            Locomotion_All_Axis_Off();
            Locomotion_Drive_Diag_BR();
        }
        else if(LJOY_DL)
        {
            Locomotion_All_Axis_Off();
            Locomotion_Drive_Diag_BL();
        }
    }

    if(RJOY_LEFT)
    {
        Locomotion_Rotate_CCW();
    }
    else if(RJOY_RIGHT)
    {
        Locomotion_Rotate_CW();
    }
}

bool Joystick_Input_Present()
{
    return !(
        BTN_NONE_PRESSED && \
        LJOY_DEADZONE_XY && \
        RJOY_DEADZONE_XY && \
        TRIGGER_LEFT_DEADZONE && \
        TRIGGER_RIGHT_DEADZONE
        );
}

uint8_t Joystick_Map(const int32_t &val)
{
    float temp = (float)abs(val);
    const uint8_t result = (uint8_t)((float)(temp - IN_MIN) * RATIO + OUT_MIN);
    return result;
}




#if 0

    //check for controller input
    if(ljoy_deadzone() && !joy_state.dpad_up && !joy_state.dpad_left && !joy_state.dpad_right && !joy_state.dpad_down && !joy_state.fnc_btn)
    {
        Locomotion_All_Axis_Off();
        return;
    }

    /* check left joystick */

    // check for compound motions first
    if(joy_state.drive_mode == DIAG_DRIVE_MODE)
    {
        if(ljoy_up() && ljoy_right())
        {
            Serial.println("diag fr");
            Locomotion_Drive_Diag_FR();
            return;
        }
        else if(ljoy_up() && ljoy_left())
        {
            Serial.println("diag fl");
            Locomotion_Drive_Diag_FL();
            return;
        }
        else if(ljoy_down() && ljoy_right())
        {
            Serial.println("diag br");
            Locomotion_Drive_Diag_BR();
            return;
        }
        else if(ljoy_down() && ljoy_left())
        {
            Serial.println("diag bl");
            Locomotion_Drive_Diag_BL();
            return;
        }
    }


    if(joy_state.drive_mode == STRAIGHT_DRIVE_MODE)
    {
        // check for simple motions next
        if(ljoy_up())
        {
            Serial.println("forward");
            Locomotion_Drive_Forward();
        }
        else if(ljoy_down())
        {
            Serial.println("back");
            Locomotion_Drive_Backward();
        }
        else if(ljoy_right())
        {
            Serial.println("right");
            Locomotion_Drive_Right();
        }
        else if(ljoy_left())
        {
            Serial.println("left");
            Locomotion_Drive_Left();
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
        Locomotion_Rotate_CW();
    }
    else if(joy_state.dpad_left)
    {
        Serial.println("CWW");
        Locomotion_Rotate_CCW();
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


