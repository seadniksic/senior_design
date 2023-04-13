#include "Joystick.h"
#include <string.h>
#include <Locomotion.h>
#include <CameraGimbal.h>
#include <LightBar.h>
#include <bno055.h>

static joy_state_t joy_state = {0};
static control_state_t control_state = {0}; 


void Joystick_Init()
{
    control_state.control.bits.turn_off = 1;
    control_state.headlight_brightness = BRIGHTNESS_STEP;
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
    Serial.print(control_state.control.bits.center_cams);
    Serial.println("");

}

void Joystick_Store_State(Joystick_Input *js_in)
{
    // Store the previous buttons states in prev_buttons
    joy_state.prev_buttons.state = joy_state.buttons.state;
    
    // Store current state of the contoller
    joy_state.buttons.state = (uint32_t)(js_in->get_button());
    joy_state.ljoy_x = js_in->get_LJOY_X();
    joy_state.ljoy_y = js_in->get_LJOY_Y();
    joy_state.rjoy_x = js_in->get_RJOY_X();
    joy_state.rjoy_y = js_in->get_RJOY_Y();
    joy_state.tl = js_in->get_TL();
    joy_state.tr = js_in->get_TR();

    // Perform processing for control bits
    // Capture on rising edge
    if(joy_state.prev_buttons.bits.BTN_X == 00&& \
        joy_state.buttons.bits.BTN_X == 1)
    {
        control_state.control.bits.turn_off = !control_state.control.bits.turn_off;
    }

    if(!IN_POWER_DOWN_MODE)
    {
        // Capture on rising edge
        if(joy_state.prev_buttons.bits.BTN_THUMBL == 0 && \
            joy_state.buttons.bits.BTN_THUMBL == 1)
        {
            control_state.control.bits.drive_mode = !control_state.control.bits.drive_mode;
        }

        // Capture on rising edge
        if(joy_state.prev_buttons.bits.BTN_B == 0 && \
            joy_state.buttons.bits.BTN_B == 1)
        {
            control_state.control.bits.center_cams = 1;
        }

        // capture on rising edge
        if(joy_state.prev_buttons.bits.BTN_START == 0 && \
            joy_state.buttons.bits.BTN_START == 1)
        {
            control_state.control.bits.inc_servo_speed = 1;
        }

        // capture on rising edge
        if(joy_state.prev_buttons.bits.BTN_SELECT == 0 && \
            joy_state.buttons.bits.BTN_SELECT == 1)
        {
            control_state.control.bits.dec_servo_speed = 1;
        }

        // capture on rising edge
        if(joy_state.prev_buttons.bits.BTN_Y == 0 && \
            joy_state.buttons.bits.BTN_Y == 1)
        {
            control_state.control.bits.reset_home_pos = 1;
        }

        // capture on rising edge
        if(joy_state.prev_buttons.bits.BTN_A == 0 && \
            joy_state.buttons.bits.BTN_A == 1)
        {
            control_state.control.bits.print_imu_calib = 1;
        }

        // capture on rising edge
        if(joy_state.prev_buttons.bits.BTN_THUMBR == 0 && \
            joy_state.buttons.bits.BTN_THUMBR == 1)
        {
            control_state.control.bits.write_calib_profile = 1;
        }
    }



}

void Joystick_Reset_State()
{
    // Store the previous buttons states in prev_buttons
    joy_state.prev_buttons.state = (uint32_t)0;
    
    // Store current state of the contoller
    joy_state.buttons.state = (uint32_t)0;
    joy_state.ljoy_x = 0;
    joy_state.ljoy_y = 0;
    joy_state.rjoy_x = 0;
    joy_state.rjoy_y = 0;
    joy_state.tl = 0;
    joy_state.tr = 0;
}

void Joystick_Run()
{
    // Check control and handle and pending actions first
    if(!IN_POWER_DOWN_MODE)
    {
        // Camera gimbal
        if(CENTER_CAMS)
        {
            control_state.control.bits.center_cams = 0;
            Serial.println("Centering cameras...");
            CameraGimbal_Home();
        }
        if(control_state.control.bits.inc_servo_speed)
        {
            control_state.control.bits.inc_servo_speed = 0;
            CameraGimbal_Increment_Speed();
        }
        if(control_state.control.bits.dec_servo_speed)
        {
            control_state.control.bits.dec_servo_speed = 0;
            CameraGimbal_Decrement_Speed();
        }
        if(control_state.control.bits.reset_home_pos)
        {
            control_state.control.bits.reset_home_pos = 0;
            Serial.println("setting new home pos");
            CameraGimbal_SetHome();
        }

        // imu stuff
        if(control_state.control.bits.print_imu_calib)
        {
            control_state.control.bits.print_imu_calib = 0;
            
            // Print calibration status
            bno055::print_calibration();

            if(bno055::enter_config_mode())
            {
                // Attempt to get calibration profile
                if(bno055::get_calib_profile())
                {
                    bno055::print_calib_profile();
                }
                
                // return to run mode
                bno055::enter_run_mode();
            }  
        }

        // if(control_state.control.bits.write_calib_profile)
        // {
        //     control_state.control.bits.write_calib_profile = 0;
        //     if(bno055::write_calib_profile(ACTUAL_CALIB_DATA))
        //     {
        //         Serial.println("successfully wrote calib profile");
        //     }
        //     else
        //     {
        //         Serial.println("FAILED TO WRITE CALIB PROFILE");
        //     }
        // }
    }
    

    
    // Check for joystick input
    if(!Joystick_Input_Present() || IN_POWER_DOWN_MODE)
    {
        Locomotion_All_Axis_Off();

        return;
    }

    // Handle LightBar
    if(DPAD_LEFT_PRESSED)
    {
        LightBar_Brightness(0);
    }
    else if(DPAD_RIGHT_PRESSED)
    {
        LightBar_Brightness(control_state.headlight_brightness);
    }
    else if(DPAD_UP_PRESSED)
    {
        if(control_state.headlight_brightness <= (255 - BRIGHTNESS_STEP))
        {
            control_state.headlight_brightness += BRIGHTNESS_STEP;
            LightBar_Brightness(control_state.headlight_brightness);
        }
    }
    else if(DPAD_DOWN_PRESSED)
    {
        if(control_state.headlight_brightness >= (BRIGHTNESS_STEP))
        {
            control_state.headlight_brightness -= BRIGHTNESS_STEP;
            LightBar_Brightness(control_state.headlight_brightness);
        }
    }

    // Handle joystick input
    if(IN_STRAIGHT_DRIVE_MODE)
    {
        if(LJOY_UP && RJOY_RIGHT)
        {
            const uint8_t mapped = Joystick_Map_Generic(Joystick_Map(joy_state.rjoy_x), 0, 255, 0, (OUT_MAX - OUT_MIN)/2);
            const uint8_t pwm_left = OUT_MAX - OUT_MIN + mapped;  // left side should move faster
            const uint8_t pwm_right = OUT_MAX - OUT_MIN - mapped;
            Locomotion_Differential_Drive_Forward(pwm_left, pwm_right);
        }
        else if(LJOY_UP && RJOY_LEFT)
        {
            const uint8_t mapped = Joystick_Map_Generic(Joystick_Map(joy_state.rjoy_x), 0, 255, 0, (OUT_MAX - OUT_MIN)/2);
            const uint8_t pwm_left = OUT_MAX - OUT_MIN - mapped; 
            const uint8_t pwm_right = OUT_MAX - OUT_MIN + mapped; // right_side should move faster
            Locomotion_Differential_Drive_Forward(pwm_left, pwm_right);
        }
        else if(LJOY_DOWN && RJOY_RIGHT)
        {
            const uint8_t mapped = Joystick_Map_Generic(Joystick_Map(joy_state.rjoy_x), 0, 255, 0, (OUT_MAX - OUT_MIN)/2);
            const uint8_t pwm_left = OUT_MAX - OUT_MIN + mapped; 
            const uint8_t pwm_right = OUT_MAX - OUT_MIN - mapped; // right_side should move faster
            Locomotion_Differential_Drive_Backward(pwm_left, pwm_right);
        }
        else if(LJOY_DOWN && RJOY_LEFT)
        {
            const uint8_t mapped = Joystick_Map_Generic(Joystick_Map(joy_state.rjoy_x), 0, 255, 0, (OUT_MAX - OUT_MIN)/2);
            const uint8_t pwm_left = OUT_MAX - OUT_MIN - mapped; 
            const uint8_t pwm_right = OUT_MAX - OUT_MIN + mapped; // right_side should move faster
            Locomotion_Differential_Drive_Backward(pwm_left, pwm_right);
        }
        
        else if(RJOY_RIGHT)
        {
            Locomotion_Rotate_CW(Joystick_Map(joy_state.rjoy_x));
        }
        else if(RJOY_LEFT)
        {
            Locomotion_Rotate_CCW(Joystick_Map(joy_state.rjoy_x));
        }
        else if(LJOY_UP)
        {
            Locomotion_Drive_Forward(Joystick_Map(joy_state.ljoy_y));
        }
        else if(LJOY_DOWN)
        {
            Locomotion_Drive_Backward(Joystick_Map(joy_state.ljoy_y));
        }
        else if(LJOY_RIGHT)
        {
            Locomotion_Drive_Right(Joystick_Map(joy_state.ljoy_x));
        }
        else if(LJOY_LEFT)
        {
            Locomotion_Drive_Left(Joystick_Map(joy_state.ljoy_x));
        }
    }
    else
    {
        if(LJOY_UR)
        {
            Locomotion_All_Axis_Off();
            // Locomotion_Drive_Diag_FR();
            Locomotion_Drive_Diag_FR(Joystick_Map(max(joy_state.ljoy_x, joy_state.rjoy_x)));
        }
        else if(LJOY_UL)
        {
            Locomotion_All_Axis_Off();
            // Locomotion_Drive_Diag_FL();
            Locomotion_Drive_Diag_FL(Joystick_Map(min(joy_state.ljoy_x, joy_state.rjoy_x)));
        }
        else if(LJOY_DR)
        {
            Locomotion_All_Axis_Off();
            // Locomotion_Drive_Diag_BR();
            Locomotion_Drive_Diag_BR(Joystick_Map(max(joy_state.ljoy_x, joy_state.rjoy_x)));
        }
        else if(LJOY_DL)
        {
            Locomotion_All_Axis_Off();
            // Locomotion_Drive_Diag_BL();
            Locomotion_Drive_Diag_BL(Joystick_Map(min(joy_state.ljoy_x, joy_state.rjoy_x)));
        }
    }

    // Handle Camera Gimbal
    if(BTN_TR_PRESSED)
    {
        CameraGimbal_Decrement_Pan();
    }
    else if(BTN_TL_PRESSED)
    {
        CameraGimbal_Increment_Pan();
    }

    if(TRIGGER_RIGHT)
    {
        CameraGimbal_Increment_Tilt();
    }
    else if(TRIGGER_LEFT)
    {
        CameraGimbal_Decrement_Tilt();
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

uint8_t Joystick_Map_Generic(const int32_t &val, \
                             const int32_t &in_min, \
                             const int32_t &in_max, \
                             const int32_t &out_min, \
                             const int32_t &out_max)
{
    float temp = (float)abs(val); 
    float ratio = ((float)(out_max - out_min) / (float)(in_max - in_min));
    const uint8_t result = (uint8_t)((float)(temp - in_min) * ratio + out_min);
    return result;
}

void Joystick_Store_Control_State(GUI_Data *gd)
{
    uint32_t status = ((uint32_t)control_state.control.bits.turn_off | \
                       (uint32_t)(control_state.control.bits.drive_mode << 1));
    gd->set_loco_status((GUI_Data::Locomotion_Status)status);
}