#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <Arduino.h>

#include "pinout.h"

#define DRIVE_MODE_1 
/* 
Left joystick will control translational movements
    up = translate (right) forward
    right = translate right
    up-right = translate diagonally in the forward-right direction
    and so on, for a total of 8 driving directions. 
D-Pad will control rotational movements
    up = rotate clockwise
    down = rotate counter clockwise
    up-right = drive forward and turn right CHANGED TO JUST RIGHT (need to redo pwm stuff to implement)
    up-left = drive forward and turn left  CHANGED TO JUST LEFT (need to redo pwm stuff implement)
    down-right = drive backward and turn right OMITTED
    down-left = drive backward and turn left OMITTED
Right joystick for camera movements
    up = tilt camera upwards 
    down = tilt camera downwards
    right = pan camera right
    left = pan camera left
*/

// #define DRIVE_MODE_2
/* revised control
right joystick stays the same
left joystick i think 4 translational direction
and then someething else to switch it to the diag mode?
or just leave as is, yes. Can it be like 

fnc button is to recenter the camera

dpad left, rotate CCW
dpad right, rotate CW

dpad left and ljoy forward, go straight and turn left
dpad right and ljoy forward, go straight and turn right


*/

#if defined(DRIVE_MODE_1) && defined(DRIVE_MODE_2)
    #error "Both drive modes enabled."
#endif

#define INIT_DPAD
#define INIT_AUX //led and button
#define PRINT_DPAD
#define PRINT_AUX
#define PRINT_RJOY
#define PRINT_LJOY

#define JOY_DEADZONE 200 // in one direction
#define JOY_MIDDLE 511
#define JOY_MAX 1023
#define JOY_MIN 0
#define JOY_HIGH 711 //doing the math here instead of using #def for efficiency
#define JOY_LOW 311

#define STRAIGHT_DRIVE_MODE 0x00
#define DIAG_DRIVE_MODE 0x01

typedef struct joy_state_struct
{
    bool dpad_up; 
    bool dpad_down;
    bool dpad_down_prev;
    bool dpad_right;
    bool dpad_left;
    bool fnc_btn; 
    bool fnc_btn_prev;
    uint16_t ljoy_x; //x represents natural direction, not the pin its attached to
    uint16_t ljoy_y;
    uint16_t rjoy_x;
    uint16_t rjoy_y;
    uint8_t drive_mode;
} joy_state_t;

extern joy_state_t joy_state;



namespace joystick {

    void init();
    void print(); //serial port must be initialized before using
    void run(); 
    void store_joy_state();


    /* digital inputs */

    #pragma message "replace all these with macros and then u can make the joy_state struct static"

    inline bool dpad_up_pressed()
    {
        return digitalRead(DPAD_UP_PIN);
    }

    inline bool dpad_down_pressed()
    {
        return digitalRead(DPAD_DOWN_PIN);
    }

    inline bool dpad_right_pressed()
    {
        return digitalRead(DPAD_RIGHT_PIN);
    }

    inline bool dpad_left_pressed()
    {
        return digitalRead(DPAD_LEFT_PIN);
    }

    inline bool fncbtn_pressed()
    {
        return digitalRead(JOY_FNCBTN_PIN);
    }

    /* analog inputs*/

    inline bool ljoy_right()
    {
        return (joy_state.ljoy_x < JOY_LOW); 
    }

    inline bool ljoy_left()
    {
        return (joy_state.ljoy_x > JOY_HIGH); 
    }

    inline bool ljoy_up()
    {
        return (joy_state.ljoy_y < JOY_LOW); 
    }

    inline bool ljoy_down()
    {
        return (joy_state.ljoy_y > JOY_HIGH); 
    }

    inline bool rjoy_right()
    {
        return (joy_state.rjoy_x < JOY_LOW); 
    }

    inline bool rjoy_left()
    {
        return (joy_state.rjoy_x > JOY_HIGH); 
    }

    inline bool rjoy_up()
    {
        return (joy_state.rjoy_y < JOY_LOW); 
    }

    inline bool rjoy_down()
    {
        return (joy_state.rjoy_y > JOY_HIGH); 
    }

    inline bool ljoy_y_deadzone()
    {
        return (joy_state.ljoy_y < JOY_HIGH && joy_state.ljoy_y > JOY_LOW); 
    }

    inline bool ljoy_x_deadzone()
    {
        return (joy_state.ljoy_x < JOY_HIGH && joy_state.ljoy_x > JOY_LOW); 
    }

    inline bool ljoy_deadzone()
    {
        return ljoy_x_deadzone() && ljoy_y_deadzone();
    }

}






#endif