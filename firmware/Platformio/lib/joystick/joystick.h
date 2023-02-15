#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <Arduino.h>

#include "pinout.h"

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


typedef struct joy_state_struct
{
    bool dpad_up; 
    bool dpad_down;
    bool dpad_right;
    bool dpad_left;
    bool fnc_btn;
    uint16_t ljoy_x; //x represents natural direction, not the pin its attached to
    uint16_t ljoy_y;
    uint16_t rjoy_x;
    uint16_t rjoy_y;
} joy_state_t;


namespace joystick {

    void init();
    void print(); //serial port must be initialized before using
    void run(); 
    void store_joy_state();


    /* digital inputs */

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
        return (analogRead(JOY_LY_PIN) < JOY_LOW); //joystick is rotated so swap pin
    }

    inline bool ljoy_left()
    {
        return (analogRead(JOY_LY_PIN) > JOY_HIGH); //joystick is rotated so swap pin
    }

    inline bool ljoy_up()
    {
        return (analogRead(JOY_LX_PIN) < JOY_LOW); //joystick is rotated so swap pin
    }

    inline bool ljoy_down()
    {
        return (analogRead(JOY_LX_PIN) > JOY_HIGH); //joystick is rotated so swap pin
    }

    inline bool rjoy_right()
    {
        return (analogRead(JOY_RY_PIN) < JOY_LOW); //joystick is rotated so swap pin
    }

    inline bool rjoy_left()
    {
        return (analogRead(JOY_RY_PIN) > JOY_HIGH); //joystick is rotated so swap pin
    }

    inline bool rjoy_up()
    {
        return (analogRead(JOY_RX_PIN) < JOY_LOW); //joystick is rotated so swap pin
    }

    inline bool rjoy_down()
    {
        return (analogRead(JOY_RX_PIN) > JOY_HIGH); //joystick is rotated so swap pin
    }

    inline bool ljoy_y_deadzone()
    {
        return (analogRead(JOY_LX_PIN) < JOY_HIGH && analogRead(JOY_LX_PIN) > JOY_LOW); //joystick is rotated so swap pin
    }

    inline bool ljoy_x_deadzone()
    {
        return (analogRead(JOY_LY_PIN) < JOY_HIGH && analogRead(JOY_LY_PIN) > JOY_LOW); //joystick is rotated so swap pin
    }

    inline bool ljoy_deadzone()
    {
        return ljoy_x_deadzone() && ljoy_y_deadzone();
    }

}






#endif