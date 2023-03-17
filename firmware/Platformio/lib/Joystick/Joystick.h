#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>
#include <MessageInterface.h>
#include "uart_messages.h"

/* 
/// JOYSTICK CONTROLS ///
LJOY (TRANSLATION MOVEMENTS):
- Forward, Back, Left, Right
- Button pressed (TBD) switch to diag drive mode which will be 
    forward-left, forward-right, back-right, back-left
- Button A will be used to switch driving modes.
RJOY (TURNING): 
- Right -> CW Turning
- Left > CCW Turning
- These will be combined with Forward and Back movements on LJOY,
    mapped porportionally. This will allow to drive while slightly turning
    like a normal car.
BUMPERS (PAN):
- RB -> Pan Right
- LB -> Pan Left
TRIGGERS (TILT):
- RT -> Tilt Up
- LT -> Tilt Down
*/

#define JOY_DEADZONE 200 // in one direction
#define JOY_MIDDLE 511
#define JOY_MAX 1023
#define JOY_MIN 0
#define JOY_HIGH 711 //doing the math here instead of using #def for efficiency
#define JOY_LOW 311

#define STRAIGHT_DRIVE_MODE 0x00
#define DIAG_DRIVE_MODE 0x01

// https://stackoverflow.com/questions/3497345/is-there-a-way-to-access-individual-bits-with-a-union
typedef struct button_positions
{
    unsigned BTN_A : 1;
    unsigned BTN_B : 1;
    unsigned BTN_X : 1;
    unsigned BTN_Y : 1;
    unsigned BTN_START : 1;
    unsigned BTN_SELECT : 1;
    unsigned DPAD_UP : 1;
    unsigned DPAD_DOWN : 1;
    unsigned DPAD_LEFT : 1;
    unsigned DPAD_RIGHT : 1;
    unsigned BTN_THUMBR : 1;
    unsigned BTN_THUMBL : 1;
    unsigned BTN_TR : 1;
    unsigned BTN_TL : 1;
} buttons_t;

typedef union {
    uint32_t button_state;
    buttons_t button_bits;
} button_u;

typedef struct joy_state_struct
{
    button_u buttons;
    int32_t ljoy_x;
    int32_t ljoy_y;
    int32_t rjoy_x;
    int32_t rjoy_y;
    int32_t tr;
    int32_t tl;
} joy_state_t;

extern joy_state_t joy_state;


namespace Joystick {

    void init();
    void print(); //serial port must be initialized before using
    void run(); 
    void store_joy_state(Joystick_Input &js_in);


    /* digital inputs */

    #if 0

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

    #endif

}






#endif