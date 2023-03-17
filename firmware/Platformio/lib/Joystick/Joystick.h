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

#define STRAIGHT_DRIVE_MODE 0
#define DIAG_DRIVE_MODE 1

/* digital inputs */
#define BTN_A_PRESSED (joy_state.buttons.button_bits.BTN_A)
#define BTN_B_PRESSED (joy_state.buttons.button_bits.BTN_B)
#define BTN_X_PRESSED (joy_state.buttons.button_bits.BTN_X)
#define BTN_Y_PRESSED (joy_state.buttons.button_bits.BTN_Y)
#define BTN_START_PRESSED (joy_state.buttons.button_bits.BTN_START)
#define BTN_SELECT_PRESSED (joy_state.buttons.button_bits.BTN_SELECT)
#define DPAD_UP_PRESSED (joy_state.buttons.button_bits.DPAD_UP)
#define DPAD_DOWN_PRESSED (joy_state.buttons.button_bits.DPAD_DOWN)
#define DPAD_LEFT_PRESSED (joy_state.buttons.button_bits.DPAD_LEFT)
#define DPAD_RIGHT_PRESSED (joy_state.buttons.button_bits.DPAD_RIGHT)
#define BTN_THUMBR_PRESSED (joy_state.buttons.button_bits.BTN_THUMBR)
#define BTN_THUMBL_PRESSED (joy_state.buttons.button_bits.BTN_THUMBL)
#define BTN_TR_PRESSED (joy_state.buttons.button_bits.BTN_TR)
#define BTN_TL_PRESSED (joy_state.buttons.button_bits.BTN_TL)
#define BTN_NONE_PRESSED (joy_state.buttons.button_state == 0)

/* analog input ranges*/
#define JOY_DEADZONE 3000 // in one direction
#define JOY_MIDDLE 511
#define JOY_MAX 32768 // down and right are positive
#define JOY_MIN (-JOY_MAX)
#define JOY_HIGH JOY_DEADZONE
#define JOY_LOW (-JOY_DEADZONE)
#define JOY_TRIG_MAX 255
#define JOY_TRIG_MIN 0
#define JOY_TRIG_PRESSED_LIM 150

/* analog input*/
#define LJOY_RIGHT (joy_state.ljoy_x > JOY_HIGH)
#define LJOY_LEFT (joy_state.ljoy_x < JOY_LOW)
#define LJOY_UP (joy_state.ljoy_y < JOW_LOW)
#define LJOY_DOWN (joy_state.ljoy_y > JOY_HIGH)
#define LJOY_DEADZONE_X (joy_state.ljoy_x > JOY_LOW && joy_state.ljoy_x < JOY_HIGH)
#define LJOY_DEADZONE_Y (joy_state.ljoy_y > JOY_LOW && joy_state.ljoy_y < JOY_HIGH)
#define LJOY_DEADZONE_XY (LJOY_DEADZONE_X && LJOY_DEADZONE_Y)
#define LJOY_UR (LJOY_UP && LJOY_RIGHT) 
#define LJOY_UL (LJOY_UP && LJOY_LEFT)
#define LJOY_DR (LJOY_DOWN && LJOY_RIGHT)
#define LJOY_DL (LJOY_DOWN && LJOY_LEFT)
#define RJOY_RIGHT (joy_state.rjoy_x > JOY_HIGH)
#define RJOY_LEFT (joy_state.rjoy_x < JOY_LOW)
#define RJOY_UP (joy_state.rjoy_y < JOW_LOW)
#define RJOY_DOWN (joy_state.rjoy_y > JOY_HIGH)
#define RJOY_DEADZONE_X (joy_state.rjoy_x > JOY_LOW && joy_state.rjoy_x < JOY_HIGH)
#define RJOY_DEADZONE_Y (joy_state.rjoy_y > JOY_LOW && joy_state.rjoy_y < JOY_HIGH)
#define RJOY_DEADZONE_XY (RJOY_DEADZONE_X && RJOY_DEADZONE_Y)
#define RJOY_UR (RJOY_UP && RJOY_RIGHT) 
#define RJOY_UL (RJOY_UP && RJOY_REFT)
#define RJOY_DR (RJOY_DOWN && RJOY_RIGHT)
#define RJOY_DL (RJOY_DOWN && RJOY_REFT)
#define TRIGGER_LEFT (joy_state.tl > JOY_TRIG_PRESSED_LIM)
#define TRIGGER_RIGHT (joy_state.tr > JOY_TRIG_PRESSED_LIM)
#define TRIGGER_LEFT_DEADZONE (joy_state.tl <= JOY_TRIG_PRESSED_LIM)
#define TRIGGER_RIGHT_DEADZONE (joy_state.tr <= JOY_TRIG_PRESSED_LIM)



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


void Joystick_Init();
void Joystick_Print(); //serial port must be initialized before using
void Joystick_Run(); 
void Joystick_Store_State(Joystick_Input &js_in);
bool Joystick_Input_Present();


#endif