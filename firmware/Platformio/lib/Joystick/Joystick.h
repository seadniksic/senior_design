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
CONTROL BUTTONS: 
- A to be used for switching driving modes (toggle)
- X to be used for turning all axis off (toggle)
- B to be used for resetting cameras to mid position
*/

/* digital inputs */
#define BTN_A_PRESSED (joy_state.buttons.bits.BTN_A)
#define BTN_B_PRESSED (joy_state.buttons.bits.BTN_B)
#define BTN_X_PRESSED (joy_state.buttons.bits.BTN_X)
#define BTN_Y_PRESSED (joy_state.buttons.bits.BTN_Y)
#define BTN_START_PRESSED (joy_state.buttons.bits.BTN_START)
#define BTN_SELECT_PRESSED (joy_state.buttons.bits.BTN_SELECT)
#define DPAD_UP_PRESSED (joy_state.buttons.bits.DPAD_UP)
#define DPAD_DOWN_PRESSED (joy_state.buttons.bits.DPAD_DOWN)
#define DPAD_LEFT_PRESSED (joy_state.buttons.bits.DPAD_LEFT)
#define DPAD_RIGHT_PRESSED (joy_state.buttons.bits.DPAD_RIGHT)
#define BTN_THUMBR_PRESSED (joy_state.buttons.bits.BTN_THUMBR)
#define BTN_THUMBL_PRESSED (joy_state.buttons.bits.BTN_THUMBL)
#define BTN_TR_PRESSED (joy_state.buttons.bits.BTN_TR)
#define BTN_TL_PRESSED (joy_state.buttons.bits.BTN_TL)
#define BTN_NONE_PRESSED (joy_state.buttons.state == 0)

/* analog input ranges*/
#define JOY_DEADZONE 10000 // in one direction
#define JOY_MIDDLE 0
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
#define LJOY_UP (joy_state.ljoy_y < JOY_LOW)
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
#define RJOY_UP (joy_state.rjoy_y < JOY_LOW)
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

/* Control */
#define STRAIGHT_DRIVE_MODE 0
#define DIAG_DRIVE_MODE 1
#define IN_STRAIGHT_DRIVE_MODE (control_state.control.bits.drive_mode == STRAIGHT_DRIVE_MODE)
#define IN_DIAG_DRIVE_MODE (control_state.control.bits.drive_mode == DIAG_DRIVE_MODE)
#define POWER_DOWN_MODE 1
#define RUNNING_MODE 0 
#define IN_POWER_DOWN_MODE (control_state.control.bits.turn_off == POWER_DOWN_MODE)
#define IN_RUNNING_MODE (control_state.control.bits.turn_off == RUNNING_MODE)
#define CENTER_CAMS (control_state.control.bits.center_cams)

/* Mapping */
#define IN_MIN JOY_HIGH
#define IN_MAX JOY_MAX
#define OUT_MIN 100
#define OUT_MAX 255
#define RATIO ((float)(OUT_MAX - OUT_MIN) / (float)(IN_MAX - IN_MIN))


// https://stackoverflow.com/questions/3497345/is-there-a-way-to-access-individual-bits-with-a-union
typedef struct 
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

typedef union 
{
    uint32_t state;
    buttons_t bits;
} button_u;

typedef struct 
{
    unsigned drive_mode : 1;
    unsigned turn_off : 1;
    unsigned center_cams : 1;
} control_t;

typedef union
{
    uint8_t state;
    control_t bits;
} control_u;

typedef struct
{
    button_u prev_buttons;
    button_u buttons;
    int32_t ljoy_x;
    int32_t ljoy_y;
    int32_t rjoy_x;
    int32_t rjoy_y;
    int32_t tr;
    int32_t tl;
} joy_state_t;

typedef struct
{
    control_u control;
    uint16_t pan_pos;
    uint16_t tilt_pos;
} control_state_t;



void Joystick_Init();
void Joystick_Print(); //serial port must be initialized before using
void Joystick_Run(); 
void Joystick_Store_State(Joystick_Input &js_in);
bool Joystick_Input_Present();
uint8_t Joystick_Map(const int32_t &val);
uint8_t Joystick_Map_Generic(const int32_t &val, \
                             const int32_t &in_min, \
                             const int32_t &in_max, \
                             const int32_t &out_min, \
                             const int32_t &out_max);

#endif