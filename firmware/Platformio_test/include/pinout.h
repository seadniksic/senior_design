#ifndef PINOUT_H_
#define PINOUT_H_

//Notes regarding PWM https://www.pjrc.com/teensy/td_pulse.html
// Looks pins 2 and 3 are on the same time timer
// and so are 28 and 29
// all of them have a default frequency of 4.482 kHz

// GOAL IS HAVE ALL OF THEM BE ON THEIR OWN TIMERS IN CASE I NEED THAT

// BR = Back Right
#define BR_IN1_PIN 30 // MOVING TO PIN 20
#define BR_IN2_PIN 31 // MOVING TO PIN 21
#define BR_EN_PIN 2 //MOVING TO PIN 5 (updated on fritzing)

// BL = Back Left
#define BL_IN3_PIN 26
#define BL_IN4_PIN 27
#define BL_EN_PIN 3 // MOVING TO PIN 23 (updated on fritzing)

// FL = Front Left
#define FL_IN1_PIN 36 //MOVING TO PIN 41 (updated on fritzing)
#define FL_IN2_PIN 37 //MOVING TO PIN 32 (updated on fritzing)
#define FL_EN_PIN 28

// FR = Front Right
#define FR_IN3_PIN 34
#define FR_IN4_PIN 35 //MOVING TO PIN 40 (updated on fritizing)
#define FR_EN_PIN 29 //MOVING TO PIN 9 (updated on fritzing)




/* i2c buses */
#define I2C_SCL_GYRO_PIN 19
#define I2C_SDA_GYRO_PIN 18
#define I2C_SCL1_JETSON_PIN 24 //backup bus
#define I2C_SDA1_JETSON_PIN 25

/* LEDs */
//using the WS2812 library https://www.pjrc.com/non-blocking-ws2812-led-library/
// and https://github.com/PaulStoffregen/WS2812Serial
#define LED_STRIP_SERIAL8_PIN 35

/* ENCODERs  */

// For Teensy 4.1: pins 0-8, 30, 31, 33 and pin 37 are supported.
// WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and 
// are as such exclusive...pick one or the other. Same thing applies to pins
//  1 / 36 and 5 / 37.


#define BR_ENC_PHASEA_PIN 2
#define BR_ENC_PHASEB_PIN 3

#define BL_ENC_PHASEA_PIN 4
#define BL_ENC_PHASEB_PIN 6

#define FL_ENC_PHASEA_PIN 7
#define FL_ENC_PHASEB_PIN 8

#define FR_ENC_PHASEA_PIN 30
#define FR_ENC_PHASEB_PIN 31


/* CAMERA CONTROL */
// pins are on the same timer so the setfreq function will affect both
#define PAN_SERVO_PIN 36
#define TILT_SERVO_PIN 37


/* UART (comms with jetson) */
// https://www.pjrc.com/teensy/td_uart.html
#define UART3_TX_PIN 14
#define UART3_RX_PIN 15
#define UART3_RTS_PIN 33 // can be any
#define UART3_CTS_PIN 19
#define UART3_TX_EN_PIN  //can be any, not needed this is for enabling RS 485 transceiver chips


/* pins for battery monitoring / control */
#warning "complete these pins battery control"


/* pins for interacting with jetson? */
#warning "complete these pins for jetson"


/* Joystick pinout
overlap with these pin numbers dont matter right 
now because these won't be used on the final model

LEFT CONNECTOR
_________________________
| left  | up   |  LED    |
|_______|______|_________|  
| right | down |  FNC SW |
|_______|______|_________|

RIGHT CONNECTOR
_________________________
| LX    | LY   |  VCC    |
|_______|______|_________|  
| RX    | RY   |  GND    |
|_______|______|_________|

The RX and LY mappings correspond to the physical pin name, not the direction
that the joysticks will actually be moving IRL (since the joystick PCB is rotated)
The code will need to account for this and flip them
*/ 

#define DPAD_UP_PIN 25
#define DPAD_DOWN_PIN 11
#define DPAD_RIGHT_PIN 12
#define DPAD_LEFT_PIN 24
#define JOY_LX_PIN 
#define JOY_LY_PIN 
#define JOY_RX_PIN 
#define JOY_RY_PIN
#define JOY_LED_PIN 33
#define JOY_HOMEBTN_PIN 


#endif 