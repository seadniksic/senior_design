#ifndef CONFIG_H_
#define CONFIG_H_

#define USE_TEST_ROVER_PINS 0
#define USE_PRODUCTION_PINS 1

#if USE_TEST_ROVER_PINS && USE_PRODUCTION_PINS
    #error "CANNOT USE BOTH PIN LAYOUTS"
#endif 

#if USE_TEST_ROVER_PINS
    #warning "!!!!!!USING TEST ROVER PINS!!!!!!"
#endif

#if USE_PRODUCTION_PINS
    #warning "!!!!!!USING PRODUCTION PINS!!!!!!"
#endif

// Serial
#define SERIAL_BAUD 500000
#define SERIAL_COMMS_RECEIVE_TIMEOUT 350

// TASK SCHEDULER (time in ms)
#define TS_HEARTBEAT 500 // 2 Hz
#define TS_PRINT_1 100 // 10 Hz
#define TS_PRINT_2
#define TS_PRINT_3
#define TS_JOY_UPDATE 50 // 20Hz
#define TS_JOY_COMMS 25 // 40Hz
#define TS_SLAM_COMMS 10 // 100Hz
#define TS_GUI_DATA 200 // 5 Hz

#endif