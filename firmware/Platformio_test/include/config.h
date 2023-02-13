#ifndef CONFIG_H_
#define CONFIG_H_

#define USE_TEST_ROVER_PINS 1
#define USE_PRODUCTION_PINS 0

#if USE_TEST_ROVER_PINS && USE_PRODUCTION_PINS
    #error "CANNOT USE BOTH PIN LAYOUTS"
#endif

#endif