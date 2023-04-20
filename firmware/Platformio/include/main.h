#ifndef MAIN_H_
#define MAIN_H_

#include "pinout.h"

#define CPUTemp_Init() InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES)

#define BATT_TEMP_R0 10000.0f
#define BATT_TEMP_B 3350.0f // bias provided by manufacturer
#define BATT_TEMP_T0 25.0f

typedef struct
{
    bool bno_init;
    bool hts_init;
    uint32_t i2c_msg_failed;
    uint32_t i2c_msg_passed;
    uint32_t uart_msg_failed;
    uint32_t uart_msg_passed;
    uint32_t lost_sync_byte;
    uint32_t resetting_comms;
    uint32_t uptime; 
    uint32_t looptime;
    uint32_t max_looptime;
} The_Watcher_t; 

extern The_Watcher_t g_watcher;



inline uint32_t Battery_ReadTemp()
{
    float temp = 0;
    uint32_t curr_val = (uint32_t)analogRead(BMS_CHIP_TEMP_PIN);

    float R = 10000.0f / (( 1023.0f / ((float)curr_val)) - 1.0f);
    
    temp = 1000.0f / ((1.0f / BATT_TEMP_T0) +  1.0f/BATT_TEMP_B*(log(R / BATT_TEMP_R0)));

    return (uint32_t)temp;
}


#endif