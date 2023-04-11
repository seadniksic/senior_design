#ifndef MAIN_H_
#define MAIN_H_

#define CPUTemp_Init() InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES)

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
} The_Watcher_t; 

extern The_Watcher_t g_watcher;

#endif