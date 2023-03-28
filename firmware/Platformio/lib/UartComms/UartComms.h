#ifndef UARTCOMMS_H_
#define UARTCOMMS_H_

#include <UartReadBuffer.h>
#include <UartWriteBuffer.h>
#include <MessageInterface.h>
#include "uart_messages.h"
#include <Arduino.h>
#include <elapsedMillis.h>


#define HWSERIAL Serial2 // pins 7 and 8
#define HWSERIAL_BAUD 115200
#define SYNC_BYTE 0x64 // read
#define SYNC_BYTE_WRITE 0x46
#define TIMEOUT_DATA 20 //ms
#define TIMEOUT_NUMBYTES 10

typedef struct {
    UartReadBuffer *read_buffer;
    UartWriteBuffer *write_buffer;
    Joystick_Input *js_in;
    GUI_Data *gui_data;
    IMU_Data *imu_data;
    elapsedMillis rcv_clock;
    elapsedMillis time_since_last_serialize;
    bool commsNeedReset;
} UartComms_t;

// #define PRINT_MSG 


void UartComms_Init();
void UartComms_RcvControls();
void UartComms_PopulateGUIReply(const float &cpu_temp);
void UartComms_SendGUIData();
void UartComms_ClearWriteBuffer();
void UartComms_ClearReadBuffer();
void UartComms_ClearBuffers();
elapsedMillis* UartComms_GetTimeSinceLastRead();
void UartComms_ClearJoystick();
Joystick_Input* UartComms_GetJoystick();

#endif