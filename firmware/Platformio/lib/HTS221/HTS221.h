#ifndef HTS221_H_
#define HTS221_H_

#include "i2c_device.h"
#include "MessageInterface.h"
#include "uart_messages.h"
#include <elapsedMillis.h>

// #defines taken from Adafruit, HTS221 Library, all credit goes to them
#define HTS221_I2CADDR 0x5F ///< HTS221 default i2c address

#define HTS221_WHOAMI 0x0F ///< Chip ID register
#define HTS221_CHIP_ID 0xBC         ///< HTS221 default device id from WHOAMI
#define HTS221_CTRL_REG_1 0x20      ///< First control regsiter; PD, OBDU, ODR
#define HTS221_CTRL_REG_2 0x21 ///< Second control regsiter; BOOT, Heater, ONE_SHOT
#define HTS221_CTRL_REG_3 0x22   ///< Third control regsiter; DRDY_H_L, DRDY
#define HTS221_STATUS_REG 0x27
#define HTS221_HUMIDITY_OUT 0x28 ///< Humidity output register (LSByte)
#define HTS221_TEMP_OUT_L 0x2A   ///< Temperature output register (LSByte)
#define HTS221_H0_RH_X2 0x30     ///< Humididy calibration LSB values
#define HTS221_H1_RH_X2 0x31     ///< Humididy calibration LSB values
#define HTS221_T0_DEGC_X8 0x32   ///< First byte of T0, T1 calibration values
#define HTS221_T1_T0_MSB 0x35    ///< Top 2 bits of T0 and T1 (each are 10 bits)
#define HTS221_H0_T0 0x36        ///< Humididy calibration Time 0 value
#define HTS221_H0_T1 0x3A        ///< Humididy calibration Time 1 value
#define HTS221_T0_OUT 0x3C       ///< T0_OUT LSByte
#define HTS221_T1_OUT 0x3E       ///< T1_OUT LSByte

#define HTS_BOOT_TIMEOUT 500

/*
The I²C embedded in the HTS221 behaves like a slave device and the following protocol
must be adhered to. After the start condition (ST) a slave address is sent, once a slave
acknowledge (SAK) has been returned, an 8-bit sub-address (SUB) will be transmitted: the
7 LSB represents the actual register address while the MSB enables address autoincrement.
If the MSB of the SUB field is ‘1’, the SUB (register address) will be automatically
increased to allow multiple data read/write
*/
#define AUTOINCR(reg) ((uint8_t)((reg) | 0x80))

#define C_TO_F(val) (((val)*9.0f / 5.0f) + 32)

typedef struct{
    float humid_slope;
    float humid_base;
    float temp_slope;
    float temp_base;
} HTS221_Calib_t;

bool HTS221_Init(I2CDevice * dev);
bool HTS221_EstablishComms();
bool HTS221_Calibrate();
bool HTS221_GetTempCalib();
bool HTS221_GetHumidCalib();
void HTS221_ReadData(GUI_Data * gd);
bool HTS221_DataAvail();

#endif 