#include "HTS221.h"

static I2CDevice * hts;

static HTS221_Calib_t calib = {0};
static uint8_t hts_data[2] = {0};

void HTS221_Init(I2CDevice * dev)
{
    hts = dev;

    // Try to talk to the HTS
    if(!HTS221_EstablishComms())
    {
        Serial.println("Failed to establish comms with HTS.");
        return;
    }

    // Calibrate


}

bool HTS221_EstablishComms()
{
    uint8_t res = 0;
    if(!hts->read(HTS221_WHOAMI, &res, true))
    {
        return false;
    }
    else
    {
        return (res == HTS221_CHIP_ID);
    }
}

bool HTS221_Calibrate()
{
    if(!HTS221_GetTempCalib())
    {
        Serial.println("Failed to get temperature calibration.");
        return false;
    }

    if(!HTS221_GetHumidCalib())
    {
        Serial.println("Failed to get humidity calibration");
    }

    Serial.println("Sucessfully fetched calibration values.");
    return true;

}

bool HTS221_GetTempCalib()
{
    // #define HTS221_H0_RH_X2 0x30     ///< Humididy calibration LSB values
    // #define HTS221_H1_RH_X2 0x31     ///< Humididy calibration LSB values
    // #define HTS221_T0_DEGC_X8 0x32   ///< First byte of T0, T1 calibration values
    // #define HTS221_T1_T0_MSB 0x35    ///< Top 2 bits of T0 and T1 (each are 10 bits)
    // #define HTS221_H0_T0 0x36        ///< Humididy calibration Time 0 value
    // #define HTS221_H0_T1 0x3A        ///< Humididy calibration Time 1 value
    // #define HTS221_T0_OUT 0x3C       ///< T0_OUT LSByte
    // #define HTS221_T1_OUT 0x3E       ///< T1_OUT LSByte

    // Note for multiple auto increment of register address, need MSB to 1
    // see .h file for the note

    uint8_t T0_T1_MSB = 0;
    if(!hts->read(AUTOINCR(HTS221_T0_DEGC_X8), hts_data, (size_t)2, true))
    {
        return false;
    }

    if(!hts->read(HTS221_T1_T0_MSB, &T0_T1_MSB, true))
    {
        return false;
    }

    // T0 is 10 bits, the MSB 2 bits are bit 0 and 1
    const uint16_t T0 = ((uint16_t)(hts_data[0]) | ((((uint16_t)(T0_T1_MSB)) & 0x3) << 8));
    const uint16_t T1 = ((uint16_t)(hts_data[1]) | ((((uint16_t)(T0_T1_MSB)) & 0xC) << 6));
    
    if(!hts->read(AUTOINCR(HTS221_T0_OUT), hts_data, (size_t)2,true))
    {
        return false;
    }
    const int16_t T0_OUT = (int16_t)(hts_data[0]) | (((int16_t)(hts_data[1])) << 8);

    if(!hts->read(AUTOINCR(HTS221_T1_OUT), hts_data, (size_t)2,true))
    {
        return false;
    }
    const int16_t T1_OUT = (int16_t)(hts_data[0]) | (((int16_t)(hts_data[1])) << 8);

    Serial.printf("t1: %u, t0: %u, t1_out: %d, t0_out %d", T1, T0, T1_OUT, T0_OUT);

    // see page 27 in datasheet, equations of lines, rise/over. y=mx+b => b=y-mx
    calib.temp_slope = (float)((T1 - T0) / ((T1_OUT - T0_OUT) * 8));
    calib.temp_base = ((float)T1) - (calib.temp_slope*((float)T1_OUT));
    Serial.printf("temp slope: %f, temp b: %f", calib.temp_slope, calib.temp_base);
    return true;
}


bool HTS221_GetHumidCalib()
{
    // #define HTS221_H0_RH_X2 0x30     ///< Humididy calibration LSB values
    // #define HTS221_H1_RH_X2 0x31     ///< Humididy calibration LSB values
    // #define HTS221_T0_DEGC_X8 0x32   ///< First byte of T0, T1 calibration values
    // #define HTS221_T1_T0_MSB 0x35    ///< Top 2 bits of T0 and T1 (each are 10 bits)
    // #define HTS221_H0_T0 0x36        ///< Humididy calibration Time 0 value
    // #define HTS221_H0_T1 0x3A        ///< Humididy calibration Time 1 value
    // #define HTS221_T0_OUT 0x3C       ///< T0_OUT LSByte
    // #define HTS221_T1_OUT 0x3E       ///< T1_OUT LSByte

    // Note for multiple auto increment of register address, need MSB to 1
    // see .h file for the note

    uint8_t H0, H1;



    return true;

}