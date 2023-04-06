#include "HTS221.h"

static I2CDevice * hts;

static HTS221_Calib_t calib = {0};
static uint8_t hts_data[2] = {0};

void HTS221_Init(I2CDevice * dev)
{
    hts = dev;
    uint8_t result;

    // Try to talk to the HTS
    if(!HTS221_EstablishComms())
    {
        Serial.println("Failed to establish comms with HTS.");
        return;
    }

    // Turn on the device
    if(!hts->write(HTS221_CTRL_REG_1, (uint8_t)0x80, true))
    {
        Serial.println("Failed to write control reg 1");
        return;
    }

    // Boot
    if(!hts->write(HTS221_CTRL_REG_2, (uint8_t)0x80, true))
    {
        Serial.println("Failed to put device into boot");
    }

    Serial.println("Rebooting HTS221.");

    // Wait until device reboots
    while(1)
    {
        if(!hts->read(HTS221_CTRL_REG_2, &result, true))
        {
            delay(10);
            if((result & 0x80) == 0x80)
            {
                Serial.println("Device rebooted.");
                break;
            }
        }
    }

    // rewrite control reg 1
    if(!hts->write(HTS221_CTRL_REG_1, (uint8_t)0b10000100, true))
    {
        Serial.println("Failed to write control reg 1");
        return;
    }
    
    // write control reg 2. set heater to disable
    if(!hts->write(HTS221_CTRL_REG_2, (uint8_t)0b00000000, true))
    {
        Serial.println("Failed to write control reg 2");
        return;
    }

    // Set temp and humidity resolution
    // jk leave at default
    Serial.println("Leaving resolution at default.");

    // Get Calibration Values
    if(!HTS221_Calibrate())
    {
        Serial.println("Failed to calibrate HTS221");
        return;
    }

    // Trigger a reading
    if(!hts->write(HTS221_CTRL_REG_2, (uint8_t)0x01, true))
    {
        Serial.println("Failed to begin start for new dataset");
        return;
    }

    Serial.println("Successfully initialized HTS221");
}

bool HTS221_EstablishComms()
{
    uint8_t res = 0;
    if(!hts->read(HTS221_WHOAMI, &res, true))
    {
        return false;
    }

    return (res == HTS221_CHIP_ID);
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
    const uint16_t T0 = ((uint16_t)(hts_data[0]) | ((((uint16_t)(T0_T1_MSB)) & 0x03) << 8));
    const uint16_t T1 = ((uint16_t)(hts_data[1]) | ((((uint16_t)(T0_T1_MSB)) & 0x0C) << 6));
    
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
    calib.temp_slope = (((float)(T1 - T0)) / (((float)(T1_OUT - T0_OUT)) * 8.0f));
    calib.temp_base = (((float)T1) / 8.0f) - (calib.temp_slope*((float)T1_OUT));
    Serial.printf("temp slope: %f, temp b: %f", calib.temp_slope, calib.temp_base);
    return true;
}


bool HTS221_GetHumidCalib()
{
    // Note for multiple auto increment of register address, need MSB to 1
    // see .h file for the note

    uint8_t H0, H1;
    if(!hts->read(HTS221_H0_RH_X2, &H0, true) || !hts->read(HTS221_H1_RH_X2, &H1, true))
    {
        return false;
    }

    if(!hts->read(AUTOINCR(HTS221_H0_T0), hts_data, (size_t)2, true))
    {
        return false;
    }
    const int16_t H0_OUT = (int16_t)(hts_data[0]) | (((int16_t)(hts_data[1])) << 8);

    if(!hts->read(AUTOINCR(HTS221_H0_T1), hts_data, (size_t)2, true))
    {
        return false;
    }
    const int16_t H1_OUT = (int16_t)(hts_data[0]) | (((int16_t)(hts_data[1])) << 8);

    Serial.printf("h1: %u, h0: %u, h1_out: %d, h0_out %d", H1, H0, H1_OUT, H0_OUT);

    calib.humid_slope = (((float)(H1-H0)) / (((float)(H1_OUT - H0_OUT)) * 2.0f));
    calib.humid_base = (((float)H1) / 2.0f) - (calib.humid_slope*((float)H1_OUT));
    Serial.printf("humid slope: %f, humid b: %f", calib.humid_slope, calib.humid_base);
    return true;
}

void HTS221_ReadData(GUI_Data * gd)
{
    // check
    if(!HTS221_DataAvail())
    {
        Serial.println("HTS data is not available yet.");
        return;
    }

    if(!hts->read(AUTOINCR(HTS221_TEMP_OUT_L), hts_data, (size_t)2, true))
    {
        Serial.println("Failed to read TEMP");
        return;
    }

    const int16_t TEMP = (int16_t)(hts_data[0]) | (((int16_t)(hts_data[1])) << 8);
    #warning "you're gonna want to make this a float"
    gd->set_hts_temp(int32_t(C_TO_F(calib.temp_slope*((float)TEMP) + calib.temp_base) * 1000));

    if(!hts->read(AUTOINCR(HTS221_HUMIDITY_OUT), hts_data, (size_t)2, true))
    {
        Serial.println("Failed to read HUMID");
        return;
    }

    const int16_t HUMID = (int16_t)(hts_data[0]) | (((int16_t)(hts_data[1])) << 8);
    gd->set_hts_humidity((int32_t)((calib.humid_slope*HUMID + calib.humid_base) * 1000));

    // Inform sensor to get next reading
    if(!hts->write(HTS221_CTRL_REG_2, (uint8_t)0x01, true))
    {
        Serial.println("Failed to begin start for new dataset");
        return;
    }

}

bool HTS221_DataAvail()
{
    uint8_t data;
    if(!hts->read(HTS221_STATUS_REG, &data, true))
    {
        return false;
    }
    return ((data & 0x03) == 0x03);
}