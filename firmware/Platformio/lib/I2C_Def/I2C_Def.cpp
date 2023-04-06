#include "I2C_Def.h"
#include "bno055.h"
#include "HTS221.h"

// I2C Definitions
// Master1 is defined in imx_rt1060_i2c_driver.cpp
static I2CMaster & g_master = Master1; //pins 16 and 17 on teensy 4.1
static I2CDevice hts = I2CDevice(g_master, HTS221_I2CADDR, _LITTLE_ENDIAN);
static I2CDevice bno = I2CDevice(g_master, BNO_I2C_ADDRESS, _LITTLE_ENDIAN);

void I2C_Def_Init()
{
    g_master.begin(I2C_SPEED);
}

I2CDevice * I2C_Def_GetBnoDev()
{
    return &hts;
}

I2CDevice * I2C_Def_GetHtsDev()
{
    return &bno;
}