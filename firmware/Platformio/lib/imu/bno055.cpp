#include "bno055.h"
#include "i2c_device.h"

// which master you pick will depend on the pins
// Master1 is defined in imx_rt1060_i2c_driver.cpp
I2CMaster & master1 = Master1; //pins 16 and 17 on teensy 4.1
I2CDevice bno = I2CDevice(master1, BNO_I2C_ADDRESS, _BIG_ENDIAN); //confirmed its big endian

void bno055::init()
{
    master1.begin(400000); //start the i2c master at 400khz
    
    // check to if comms up and chip is working
    checkIDs();

}

void bno055::checkIDs()
{
    uint8_t result = 0;
    if(bno.read(BNO_CHIP_ID_REG, &result, true)) //guessing the last param is for repeated start commands?
    {
        Serial.println(result == BNO_CHIP_ID ? "CHIP ID GOOD" : "ERROR ON CHIP ID");
    }
    else{
        Serial.println("[CHIP ID]: Failed to communicate");
    }

    if(bno.read(BNO_ACC_ID_REG, &result, true)) //guessing the last param is for repeated start commands?
    {
        Serial.println(result == BNO_ACC_ID ? "ACC ID GOOD" : "ERROR ON ACC ID");
    }
    else{
        Serial.println("[ACC ID]: Failed to communicate");
    }

    if(bno.read(BNO_MAG_ID_REG, &result, true)) //guessing the last param is for repeated start commands?
    {
        Serial.println(result == BNO_MAG_ID ? "MAG ID GOOD" : "ERROR ON MAG ID");
    }
    else{
        Serial.println("[MAG ID]: Failed to communicate");
    }

    if(bno.read(BNO_GYR_ID_REG, &result, true)) //guessing the last param is for repeated start commands?
    {
        Serial.println(result == BNO_GYR_ID ? "GYR ID GOOD" : "ERROR ON GYR ID");
    }
    else{
        Serial.println("[GYR ID]: Failed to communicate");
    }
}

void bno055::getTemp()
{
    
}