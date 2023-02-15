#include "bno055.h"
#include "i2c_device.h"

// which master you pick will depend on the pins
// Master1 is defined in imx_rt1060_i2c_driver.cpp
I2CMaster & master1 = Master1; //pins 16 and 17 on teensy 4.1
I2CDevice bno = I2CDevice(master1, BNO_I2C_ADDRESS, _BIG_ENDIAN); //confirmed its big endian

#pragma message "im curious how the library does it? is it writing direct to reg or reading then writing back with a mask?"

void bno055::init()
{
    master1.begin(400000); //start the i2c master at 400khz
    
    // check to if comms up and chip is working

    getsysstatus();

    checkIDs();

    getsysstatus();

    //config units
    // set temp to be F

    uint8_t result = 0;
    bno.read(BNO_UNIT_SEL, &result, true);
    Serial.printf("result is %X\n", result);

    if(bno.write(BNO_UNIT_SEL, (uint8_t)(result | 0b10000), true))
    {
        Serial.println("wrote config units");
        delay(1000);
    }

    getsysstatus();


    if(bno.write(BNO_OPR_MODE, (uint8_t)BNO_OPR_MODE_M4G, true))
    {
        Serial.println("wrote opr mode");
        delay(1000);
    }

    //for some reason the units arent being switched over to F even tho its reading
    //back the correct value of the reg.
    bno.read(BNO_UNIT_SEL, &result, true);
    Serial.printf("result is %X\n", result);
    // now it works! just needed to power cycle.
    // can read in both C and F





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
    // if(bno.write(BNO_TEMP_SOURCE_REG, (uint8_t)BNO_TEMP_SRC_ACC, true))
    // {
    //     Serial.println("sucessfulyl wrote");
    //     uint8_t result = 0;
    //     bno.read(BNO_TEMP_REG, &result, true);
    //     Serial.printf("temperature is %X\n", result);

    // }
    // else{
    //     Serial.println("failed to write");
    // }

    uint8_t result = 0;
    bno.read(BNO_TEMP_REG, &result, true);
    Serial.printf("temperature is %X\n", result);


}

void bno055::getsysstatus()
{
    uint8_t result= 0;
    delay(200);
    bno.read(BNO_SYS_STATUS_REG, &result, true);
    Serial.printf("sys status reg is %X\n", result);
    
    delay(50);
    bno.read(BNO_SYS_ERR, &result, true);
    Serial.printf("sys error reg is %X\n", result);
    delay(1000);
}