#include "bno055.h"
#include "i2c_device.h"

// which master you pick will depend on the pins
// Master1 is defined in imx_rt1060_i2c_driver.cpp
I2CMaster & master1 = Master1; //pins 16 and 17 on teensy 4.1
I2CDevice bno = I2CDevice(master1, BNO_I2C_ADDRESS, _BIG_ENDIAN); //confirmed its big endian

#pragma message "im curious how the library does it? is it writing direct to reg or reading then writing back with a mask?"

static const uint8_t mask8[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void bno055::init()
{

    uint8_t result = 0;

    master1.begin(400000); //start the i2c master at 400khz
    
    // check to if comms up and chip is working
    checkIDs();

    //check the current operating mode
    bno.read(BNO_OPR_MODE, &result, true);
    while((result & 0x0F) != BNO_OPR_MODE_CONFIG)
    {
        Serial.println("IMU is not in config mode. returning to config");
        result = (result & 0xF0) | BNO_OPR_MODE_CONFIG; 
        bno.write(BNO_OPR_MODE, result, true);
        delay(20); // needs 20 seconds to get to config mode. 
    }

    // power on self test (section 3.8)
    // the self test checks the acc, mag, and gyr ids so the original check for if comms is working can just be check chip id
    bno.read(BNO_ST_RESULT, &result, true);
    if((result & 0x0F) == 0xF)
    {
        Serial.println("Power on Self Test Passed");
    }
    else
    {
        for(uint8_t i = 0; i < 4; i++)
        {
            if(!(result & mask8[i]))
            {
                Serial.printf("bit %u failed.\n", i);
            }
        }
    }


    //config units
    // set temp to be F
    bno.read(BNO_UNIT_SEL, &result, true);
    Serial.printf("result is %X (reading unit_sel reg)\n", result);

    if(bno.write(BNO_UNIT_SEL, (uint8_t)(result | 0b10000), true))
    {
        Serial.println("wrote config units");
    }

    // read again to confirm selection
    bno.read(BNO_UNIT_SEL, &result, true);
    Serial.printf("result is %X (reading unit_sel reg after writing to it)\n", result);


    getsysstatus();

    // set operating mode
    if(bno.write(BNO_OPR_MODE, (uint8_t)BNO_OPR_MODE_M4G, true))
    {
        Serial.println("wrote opr mode to M4G");
    }

    // Sensor calibration
    Serial.println("waiting for calibration...");
    do
    {
        Serial.printf("attempting to calibrature: result = %u\n", result);
        bno.read(BNO_CALIB_STAT, &result, true);
        delay(50);
    }while((result & 0xC0) == 0xC0);

    Serial.println("finished calibrating");


    Serial.println("COMPLETED INIT");

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
    Serial.printf("temperature is %u\n", result);


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

void bno055::getEULypr()
{
    uint8_t ypr_data[6] = {0}; //in that order exactly, yaw is array 0 and 1
    // register is automatically incremented when reading multiple bytes
    bno.read(BNO_EUL_HEADING_LSB, ypr_data, (size_t)6, true);
    // 1 degree = 16 LSB, so need to divide by 16 which is same as right shifting 4
    int16_t heading = (int16_t)((ypr_data[1] << 8 | ypr_data[0]) >> 4);
    int16_t pitch = (int16_t)((ypr_data[3] << 8 | ypr_data[2]) >> 4);
    int16_t roll = (int16_t)((ypr_data[5] << 8 | ypr_data[4]) >> 4);

    // heading = heading -180;
    // pitch = map(pitch, 0, 4096, -180, 180 );
    // roll = map(pitch, 0, 4096, -180, 180 );
    
    heading = ((heading +180) % 360) - 140;
    if(pitch > 180)
    {
        pitch = pitch - 4096;
    }
    if(roll > 180)
    {
        roll = roll -4096;
    }




    Serial.printf("Y:%d \tP:%d \tR:%d\n", heading, pitch, roll);

}

