#include "bno055.h"
#include "i2c_device.h"
#include "util.h"

// which master you pick will depend on the pins
// Master1 is defined in imx_rt1060_i2c_driver.cpp
I2CMaster & master1 = Master1; //pins 16 and 17 on teensy 4.1
I2CDevice bno = I2CDevice(master1, BNO_I2C_ADDRESS, _BIG_ENDIAN); //confirmed its big endian

bool bno055::init()
{
    master1.begin(400000); //start the i2c master at 400khz
    
    // check to if comms up and chip is working
    if(!establish_comms())
    {
        return false;
    }

    Serial.println("[BNO055]: Resetting the sensor...");
    if(!reset_sys())
    {
        return false;
    }

    // change to config mode (although it should already be in config mode because of the reset)
    set_mode(OPR_MODE_CONFIG);

    // perform self test
    if(!self_test())
    {
        return false;
    }

    // change temp output to be F
    set_units(TEMP_UNIT_F);    

    // set to page 0
    // bno.write(BNO_PAGE_ID_REG, (uint8_t)0x00, true);

    // check state of the system at this point, if all is good proceed.
    if(!get_sys_status())
    {
        return false;
    }

    // Set clock to external
    if(!set_clock_source(BNO_CLK_SRC_EXTERNAL))
    {
        return false;
    }

    // set out of config mode
    opr_mode_e desired_mode = OPR_MODE_NDOF;

    set_mode(desired_mode);
    while(get_mode() == OPR_MODE_CONFIG)
    {
        Serial.println("[BNO055]: Failed to set mode, retrying..");
        set_mode(desired_mode);
    }

    // Sensor calibration
    calibrate(desired_mode);

    // check state of the system at this point, if all is good proceed.
    if(!get_sys_status())
    {
        return false;
    }

    Serial.println("[BNO055]: COMPLETED INIT");
    return true;
}

void bno055::calibrate(uint8_t mode)
{
    uint8_t result = 0, target_calib = BNO_SYS_CALIBRATED;

    switch(mode)
    {
        case OPR_MODE_ACCONLY:
            target_calib |= BNO_ACC_CALIBRATED;
            break;
        case OPR_MODE_MAGONLY:
            target_calib |= BNO_MAG_CALIBRATED;
            break;
        case OPR_MODE_GYROONLY:
        case OPR_MODE_M4G:
            target_calib |= BNO_GYR_CALIBRATED;
            break;
        case OPR_MODE_ACCMAG:
        case OPR_MODE_COMPASS:
            target_calib |= BNO_ACC_CALIBRATED | BNO_MAG_CALIBRATED;
            break;
        case OPR_MODE_ACCGYRO:
        case OPR_MODE_IMU:
            target_calib |= BNO_ACC_CALIBRATED | BNO_GYR_CALIBRATED;
            break;
        case OPR_MODE_MAGGYRO:
            target_calib |= BNO_MAG_CALIBRATED | BNO_GYR_CALIBRATED;
            break;
        case OPR_MODE_AMG:
        case OPR_MODE_NDOF_FMC_OFF:
        case OPR_MODE_NDOF:
            target_calib |= BNO_MAG_CALIBRATED | BNO_ACC_CALIBRATED | BNO_GYR_CALIBRATED;
            break;
    }   

    Serial.println("[BNO055]: Waiting for calibration...");
    do
    {
        // Serial.printf("[BNO055]: attempting to calibrate: result = 0x%X, target = 0x%X\n", result, target_calib);
        bno.read(BNO_CALIB_STAT, &result, true);
        
        // bits 5 and 4 are gyr stat, bits 3 and 2 are acc stat, bits 1 and 0 are mag stat
        Serial.printf("[BNO055]: Attempted to calibrate (0 = uncal, 3 = cal):  gyr:%u  acc:%u  mag:%u\n", \
            (result & 0x30) >> 4, (result & 0xC0) >> 2, (result & 0x03));
        delay(100);
    }while(result != target_calib);

    Serial.println("[BNO055]: finished calibrating");
}

bool bno055::check_IDs()
{
    uint8_t result[4] = {0};

    // read all of the IDs off the chip
    if(bno.read(BNO_CHIP_ID_REG, result, NUM_ID_REG, true))
    {
        if(result[0] == BNO_CHIP_ID && result[1] == BNO_ACC_ID && result[2] == BNO_ACC_ID && result[3] == BNO_GYR_ID)
        {
            Serial.println("[BNO055]: CHIP, ACC, MAG, and GYR ID correct.");
            return true;
        }
        Serial.println("[BNO055]: INCORRECT IDs READ. Is this the correct device?");
        return false;
    }
    else
    {
        Serial.println("[BNO055]: Failed to read device IDs.");
        return false;
    }
}

bool bno055::establish_comms()
{
    uint8_t result = 0;
    if(bno.read(BNO_CHIP_ID_REG, &result, true))
    {
        Serial.println("[BNO055]: Successfully established comms.");
        return true;
    }

    Serial.println("[BNO055]: FAILED TO COMMUNICATE WITH SENSOR.");
    return false;
}

void bno055::get_temp()
{
    uint8_t result = 0;
    bno.read(BNO_TEMP_REG, &result, true);
    Serial.printf("[BNO055]: Temperature is %u F\n", result);
}

bool bno055::get_sys_status()
{
    uint8_t result= 0;
    bno.read(BNO_SYS_STATUS_REG, &result, true);
    Serial.printf("[BNO055]: sys status reg is 0x%X\n", result);
    
    if(result == 0x01)
    {
        Serial.println("[BNO055]: SYS ERROR PRESENT");
        bno.read(BNO_SYS_ERR, &result, true);
        Serial.printf("[BNO055]: sys error reg is 0x%X\n", result);
        return false;
    } 

    return true;

}

void bno055::get_euler_ypr()
{
    uint8_t ypr_data[6] = {0}; //in that order exactly, yaw is array 0 and 1
    // register is automatically incremented when reading multiple bytes
    bno.read(BNO_EUL_HEADING_LSB, ypr_data, (size_t)6, true);
    // 1 degree = 16 LSB, so need to divide by 16 which is same as right shifting 4

    int16_t heading = REG_DATA_TO_VAL_S16(ypr_data[1], ypr_data[0]);
    int16_t pitch = REG_DATA_TO_VAL_S16(ypr_data[3], ypr_data[2]);
    int16_t roll = REG_DATA_TO_VAL_S16(ypr_data[5], ypr_data[4]);


    //notes on calibration. depending on the mode, you need to wait for different things to be calibrated.

    // perform adjustments on the angles. zero them out
    // heading = heading - (int16_t)(70 << 4);
    // pitch = pitch - (5 << 4);
    #warning "implement something to zero out heading on start"

    Serial.printf("Y:%f \tP:%f \tR:%f\n", (float)(heading) / 16.0f, (float)(pitch) / 16.0f, (float)(roll) / 16.0f );

}

opr_mode_e bno055::get_mode()
{
    uint8_t result;
    bno.read(BNO_OPR_MODE_REG, &result, true);
    return (opr_mode_e)(result & 0x0F);
     
}

bool bno055::reset_sys()
{
    bno.write(BNO_SYS_TRIGGER, (uint8_t)BITMASK_5, true);
    delay(1000); // wait for the chip to reboot

    // check to if comms restored and chip is working
    if(!establish_comms())
    {
        Serial.println("[BNO055]: Failed to reset sensor.");
        return false;
    }

    // reset the reset bit.
    bno.write(BNO_SYS_TRIGGER, (uint8_t)0x00, true);
    return true;
}

void bno055::set_mode(opr_mode_e mode)
{
    uint8_t result = 0;
    Serial.println("[BNO055]: Changing operation modes.. ");
    // bno.read(BNO_OPR_MODE_REG, &result, true);
    // delay(1);
    // bno.write(BNO_OPR_MODE_REG, (uint8_t)((result & 0xF0) | mode), true);
    Serial.printf("[BNO055]: writing mode 0x%X\n", (uint8_t)(mode));
    bno.write(BNO_OPR_MODE_REG, (uint8_t)(mode), true);
    delay(30); // at most needs, about 20ms to switch modes 
}

bool bno055::self_test()
{
    // power on self test (section 3.8)
    // the self test checks the acc, mag, and gyr ids and does an internal self check as well.
    uint8_t result;
    if(!bno.read(BNO_ST_RESULT, &result, true))
    {
        Serial.println("[BNO055]: Comms failed on Power on Self Test Passed");
        return false;
    }

    if((result & 0x0F) == 0xF)
    {
        Serial.println("[BNO055]: Power on Self Test Passed");
        return true;
    }
    
    for(uint8_t i = 0; i < 4; i++)
    {
        if(!(result & mask8[i]))
        {
            Serial.printf("[BNO055]: bit %u failed.\n", i);
        }
    }
    return false;
}

void bno055::set_units(uint8_t units)
{
    //config units
    // set temp to be F
    if(bno.write(BNO_UNIT_SEL, units, true))
    {
        Serial.println("[BNO055]: successfully wrote config units.");
    }
    delay(2);

    // read again to confirm selection
    uint8_t result;
    bno.read(BNO_UNIT_SEL, &result, true);
    Serial.printf("[BNO055]: Result is 0x%X (reading unit_sel reg after writing to it)\n", result);
}

bool bno055::set_clock_source(uint8_t clk_source)
{
    uint8_t result;
    //check sys clk status
    bno.read(BNO_SYS_CLK_STATUS, &result, true);
    if(!(result & 0x01)) //bit 0 of result needs to be 0
    {
        //if if statement true then, we are free to configure clk source
        Serial.println("[BNO055]: Free to config clk source, configuring..");
        //set to external oscillator
        //set clk to external osc
        bno.write(BNO_SYS_TRIGGER, clk_source, true);
        delay(1000); // takes a while to quick in
        Serial.println("[BNO055]: CLK SOURCE CONFIGURED");
        return true;
    }
    
    Serial.println("[BNO055]: system not available to config clk source.");
    return false;

    // theres no way to check whether the clock config was sucessful or not i dont think.
}

