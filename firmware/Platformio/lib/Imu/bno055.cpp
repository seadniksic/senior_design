#include "bno055.h"
#include "util.h"
#include "main.h"

static I2CDevice * bno;
static uint8_t Data_Buffer[6] ={0};
static Vec3_Data_t vec3_data = {0};
Calib_Data_t calib_data = {0};
Calib_Data_t ACTUAL_CALIB_DATA = {0};


bool bno055::init(I2CDevice * dev, bool run_calib)
{
    bno = dev;

    // Initialize calibration profile (for testing)
    initialize_calib_profile(ACTUAL_CALIB_DATA);
    
    // return false;
    // return false;
    // check to if comms up and chip is working
    if(!establish_comms())
    {
        return false;
    }

    // Serial.println("[BNO055]: Resetting the sensor...");
    if(!reset_sys())
    {
        return false;
    }

    // change to config mode (although it should already be in config mode because of the reset)
    if(!enter_config_mode())
    {
        Serial.println("Failed to enter the config mode");
        return false;
    }

    // set to normal power mode
    bno->write(BNO_POWER_MODE_REG, (uint8_t)0x00, true);

    // set to page 0
    bno->write(BNO_PAGE_ID_REG, (uint8_t)0x00, true);

    // perform self test
    if(!self_test())
    {
        return false;
    }

    // change temp output to be F
    // #pragma message("need to update get data functions cause LSB changes when u update units")
    set_units(TEMP_UNIT_F | ANDRIOD_ORIENTATION | EULER_ANG_UNIT_RAD);   
    // uint8_t result; 
    // bno->read(BNO_UNIT_SEL, &result, true);
    // Serial.printf("the value of unit sel reg is : 0x%X\n", result);


    // check state of the system at this point, if all is good proceed.
    if(!get_sys_status())
    {
        return false;
    }

    // #warning "why did i remove this?"
    // looks like i did while trying to fix it, maybe i thought ths was one of the problems. we definitely want to set the clock source to be an output tho
    // Set clock to external
    if(!set_clock_source(BNO_CLK_SRC_EXTERNAL))
    {
        return false;
    }

    Serial.println("hello1");
    get_sys_status();

    //write the stored calibration
    Serial.println("Writing the stored calibration values.");
    if(!write_calib_profile(ACTUAL_CALIB_DATA))
    {
        Serial.println("Failed to write calibration profile. Values may be unstable!!");
    }

    // set out of config mode
    // opr_mode_e desired_mode = (opr_mode_e)BNO_RUN_MODE;

    // if(!enter_run_mode())
    // {
    //     Serial.println("Failed to enter run mode");
    //     return false;
    // }

    // // Sensor calibration
    // if(run_calib)
    // {
    //     calibrate(desired_mode);
    // }
    // else
    // {
    //     Serial.println("-----------------------------");
    //     delay(1);
    //     Serial.println("-----------------------------");
    //     delay(1);
    //     Serial.println("-----------------------------");
    //     delay(1);
    //     Serial.println("WARNING: OMITTING CALIBRATION");
    //     delay(1);
    //     Serial.println("-----------------------------");
    //     delay(1);
    //     Serial.println("-----------------------------");
    //     delay(1);
    //     Serial.println("-----------------------------");
    // }

    // check state of the system at this point, if all is good proceed.
    if(!get_sys_status())
    {
        return false;
    }

    Serial.println("[BNO055]: COMPLETED INIT");
    return true;
}

void bno055::initialize_calib_profile(Calib_Data_t & cal_data)
{
    cal_data.acc.x.s_16 = 424;
    cal_data.acc.y.s_16 = -424;
    cal_data.acc.z.s_16 = 377;
    cal_data.mag.x.s_16 = -208;
    cal_data.mag.y.s_16 = 280;
    cal_data.mag.z.s_16 = -244;
    cal_data.gyr.x.s_16 = -1;
    cal_data.gyr.y.s_16 = -2;
    cal_data.gyr.z.s_16 = 0;
    cal_data.acc_rad = 1000; 
    cal_data.mag_rad = 984; 
}

void bno055::calibrate(uint8_t mode)
{
    uint8_t result = 0, target_calib = 0;
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
    uint8_t count = 0;
    bool run = true;
    do
    {
        // Serial.printf("[BNO055]: attempting to calibrate: result = 0x%X, target = 0x%X\n", result, target_calib);
        bno->read(BNO_CALIB_STAT, &result, true);
        // bits 5 and 4 are gyr stat, bits 3 and 2 are acc stat, bits 1 and 0 are mag stat
        // 0 is uncal, 2 = cal
        Serial.printf("[BNO055]: Calibrating...: g:%u a:%u m:%u s:%u; Desired:%u,%u,%u,%u; current: 0x%X, Desired 0x%X\n", \
            BNO_GET_GYR_CAL(result), BNO_GET_ACC_CAL(result), BNO_GET_MAG_CAL(result), BNO_GET_SYS_CAL(result), \
            BNO_GET_GYR_CAL(target_calib), BNO_GET_ACC_CAL(target_calib), BNO_GET_MAG_CAL(target_calib), BNO_GET_SYS_CAL(target_calib), \
            result, target_calib | BNO_SYS_CALIBRATED);
        
        if(result == (target_calib | BNO_SYS_CALIBRATED))
        {
            if(++count > 30)
            {
                run = false;
            }
        }
        else
        {
            count = 0;
        }
        
        delay(100);

    } while(run);

    Serial.println("[BNO055]: finished calibrating");
    delay(2000);
}

void bno055::print_calibration()
{
    uint8_t result = 0;
    // Serial.printf("[BNO055]: attempting to calibrate: result = 0x%X, target = 0x%X\n", result, target_calib);
    bno->read(BNO_CALIB_STAT, &result, true);
        // bits 5 and 4 are gyr stat, bits 3 and 2 are acc stat, bits 1 and 0 are mag stat
    Serial.printf("gyr_cal=%u \t acc_cal=%u \t mag_cal=%u \t sys_cal=%u\n", \
        BNO_GET_GYR_CAL(result), BNO_GET_ACC_CAL(result), BNO_GET_MAG_CAL(result), BNO_GET_SYS_CAL(result));
    
}

void bno055::get_calib_stat(uint8_t & status)
{
    if(!bno->read(BNO_CALIB_STAT, &status, true))
    {
        status = 0;
        g_watcher.i2c_msg_failed++;
        return;
    }
    g_watcher.i2c_msg_passed++;
}


bool bno055::get_calib_profile()
{
    /*
    The calibration profile includes sensor offsets and sensor radius. Host system can read the
    offsets and radius only after a full calibration is achieved and the operation mode is switched
    to CONFIG_MODE. Refer to sensor offsets and sensor radius registers.
    */

    // need to switch to config mode to read calibration data
    // if any of the reads fails and the function returns false, need to ensure we are back in run mode
    // if(!enter_config_mode())
    // {
    //     return false;
    // }

    // Serial.println("Getting offsets1");
    if(!bno->read(ACC_OFFSET_X_LSB, Data_Buffer, (size_t)6,true))
    {
        Serial.println("Failed to get ACC_OFFSETs");
        enter_run_mode();
        return false;
    }

    calib_data.acc.x.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[1], Data_Buffer[0]);
    calib_data.acc.y.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[3], Data_Buffer[2]);
    calib_data.acc.z.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[5], Data_Buffer[4]);

    // Serial.println("Getting offsets2");
    if(!bno->read(MAG_OFFSET_X_LSB, Data_Buffer, (size_t)6,true))
    {
        Serial.println("Failed to get MAG_OFFSETS");
        enter_run_mode();
        return false;
    }

    calib_data.mag.x.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[1], Data_Buffer[0]);
    calib_data.mag.y.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[3], Data_Buffer[2]);
    calib_data.mag.z.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[5], Data_Buffer[4]);

    // Serial.println("Getting offsets3");
    if(!bno->read(GYR_OFFSET_X_LSB, Data_Buffer, (size_t)6,true))
    {
        Serial.println("Failed to get GYR_OFFSETS");
        enter_run_mode();
        return false;
    }

    calib_data.gyr.x.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[1], Data_Buffer[0]);
    calib_data.gyr.y.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[3], Data_Buffer[2]);
    calib_data.gyr.z.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[5], Data_Buffer[4]);

    // Serial.println("Getting offsets4");
    if(!bno->read(ACC_RADIUS_LSB, Data_Buffer, (size_t)2,true))
    {
        Serial.println("Failed to get ACC_RADIUS");
        enter_run_mode();
        return false;
    }

    calib_data.acc_rad = REG_DATA_TO_VAL_S16(Data_Buffer[1], Data_Buffer[0]);

    // Serial.println("Getting offsets5");
    if(!bno->read(MAG_RADIUS_LSB, Data_Buffer, (size_t)2,true))
    {
        Serial.println("Failed to get MAG_RADIUS");
        enter_run_mode();
        return false;
    }

    calib_data.mag_rad = REG_DATA_TO_VAL_S16(Data_Buffer[1], Data_Buffer[0]);


    // Serial.println("Reach here");

    // if(!enter_run_mode())
    // {
    //     Serial.println("failed to enter back into run mode");
    //     return false;
    // }
    // else
    // {
    //     Serial.println("successfully back in run mode");
    // }

    // Serial.println("returning true");

    return true;
}

bool bno055::write_sensor_offsets(const Offset_t & data, uint8_t start_reg)
{
    bool status = true;
    // Serial.printf("%u ", data.x.us_8[DATA_LSB]);
    // Serial.printf("%u ", data.x.us_8[DATA_MSB]);
    // Serial.printf("%u ", data.y.us_8[DATA_LSB]);
    // Serial.printf("%u ", data.y.us_8[DATA_MSB]);
    // Serial.printf("%u ", data.z.us_8[DATA_LSB]);
    // Serial.printf("%u ", data.z.us_8[DATA_MSB]);
    // Serial.println("");
    status = status && bno->write(start_reg++, data.x.us_8[DATA_LSB], true);
    status = status && bno->write(start_reg++, data.x.us_8[DATA_MSB], true);
    status = status && bno->write(start_reg++, data.y.us_8[DATA_LSB], true);
    status = status && bno->write(start_reg++, data.y.us_8[DATA_MSB], true);
    status = status && bno->write(start_reg++, data.z.us_8[DATA_LSB], true);
    status = status && bno->write(start_reg++, data.z.us_8[DATA_MSB], true);
    return status;
}

bool bno055::write_sensor_radius(const int16_t & data, uint8_t start_reg)
{
    /*
    There are 4 bytes (2 bytes for each accelerometer and magnetometer) to configure the
    radius. Configuration will take place only when user writes to the last byte (i.e.,
    ACC_RADIUS_MSB and MAG_RADIUS_MSB). Therefore the last byte must be written
    whenever the user wants to changes the configuration. The range of the radius for
    accelerometer is +/-1000, magnetometer is +/-960 and Gyroscope is NA.
    */

    bool status = true;
    // Serial.printf("radius data: %X", data);
    // Serial.printf("msb: %X", (uint8_t)((data & (int16_t)0xFF00) >> 8));
    // Serial.printf("lsb: %X", (uint8_t)(data & (int16_t)0x00FF));
    status = status && bno->write(++start_reg, (uint8_t)((data & (int16_t)0xFF00) >> 8), true); //write MSB, ++ start_reg because lsb is passed in
    status = status && bno->write(--start_reg, (uint8_t)(data & (int16_t)0x00FF), true); //write LSB, -- to get back down to lsb byte
    return status;
}

bool bno055::write_calib_profile(const Calib_Data_t & cal_data)
{
    /*
    Setting Calibration profile
    It is important that the correct offsets and corresponding sensor radius are used. Incorrect
    offsets may result in unreliable orientation data even at calibration accuracy level 3. To set
    the calibration profile the following steps need to be taken
    1. Select the operation mode to CONFIG_MODE
    2. Write the corresponding sensor offsets and radius data
    3. Change operation mode to fusion mode
    */

    // if(!enter_config_mode())
    // {
    //     return false;
    // }

    if(!write_sensor_offsets(cal_data.acc, ACC_OFFSET_X_LSB))
    {
        Serial.println("Failled at writing ACC offsets");
        return false;
    }

    if(!write_sensor_offsets(cal_data.mag, MAG_OFFSET_X_LSB))
    {
        Serial.println("Failled at writing MAG offsets");
        return false;
    }
    
    if(!write_sensor_offsets(cal_data.gyr, GYR_OFFSET_X_LSB))
    {
        Serial.println("Failled at writing GYR offsets");
        return false;
    }

    if(!write_sensor_radius(cal_data.mag_rad, MAG_RADIUS_LSB))
    {
        Serial.println("Failled at writing MAG radius");
        return false;
    }

    if(!write_sensor_radius(cal_data.acc_rad, ACC_RADIUS_LSB))
    {
        Serial.println("Failled at writing ACC radius");
        return false;
    }

    // get_calib_profile();
    // print_calib_profile();

    if(!enter_run_mode())
    {
        return false;
    }

    return true;
}



void bno055::print_calib_profile()
{

    const uint8_t delay_time = 1;

    Serial.printf("ACC OFFSETS__");
    delay(delay_time);
    Serial.printf("X: %d, Y: %d, Z: %d\n",calib_data.acc.x.s_16, calib_data.acc.y.s_16, calib_data.acc.z.s_16);
    delay(delay_time);

    Serial.printf("MAG OFFSETS__");
    delay(delay_time);
    Serial.printf("X: %d, Y: %d, Z: %d\n",calib_data.mag.x.s_16, calib_data.mag.y.s_16, calib_data.mag.z.s_16);
    delay(delay_time);

    Serial.printf("GYR OFFSETS__");
    delay(delay_time);
    Serial.printf("X: %d, Y: %d, Z: %d\n",calib_data.gyr.x.s_16, calib_data.gyr.y.s_16, calib_data.gyr.z.s_16);
    delay(delay_time);

    Serial.printf("ACC RAD__");
    delay(delay_time);
    Serial.printf("rad: %d \n",calib_data.acc_rad);
    delay(delay_time);

    Serial.printf("MAG RAD__");
    delay(delay_time);
    Serial.printf("rad: %d \n",calib_data.mag_rad);
    delay(delay_time);
}

bool bno055::check_IDs()
{
    uint8_t result[4] = {0};

    // read all of the IDs off the chip
    if(bno->read(BNO_CHIP_ID_REG, result, NUM_ID_REG, true))
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
    if(bno->read(BNO_CHIP_ID_REG, &result, true))
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
    bno->read(BNO_TEMP_REG, &result, true);
    Serial.printf("[BNO055]: Temperature is %u F\n", result);
}

bool bno055::get_sys_status()
{
    uint8_t result= 0;
    bno->read(BNO_SYS_STATUS_REG, &result, true);
    Serial.printf("[BNO055]: sys status reg is 0x%X\n", result);
    
    if(result == 0x01)
    {
        Serial.println("[BNO055]: SYS ERROR PRESENT");
        bno->read(BNO_SYS_ERR, &result, true);
        Serial.printf("[BNO055]: sys error reg is 0x%X\n", result);
        return false;
    } 

    return true;

}

void bno055::get_euler_ypr(SLAM_Data * sd)
{
    // register is automatically incremented when reading multiple bytes
    if(!bno->read(BNO_EUL_HEADING_LSB, Data_Buffer, (size_t)6, true))
    {
        Serial.println("euler ypr read failing");
        g_watcher.i2c_msg_failed++;
        return;
    }

    g_watcher.i2c_msg_passed++;

    vec3_data.v1.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[1], Data_Buffer[0]); //yaw
    vec3_data.v2.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[3], Data_Buffer[2]); //pitch
    vec3_data.v3.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[5], Data_Buffer[4]); //roll

    sd->set_eul_y((float)vec3_data.v1.s_16 / 900.0f);
    sd->set_eul_p((float)vec3_data.v2.s_16 / 900.0f);
    sd->set_eul_r((float)vec3_data.v3.s_16 / 900.0f);

    // Serial.printf("Y:%f \tP:%f \tR:%f\n", sd->get_eul_y(), sd->get_eul_p(), sd->get_eul_r());
}

void bno055::get_lia_xyz(SLAM_Data * sd)
{
    if(!bno->read(BNO_LIA_X_LSB, Data_Buffer, (size_t)6, true))
    {
        Serial.println("lia xyz read failing");
        g_watcher.i2c_msg_failed++;
        return;
    }

    g_watcher.i2c_msg_passed++;

    vec3_data.v1.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[1], Data_Buffer[0]); //x
    vec3_data.v2.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[3], Data_Buffer[2]); //y
    vec3_data.v3.s_16 = REG_DATA_TO_VAL_S16(Data_Buffer[5], Data_Buffer[4]); //z

    sd->set_lia_x((float)vec3_data.v1.s_16 / 100.0f);
    sd->set_lia_y((float)vec3_data.v2.s_16 / 100.0f);
    sd->set_lia_z((float)vec3_data.v3.s_16 / 100.0f);

    // Serial.printf("X:%f \tY:%f \tZ:%f\n", sd->get_lia_x(), sd->get_lia_y(), sd->get_lia_z());
}



opr_mode_e bno055::get_mode()
{
    uint8_t result;
    bno->read(BNO_OPR_MODE_REG, &result, true);
    return (opr_mode_e)(result & 0x0F);
     
}

bool bno055::reset_sys()
{
    bno->write(BNO_SYS_TRIGGER, (uint8_t)BITMASK_5, true);
    delay(1000); // wait for the chip to reboot

    // check to if comms restored and chip is working
    if(!establish_comms())
    {
        Serial.println("[BNO055]: Failed to reset sensor.");
        return false;
    }

    // reset the reset bit.
    bno->write(BNO_SYS_TRIGGER, (uint8_t)0x00, true);
    return true;
}

void bno055::set_mode(opr_mode_e mode)
{
    Serial.println("[BNO055]: Changing operation modes.. ");
    // Serial.printf("[BNO055]: writing mode 0x%X\n", (uint8_t)(mode));
    bno->write(BNO_OPR_MODE_REG, (uint8_t)(mode), true);
    Serial.println("Done writing mode");
    delay(30); // at most needs, about 20ms to switch modes 
}

bool bno055::enter_config_mode()
{
    uint8_t error_count = 0;
    set_mode(OPR_MODE_CONFIG);
    while(get_mode() != OPR_MODE_CONFIG)
    {
        Serial.println("[BNO055]: Failed to enter config mode, retrying..");
        set_mode(OPR_MODE_CONFIG);
        if(++error_count > 5)
        {
            return false;
        }
    }
    Serial.println("Succesfully entered config mode");
    return true;
}

bool bno055::enter_run_mode()
{
    uint8_t error_count = 0;
    set_mode((opr_mode_e)BNO_RUN_MODE);
    while(get_mode() != (opr_mode_e)BNO_RUN_MODE)
    {
        Serial.println("[BNO055]: Failed to set mode, retrying..");
        set_mode((opr_mode_e)BNO_RUN_MODE);
        if(++error_count > 5)
        {
            return false;
        }
    }
    Serial.println("Entered run mode");
    return true;
}

bool bno055::self_test()
{
    // power on self test (section 3.8)
    // the self test checks the acc, mag, and gyr ids and does an internal self check as well.
    uint8_t result;
    if(!bno->read(BNO_ST_RESULT, &result, true))
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
    if(bno->write(BNO_UNIT_SEL, units, true))
    {
        Serial.println("[BNO055]: successfully wrote config units.");
    }
    delay(2);

    // read again to confirm selection
    uint8_t result;
    bno->read(BNO_UNIT_SEL, &result, true);
    Serial.printf("[BNO055]: Result is 0x%X (reading unit_sel reg after writing to it)\n", result);
}

bool bno055::set_clock_source(uint8_t clk_source)
{
    uint8_t result;
    //check sys clk status
    bno->read(BNO_SYS_CLK_STATUS, &result, true);
    if(!(result & 0x01)) //bit 0 of result needs to be 0
    {
        //if if statement true then, we are free to configure clk source
        Serial.println("[BNO055]: Free to config clk source, configuring..");
        //set to external oscillator
        //set clk to external osc
        bno->write(BNO_SYS_TRIGGER, clk_source, true);
        delay(1000); // takes a while to quick in
        Serial.println("[BNO055]: CLK SOURCE CONFIGURED");
        return true;
    }
    
    Serial.println("[BNO055]: system not available to config clk source.");
    return false;

    // theres no way to check whether the clock config was sucessful or not i dont think.
}

void bno055::store_calib_status(GUI_Data * gd)
{
    uint8_t status;
    get_calib_stat(status);
    gd->set_calib_status(status);
}

