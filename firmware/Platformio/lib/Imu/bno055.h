#ifndef BNO055_H_
#define BNO055_H_

#include <stdint.h>

#warning "recall that the sensor can only output data so fast, don't need to read it every loop cause the data woun't be fresh."

#define BNO_RUN_MODE OPR_MODE_M4G

#define BNO_I2C_ADDRESS 0x28
#define NUM_ID_REG (size_t)(4)
#define BNO_CHIP_ID_REG 0x00
#define BNO_ACC_ID_REG 0x01
#define BNO_MAG_ID_REG 0x02
#define BNO_GYR_ID_REG 0x03
#define BNO_PAGE_ID_REG 0x07

#define BNO_CHIP_ID 0xA0
#define BNO_ACC_ID 0xFB
#define BNO_MAG_ID 0x32
#define BNO_GYR_ID 0x0F

#define BNO_TEMP_REG 0x34
#define BNO_TEMP_SOURCE_REG 0x40
#define BNO_TEMP_SRC_ACC 0x00
#define BNO_TEMP_SRC_GYR 0x01

#define BNO_OPR_MODE_REG 0x3D
#define BNO_OPR_MODE_CONFIG 0x00 //for opr mode, the high 4 bits are reserved
#define BNO_OPR_MODE_ACCONLY 0x01
#define BNO_OPR_MODE_MAGONLY 0x02
#define BNO_OPR_MODE_GYROONLY 0x03
#define BNO_OPR_MODE_ACCMAG 0x04
#define BNO_OPR_MODE_ACCGYRO 0x05
#define BNO_OPR_MODE_MAGGYRO 0x06
#define BNO_OPR_MODE_AMG 0x07
#define BNO_OPR_MODE_IMU 0x08
#define BNO_OPR_MODE_COMPASS 0x09
#define BNO_OPR_MODE_M4G 0x0A 
#define BNO_OPR_MODE_NDOF_FMC_OFF 0x0B
#define BNO_OPR_MODE_NDOF 0x0C

#define BNO_UNIT_SEL 0x3B
#define TEMP_UNIT_F 0b10000
#define EULER_ANG_UNIT_RAD 0b100
#define ANDRIOD_ORIENTATION 0x80

#define BNO_SYS_STATUS_REG 0x39
#define BNO_SYS_ERR 0x3A
#define BNO_SYS_TRIGGER 0x3F
#define BNO_SYS_CLK_STATUS 0x38
#define BNO_CLK_SRC_INTERNAL 0x00
#define BNO_CLK_SRC_EXTERNAL 0x80
#define BNO_POWER_MODE_REG 0x3E

#define BNO_ST_RESULT 0x36

#define BNO_CALIB_STAT 0x35
#define BNO_SYS_CALIBRATED 0xC0
#define BNO_GYR_CALIBRATED 0x30
#define BNO_ACC_CALIBRATED 0x0C
#define BNO_MAG_CALIBRATED 0x03 
#define BNO_GET_GYR_CAL(cal_reg) (((cal_reg) & BNO_GYR_CALIBRATED) >> 4)
#define BNO_GET_ACC_CAL(cal_reg) (((cal_reg) & BNO_ACC_CALIBRATED) >> 2)
#define BNO_GET_MAG_CAL(cal_reg) ((cal_reg) & BNO_MAG_CALIBRATED)


#define BNO_EUL_PITCH_MSB 0x1F
#define BNO_EUL_PITCH_LSB 0x1E
#define BNO_EUL_ROLL_MSB 0x1D
#define BNO_EUL_ROLL_LSB 0x1C
#define BNO_EUL_HEADING_MSB 0x1B
#define BNO_EUL_HEADING_LSB 0x1A

#define REG_DATA_TO_VAL_S16(msbyte, lsbyte) ((((int16_t)(msbyte)) << 8) | ((int16_t)(lsbyte)))


typedef enum opr_mode
{
    OPR_MODE_CONFIG = BNO_OPR_MODE_CONFIG,
    OPR_MODE_ACCONLY = BNO_OPR_MODE_ACCONLY,
    OPR_MODE_MAGONLY = BNO_OPR_MODE_MAGONLY,
    OPR_MODE_GYROONLY = BNO_OPR_MODE_GYROONLY,
    OPR_MODE_ACCMAG = BNO_OPR_MODE_ACCMAG,
    OPR_MODE_ACCGYRO = BNO_OPR_MODE_ACCGYRO,
    OPR_MODE_MAGGYRO = BNO_OPR_MODE_MAGGYRO,
    OPR_MODE_AMG = BNO_OPR_MODE_AMG,
    OPR_MODE_IMU = BNO_OPR_MODE_IMU,
    OPR_MODE_COMPASS = BNO_OPR_MODE_COMPASS,
    OPR_MODE_M4G = BNO_OPR_MODE_M4G,
    OPR_MODE_NDOF_FMC_OFF = BNO_OPR_MODE_NDOF_FMC_OFF,
    OPR_MODE_NDOF = BNO_OPR_MODE_NDOF

} opr_mode_e;


namespace bno055
{
    bool init();
    bool establish_comms();
    bool check_IDs();
    void get_temp();
    bool get_sys_status();
    void get_euler_ypr();
    bool reset_sys();
    void set_mode(opr_mode_e mode);
    opr_mode_e get_mode();
    bool self_test();
    void set_units(uint8_t units);
    bool set_clock_source(uint8_t clk_source);
    void calibrate(uint8_t mode);
    void print_calibration();
}






#endif