#ifndef BNO055_H_
#define BNO055_H_

#include <stdint.h>

#warning "recall that the sensor can only output data so fast, don't need to read it every loop cause the data woun't be fresh."

#define BNO_I2C_ADDRESS 0x28
#define BNO_CHIP_ID_REG 0x00
#define BNO_ACC_ID_REG 0x01
#define BNO_MAG_ID_REG 0x02
#define BNO_GYR_ID_REG 0x03

#define BNO_CHIP_ID 0xA0
#define BNO_ACC_ID 0xFB
#define BNO_MAG_ID 0x32
#define BNO_GYR_ID 0x0F

#define BNO_TEMP_REG 0x34
#define BNO_TEMP_SOURCE_REG 0x40
#define BNO_TEMP_SRC_ACC 0x00
#define BNO_TEMP_SRC_GYR 0x01

#define BNO_OPR_MODE 0x3D
#define BNO_OPR_MODE_CONFIG 0x00
#define BNO_OPR_MODE_M4G 0x0A //for opr mode, the high 4 bits are reserved


#define BNO_UNIT_SEL 0x3B
#define TEMP_UNIT_F 0b10000
#define EULER_ANG_UNIT_RAD 0b100

#define BNO_SYS_STATUS_REG 0x39
#define BNO_SYS_ERR 0x3A
#define BNO_SYS_TRIGGER 0x3F
#define BNO_SYS_CLK_STATUS 0x38

#define BNO_ST_RESULT 0x36

#define BNO_CALIB_STAT 0x35

#define BNO_EUL_PITCH_MSB 0x1F
#define BNO_EUL_PITCH_LSB 0x1E
#define BNO_EUL_ROLL_MSB 0x1D
#define BNO_EUL_ROLL_LSB 0x1C
#define BNO_EUL_HEADING_MSB 0x1B
#define BNO_EUL_HEADING_LSB 0x1A

#define REG_DATA_TO_VAL_S16(msbyte, lsbyte) (((int16_t)(msbyte)) << 8 | ((int16_t)(lsbyte)))


namespace bno055
{
    void init();
    void checkIDs();
    void getTemp();
    void getsysstatus();
    void getEULypr();
}

inline int16_t map(int16_t val, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}





#endif