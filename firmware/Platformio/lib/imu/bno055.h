#ifndef BNO055_H_
#define BNO055_H_

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
#define BNO_OPR_MODE_M4G 0x0A

#define BNO_UNIT_SEL 0x3B

#define BNO_SYS_STATUS_REG 0x39
#define BNO_SYS_ERR 0x3A

namespace bno055
{
    void init();
    void checkIDs();
    void getTemp();
    void getsysstatus();
}







#endif