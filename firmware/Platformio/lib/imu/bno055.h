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

namespace bno055
{
    void init();
    void checkIDs();
    void getTemp();

}





#endif