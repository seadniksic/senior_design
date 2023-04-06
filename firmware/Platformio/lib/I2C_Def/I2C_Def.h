#ifndef I2C_DEF_H_
#define I2C_DEF_H_

#include "i2c_device.h"
#include "config.h"

void I2C_Def_Init();
I2CDevice * I2C_Def_GetBnoDev();
I2CDevice * I2C_Def_GetHtsDev();

#endif