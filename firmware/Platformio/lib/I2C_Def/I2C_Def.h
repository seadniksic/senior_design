#ifndef I2C_DEF_H_
#define I2C_DEF_H_

#include "i2c_device.h"
#include "config.h"

inline void I2C_Def_Init(I2CMaster & master)
{
    master.begin(I2C_SPEED);
}

#endif