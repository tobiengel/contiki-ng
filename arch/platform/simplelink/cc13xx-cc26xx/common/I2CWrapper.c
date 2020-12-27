/*
 * i2c.c
 *
 *  Created on: 24 Jan 2020
 *      Author: worluk
 */

#include "I2CWrapper.h"

#include <stdio.h>
#include <Board.h>

I2C_Handle i2c_handle;

bool initI2C(){

    if(i2c_handle) {
       return true;
    }

    I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);

    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate = I2C_400kHz;

    i2c_handle = I2C_open(Board_I2C0, &i2cParams);
    if(i2c_handle == NULL) {
       return false;
    }
    //printf("init i2c");

    return true;
}

bool i2c_write_read(uint8_t addr, void *writeBuf, size_t writeCount, void *readBuf, size_t readCount)
{
  I2C_Transaction i2cTransaction = {
    .writeBuf = writeBuf,
    .writeCount = writeCount,
    .readBuf = readBuf,
    .readCount = readCount,
    .slaveAddress = addr,//0x3c,
  };

  return I2C_transfer(i2c_handle, &i2cTransaction);
}



