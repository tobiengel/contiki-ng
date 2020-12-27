/*
 * i2c.h
 *
 *  Created on: 24 Jan 2020
 *      Author: worluk
 */

#ifndef _I2CWRAPPER_H_
#define  _I2CWRAPPER_H_

#include <ti/drivers/I2C.h>
extern I2C_Handle i2c_handle;

bool initI2C();
bool i2c_write_read(uint8_t, void *, size_t , void *, size_t );


#define i2c_write(addr, writeBuf, writeCount)   i2c_write_read(addr,writeBuf, writeCount, NULL, 0)
#define i2c_read(addr,readBuf, readCount)      i2c_write_read(addr,NULL, 0, readBuf, readCount)

#endif /*  _I2CWRAPPER_H_ */
