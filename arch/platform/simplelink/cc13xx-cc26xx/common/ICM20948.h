/*
 * ICM20648.h
 *
 *  Created on: 26 Dec 2020
 *      Author: worluk
 */

#ifndef CONTIKI_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_COMMON_ICM20948_H_
#define CONTIKI_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_COMMON_ICM20948_H_

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
/*---------------------------------------------------------------------------*/
/* The BMP-280 driver uses the I2C0 peripheral to access the senssor */
#if BOARD_SENSORS_ENABLE
#if (TI_I2C_CONF_ENABLE == 0) || (TI_I2C_CONF_I2C0_ENABLE == 0)
#error "The ICM20648 requires the I2C driver (TI_I2C_CONF_ENABLE = 1)"
#endif
#endif
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor ICM20948_sensor;




#define EXT_GYRO_SPI_CONTROLLER      Board_SPI0

#define USER_BANK_SEL   (0x7F)
#define USER_BANK_0     (0x00)
#define USER_BANK_1     (0x10)
#define USER_BANK_2     (0x20)
#define USER_BANK_3     (0x30)

#define PWR_MGMT_1      (0x06)
#define PWR_MGMT_2      (0x07)
#define GYRO_CONFIG_1   (0x01)


#define CLK_BEST_AVAIL  (0x01)
#define GYRO_RATE_250   (0x00)
#define GYRO_LPF_17HZ   (0x29)

void ICM_PowerOn();
uint8_t ICM_WHOAMI(void);
void ICM_SelectBank(uint8_t bank);
void ICM_ReadAccelGyro(void);
void ICM_ReadMag(int16_t magn[3]);
uint16_t ICM_Initialize(void);
void ICM_SelectBank(uint8_t bank);
void ICM_Disable_I2C(void);
void ICM_CSHigh(void);
void ICM_CSLow(void);
void ICM_SetClock(uint8_t clk);
void ICM_AccelGyroOff(void);
void ICM_AccelGyroOn(void);
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
void ICM_SetGyroLPF(uint8_t lpf);

#endif

