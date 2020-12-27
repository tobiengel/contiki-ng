/*
 * ICM20648.c
 *
 *  Created on: 26 Dec 2020
 *      Author: worluk
 */


/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "sys/ctimer.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
#include "ICM20948.h"
/*---------------------------------------------------------------------------*/
#include <Board.h>

#include "dev/spi.h"
/*---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <posix/unistd.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];

 static const spi_device_t gyro_spi_configuration = {
 #if GPIO_HAL_PORT_PIN_NUMBERING
   .port_spi_sck = EXT_FLASH_SPI_PORT_SCK,
   .port_spi_miso = EXT_FLASH_SPI_PORT_MISO,
   .port_spi_mosi = EXT_FLASH_SPI_PORT_MOSI,
   .port_spi_cs = EXT_FLASH_SPI_PORT_CS,
 #endif
   .spi_controller = EXT_GYRO_SPI_CONTROLLER,
   .pin_spi_sck = CC1352RSTK_SPI0_CLK,
   .pin_spi_miso = CC1352RSTK_SPI0_MISO,
   .pin_spi_mosi = CC1352RSTK_SPI0_MOSI,
   .pin_spi_cs = CC1352RSTK_SPI0_CSN,
   .spi_bit_rate = 4000000,
   .spi_pha = 0,
   .spi_pol = 0
 };

typedef enum {
  SENSOR_STATUS_DISABLED,
  SENSOR_STATUS_INITIALISED,
  SENSOR_STATUS_NOT_READY,
  SENSOR_STATUS_READY
} SENSOR_STATUS;

static volatile SENSOR_STATUS sensor_status = SENSOR_STATUS_DISABLED;

static const spi_device_t *get_spi_conf(const spi_device_t *conf)
{
  if(conf == NULL) {
    return &gyro_spi_configuration;
  }
  return conf;
}

static bool select_on_bus(const spi_device_t *gyro_spi_configuration)
{
  if(spi_select(gyro_spi_configuration) == SPI_DEV_STATUS_OK) {
    return true;
  }
  return false;
}

static void deselect(const spi_device_t *gyro_spi_configuration)
{
  spi_deselect(gyro_spi_configuration);
}

/*
 *
 * SPI abstraction
 *
 */
void ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
    bool ret;
    reg = reg | 0x80;

    const spi_device_t *gyro_spi_configuration;
    gyro_spi_configuration = get_spi_conf(NULL);

    if(select_on_bus(gyro_spi_configuration) == false) {
      return;
    }

    if(spi_write(gyro_spi_configuration, &reg, 1) != SPI_DEV_STATUS_OK) {
        /* failure */
        deselect(gyro_spi_configuration);
        return;
    }

    ret = (spi_read(gyro_spi_configuration, pData, Size) == SPI_DEV_STATUS_OK);

    deselect(gyro_spi_configuration);

    return;
}

void ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{

    bool ret;
    reg = reg & 0x7F;

    const spi_device_t *gyro_spi_configuration;
    gyro_spi_configuration = get_spi_conf(NULL);

    if(select_on_bus(gyro_spi_configuration) == false) {
      return;
    }

    if(spi_write(gyro_spi_configuration, &reg, 1) != SPI_DEV_STATUS_OK) {
        /* failure */
        deselect(gyro_spi_configuration);
        return;
    }

    ret = (spi_write(gyro_spi_configuration, pData, Size) == SPI_DEV_STATUS_OK);

    deselect(gyro_spi_configuration);

    return;

}

void ICM_ReadOneByte(uint8_t reg, uint8_t* pData) // ***
{
    ICM_readBytes(reg, pData, 1);
}

void ICM_WriteOneByte(uint8_t reg, uint8_t Data) // ***
{
    ICM_WriteBytes(reg, &Data, 1);
}

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
void i2c_Mag_write(uint8_t reg,uint8_t value)
  {
    ICM_WriteOneByte(0x7F, 0x30);

    usleep(1000);
    ICM_WriteOneByte(0x03 ,0x0C);//mode: write

    usleep(1000);
    ICM_WriteOneByte(0x04 ,reg);//set reg addr

    usleep(1000);
    ICM_WriteOneByte(0x06 ,value);//send value

    usleep(1000);
  }

  static uint8_t ICM_Mag_Read(uint8_t reg)
  {
    uint8_t  Data;
    ICM_WriteOneByte(0x7F, 0x30);
    usleep(1000);
    ICM_WriteOneByte(0x03 ,0x0C|0x80);
    usleep(1000);
    ICM_WriteOneByte(0x04 ,reg);// set reg addr
    usleep(1000);
    ICM_WriteOneByte(0x06 ,0xff);//read
    usleep(1000);
    ICM_WriteOneByte(0x7F, 0x00);
    ICM_ReadOneByte(0x3B,&Data);
    usleep(1000);
    return Data;
  }

  void ICM20948_READ_MAG(int16_t magn[3])
  {
    uint8_t mag_buffer[10];

      mag_buffer[0] =ICM_Mag_Read(0x01);

      mag_buffer[1] =ICM_Mag_Read(0x11);
      mag_buffer[2] =ICM_Mag_Read(0x12);
      magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
        mag_buffer[3] =ICM_Mag_Read(0x13);
      mag_buffer[4] =ICM_Mag_Read(0x14);
        magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
        mag_buffer[5] =ICM_Mag_Read(0x15);
      mag_buffer[6] =ICM_Mag_Read(0x16);
        magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

        i2c_Mag_write(0x31,0x01);
  }

/*
 *
 * Read magnetometer
 *
 */
void ICM_ReadMag(int16_t magn[3]) {
    uint8_t mag_buffer[10];

    mag_buffer[0] =ICM_Mag_Read(0x01);

    mag_buffer[1] =ICM_Mag_Read(0x11);
    mag_buffer[2] =ICM_Mag_Read(0x12);
    magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
    mag_buffer[3] =ICM_Mag_Read(0x13);
    mag_buffer[4] =ICM_Mag_Read(0x14);
    magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
    mag_buffer[5] =ICM_Mag_Read(0x15);
    mag_buffer[6] =ICM_Mag_Read(0x16);
    magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

    i2c_Mag_write(0x31,0x01);
}

void ICM_CSHigh(){
    const spi_device_t *gyro_spi_configuration;
    gyro_spi_configuration = get_spi_conf(NULL);

    if(select_on_bus(gyro_spi_configuration) == false) {
      return;
    }

}

/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM_PowerOn(void) {
    char uart_buffer[200];
    uint8_t whoami = 0xEA;
    volatile uint8_t test = ICM_WHOAMI();
    //if (test == whoami) {
       // ICM_CSHigh();
        usleep(10000);
        ICM_SelectBank(USER_BANK_0);
        usleep(10000);
        ICM_Disable_I2C();
        usleep(10000);
        ICM_SetClock((uint8_t)CLK_BEST_AVAIL);
        usleep(10000);
        ICM_AccelGyroOff();
        usleep(10000);
        ICM_AccelGyroOn();
        usleep(10000);
        ICM_Initialize();
    //} else {
        //sprintf(uart_buffer, "Failed WHO_AM_I.  %i is not 0xEA\r\n", test);
        //HAL_UART_Transmit_DMA(UART_BUS, (uint8_t*) uart_buffer, strlen(uart_buffer));
        //HAL_Delay(100);
    //}
}
uint16_t ICM_Initialize(void) {
        ICM_SelectBank(USER_BANK_2);
        usleep(20000);
        ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
        usleep(10000);

        // Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
        ICM_WriteOneByte(0x00, 0x0A);
        usleep(10000);

        // Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
        ICM_WriteOneByte(0x14, (0x04 | 0x11));

        // Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
        ICM_WriteOneByte(0x10, 0x00);
        usleep(10000);

        // Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
        ICM_WriteOneByte(0x11, 0x0A);
        usleep(10000);

        ICM_SelectBank(USER_BANK_2);
        usleep(10000);

        // Configure AUX_I2C Magnetometer (onboard ICM-20948)
        ICM_WriteOneByte(0x7F, 0x00); // Select user bank 0
        ICM_WriteOneByte(0x0F, 0x30); // INT Pin / Bypass Enable Configuration
        ICM_WriteOneByte(0x03, 0x20); // I2C_MST_EN
        ICM_WriteOneByte(0x7F, 0x30); // Select user bank 3
        ICM_WriteOneByte(0x01, 0x4D); // I2C Master mode and Speed 400 kHz
        ICM_WriteOneByte(0x02, 0x01); // I2C_SLV0 _DLY_ enable
        ICM_WriteOneByte(0x05, 0x81); // enable IIC and EXT_SENS_DATA==1 Byte

        // Initialize magnetometer
        i2c_Mag_write(0x32, 0x01); // Reset AK8963
        usleep(10000);;
        i2c_Mag_write(0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output

        return 1337;
    }

void ICM_ReadAccelGyro(void) {
    uint8_t raw_data[12];
    ICM_readBytes(0x2D, raw_data, 12);

    accel_data[0] = (raw_data[0] << 8) | raw_data[1];
    accel_data[1] = (raw_data[2] << 8) | raw_data[3];
    accel_data[2] = (raw_data[4] << 8) | raw_data[5];

    gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
    gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
    gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

    accel_data[0] = accel_data[0] / 8;
    accel_data[1] = accel_data[1] / 8;
    accel_data[2] = accel_data[2] / 8;

    gyro_data[0] = gyro_data[0] / 250;
    gyro_data[1] = gyro_data[1] / 250;
    gyro_data[2] = gyro_data[2] / 250;
}
void ICM_SelectBank(uint8_t bank) {
    ICM_WriteOneByte(USER_BANK_SEL, bank);
}
void ICM_Disable_I2C(void) {
    ICM_WriteOneByte(0x03, 0x78);
}

void ICM_SetClock(uint8_t clk) {
    ICM_WriteOneByte(PWR_MGMT_1, clk);
}
void ICM_AccelGyroOff(void) {
    ICM_WriteOneByte(PWR_MGMT_2, (0x38 | 0x07));
}
void ICM_AccelGyroOn(void) {
    ICM_WriteOneByte(0x07, (0x00 | 0x00));
}
uint8_t ICM_WHOAMI(void) {
    uint8_t spiData = 0x01;
    ICM_ReadOneByte(0x00, &spiData);
    return spiData;
}
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf) {
    ICM_WriteOneByte(GYRO_CONFIG_1, (rate|lpf));
}
/*
 *
 * Read Accelerometer and Gyro data
 *
 */

static int status(int type)  {
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return sensor_status;
  default:
    return SENSOR_STATUS_DISABLED;
  }
}

static int configure(int type, int enable) {

  switch(type) {

  case SENSORS_HW_INIT:

    ICM_PowerOn();
    sensor_status = SENSOR_STATUS_INITIALISED;
    break;

  case SENSORS_ACTIVE:
    /* Must be initialised first */
    if(sensor_status == SENSOR_STATUS_DISABLED) {
      break;
    }
    if(enable) {
//      enable_sensor(true);
//      ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
//      sensor_status = SENSOR_STATUS_NOT_READY;
    } else {
//      ctimer_stop(&startup_timer);
//      enable_sensor(false);
//      sensor_status = SENSOR_STATUS_INITIALISED;
    }
    break;

  default:
    break;
  }
  return sensor_status;
}

static int value(int type){

    return 0;
}


SENSORS_SENSOR(ICM20948_sensor, "ICM20948", value, configure, status);
