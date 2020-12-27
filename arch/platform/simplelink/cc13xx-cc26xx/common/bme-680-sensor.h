/*---------------------------------------------------------------------------*/
#ifndef BMP_680_SENSOR_H_
#define BMP_680_SENSOR_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
/*---------------------------------------------------------------------------*/
/* The BMP-280 driver uses the I2C0 peripheral to access the senssor */
#if BOARD_SENSORS_ENABLE
#if (TI_I2C_CONF_ENABLE == 0) || (TI_I2C_CONF_I2C0_ENABLE == 0)
#error "The BME680 requires the I2C driver (TI_I2C_CONF_ENABLE = 1)"
#endif
#endif
/*---------------------------------------------------------------------------*/
typedef enum {
  BMP_680_SENSOR_TYPE_TEMP,
  BMP_680_SENSOR_TYPE_PRESS,
  BMP_680_SENSOR_TYPE_HUM,
  BMP_680_SENSOR_TYPE_IAQ,
  BMP_680_SENSOR_TYPE_IAQ_ACC,
  BMP_680_SENSOR_TYPE_CO2,
} BMP_680_SENSOR_TYPE;
/*---------------------------------------------------------------------------*/
#define BMP_680_READING_ERROR    -1
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor bmp_680_sensor;
/*---------------------------------------------------------------------------*/
#endif /* BMP_280_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
