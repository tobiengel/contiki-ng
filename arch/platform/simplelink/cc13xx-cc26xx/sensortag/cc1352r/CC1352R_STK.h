/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       CC1352RSTK.h
 *
 *  @brief      CC1352R SensorTag Board Specific header file.
 *
 *  The CC1352RSTK header file should be included in an application as
 *  follows:
 *  @code
 *  #include "CC1352RSTK.h"
 *  @endcode
 *  ============================================================================
 */
#ifndef __CC1352RSTK_BOARD_H__
#define __CC1352RSTK_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "contiki-conf.h"
#define TI_UART_CONF_ENABLE 1
#define TI_UART_CONF_UART0_ENABLE 1
#define TI_UART_CONF_BAUD_RATE   19200
/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#define CC1352RSTK

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>   <comments>
 */

/* Analog Capable DIOs */
#define CC1352RSTK_DIO23_ANALOG          IOID_23
#define CC1352RSTK_DIO24_ANALOG          IOID_24
#define CC1352RSTK_DIO25_ANALOG          IOID_25
#define CC1352RSTK_DIO26_ANALOG          IOID_26
#define CC1352RSTK_DIO27_ANALOG          IOID_27
#define CC1352RSTK_DIO28_ANALOG          IOID_28
#define CC1352RSTK_DIO29_ANALOG          IOID_29
#define CC1352RSTK_DIO30_ANALOG          IOID_30

/* Digital IOs */
#define CC1352RSTK_DIO12                 IOID_12
#define CC1352RSTK_DIO15                 IOID_15
#define CC1352RSTK_DIO16_TDO             IOID_16
#define CC1352RSTK_DIO17_TDI             IOID_17
#define CC1352RSTK_DIO21                 IOID_21
#define CC1352RSTK_DIO22                 IOID_22
#define CC1352RSTK_DIO24                 IOID_24
#define CC1352RSTK_DIO30                 IOID_30

/* Discrete Inputs */
#define CC1352RSTK_PIN_BTN1              IOID_15
#define CC1352RSTK_PIN_BTN2              IOID_14
#define CC1352STK_KEY_LEFT              CC1352STK_PIN_BTN2
#define CC1352STK_KEY_RIGHT             CC1352STK_PIN_BTN1
#define CC1352STK_CAPTOUCH_INT1          IOID_22
#define CC1352STK_CAPTOUCH_INT2          IOID_24


/* GPIO */
#define CC1352RSTK_GPIO_LED_ON           1
#define CC1352RSTK_GPIO_LED_OFF          0

/* I2C */
#define CC1352RSTK_I2C0_SCL0             IOID_4
#define CC1352RSTK_I2C0_SDA0             IOID_5


/* I2S */
#define CC1352RSTK_I2S_ADO               IOID_25
#define CC1352RSTK_I2S_ADI               IOID_26
#define CC1352RSTK_I2S_BCLK              IOID_27
#define CC1352RSTK_I2S_MCLK              PIN_UNASSIGNED
#define CC1352RSTK_I2S_WCLK              IOID_28

/* LEDs */
#define CC1352RSTK_PIN_LED_ON            1
#define CC1352RSTK_PIN_LED_OFF           0
#define CC1352RSTK_PIN_RLED              IOID_6
#define CC1352RSTK_PIN_GLED              IOID_7
#define CC1352RSTK_PIN_BLED              IOID_21

/* PWM Outputs */
#define CC1352RSTK_PWMPIN0               CC1352RSTK_PIN_RLED
#define CC1352RSTK_PWMPIN1               CC1352RSTK_PIN_GLED
#define CC1352RSTK_PWMPIN2               CC1352RSTK_PIN_BLED
#define CC1352RSTK_PWMPIN3               PIN_UNASSIGNED
#define CC1352RSTK_PWMPIN4               PIN_UNASSIGNED
#define CC1352RSTK_PWMPIN5               PIN_UNASSIGNED
#define CC1352RSTK_PWMPIN6               PIN_UNASSIGNED
#define CC1352RSTK_PWMPIN7               PIN_UNASSIGNED

/* Sensors */
#define CC1352RSTK_MPU_INT               IOID_30
#define CC1352RSTK_TMP_RDY               IOID_25

/* SPI */
#define CC1352RSTK_SPI_FLASH_CS          IOID_20
#define CC1352RSTK_FLASH_CS_ON           0
#define CC1352RSTK_FLASH_CS_OFF          1

/* SPI Board */
#define CC1352RSTK_SPI0_MISO             IOID_8
#define CC1352RSTK_SPI0_MOSI             IOID_9
#define CC1352RSTK_SPI0_CLK              IOID_10
#define CC1352RSTK_SPI0_CSN              IOID_11
#define CC1352RSTK_SPI1_MISO             PIN_UNASSIGNED
#define CC1352RSTK_SPI1_MOSI             PIN_UNASSIGNED
#define CC1352RSTK_SPI1_CLK              PIN_UNASSIGNED
#define CC1352RSTK_SPI1_CSN              PIN_UNASSIGNED

/* UART */
#define CC1352RSTK_UART_TX               IOID_13
#define CC1352RSTK_UART_RX               IOID_12



/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC1352RSTK_initGeneral(void);

/*!
 *  @brief  Turn off the external flash on LaunchPads
 *
 */
void CC1352RSTK_shutDownExtFlash(void);

/*!
 *  @brief  Wake up the external flash present on the board files
 *
 *  This function toggles the chip select for the amount of time needed
 *  to wake the chip up.
 */
void CC1352RSTK_wakeUpExtFlash(void);

/*!
 *  @def    CC1352RSTK_ADCBufName
 *  @brief  Enum of ADCBufs
 */
typedef enum CC1352RSTK_ADCBufName {
    CC1352RSTK_ADCBUF0 = 0,

    CC1352RSTK_ADCBUFCOUNT
} CC1352RSTK_ADCBufName;

/*!
 *  @def    CC1352RSTK_ADCBuf0ChannelName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC1352RSTK_ADCBuf0ChannelName {
    CC1352RSTK_ADCBUF0CHANNEL0 = 0,
    CC1352RSTK_ADCBUF0CHANNEL1,
    CC1352RSTK_ADCBUF0CHANNEL2,
    CC1352RSTK_ADCBUF0CHANNEL3,
    CC1352RSTK_ADCBUF0CHANNEL4,
    CC1352RSTK_ADCBUF0CHANNEL5,
    CC1352RSTK_ADCBUF0CHANNEL6,
    CC1352RSTK_ADCBUF0CHANNEL7,
    CC1352RSTK_ADCBUF0CHANNELVDDS,
    CC1352RSTK_ADCBUF0CHANNELDCOUPL,
    CC1352RSTK_ADCBUF0CHANNELVSS,

    CC1352RSTK_ADCBUF0CHANNELCOUNT
} CC1352RSTK_ADCBuf0ChannelName;

/*!
 *  @def    CC1352RSTK_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC1352RSTK_ADCName {
    CC1352RSTK_ADC0 = 0,
    CC1352RSTK_ADC1,
    CC1352RSTK_ADC2,
    CC1352RSTK_ADC3,
    CC1352RSTK_ADC4,
    CC1352RSTK_ADC5,
    CC1352RSTK_ADC6,
    CC1352RSTK_ADCDCOUPL,
    CC1352RSTK_ADCVSS,
    CC1352RSTK_ADCVDDS,

    CC1352RSTK_ADCCOUNT
} CC1352RSTK_ADCName;

/*!
 *  @def    CC1352R1_LAUNCHXL_ECDHName
 *  @brief  Enum of ECDH names
 */
typedef enum CC1352RSTK_ECDHName {
    CC1352RSTK_ECDH0 = 0,

    CC1352RSTK_ECDHCOUNT
} CC1352RSTK_ECDHName;

/*!
 *  @def    CC1352R1_LAUNCHXL_ECDSAName
 *  @brief  Enum of ECDSA names
 */
typedef enum CC1352RSTK_ECDSAName {
    CC1352RSTK_ECDSA0 = 0,

    CC1352RSTK_ECDSACOUNT
} CC1352RSTK_ECDSAName;

/*!
 *  @def    CC1352R1_LAUNCHXL_ECJPAKEName
 *  @brief  Enum of ECJPAKE names
 */
typedef enum CC1352RSTK_ECJPAKEName {
    CC1352RSTK_ECJPAKE0 = 0,

    CC1352RSTK_ECJPAKECOUNT
} CC1352RSTK_ECJPAKEName;

/*!
 *  @def    CC1352R1_LAUNCHXL_SHA2Name
 *  @brief  Enum of SHA2 names
 */
typedef enum CC1352RSTK_SHA2Name {
    CC1352RSTK_SHA20 = 0,

    CC1352RSTK_SHA2COUNT
} CC1352RSTK_SHA2Name;


/*!
 *  @def    CC1352RSTK_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC1352RSTK_CryptoName {
    CC1352RSTK_CRYPTO0 = 0,

    CC1352RSTK_CRYPTOCOUNT
} CC1352RSTK_CryptoName;

/*!
 *  @def    CC1352RSTK_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum CC1352RSTK_AESCCMName {
    CC1352RSTK_AESCCM0 = 0,

    CC1352RSTK_AESCCMCOUNT
} CC1352RSTK_AESCCMName;

/*!
 *  @def    CC1352RSTK_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum CC1352RSTK_AESGCMName {
    CC1352RSTK_AESGCM0 = 0,

    CC1352RSTK_AESGCMCOUNT
} CC1352RSTK_AESGCMName;

/*!
 *  @def    CC1352RSTK_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum CC1352RSTK_AESCBCName {
    CC1352RSTK_AESCBC0 = 0,

    CC1352RSTK_AESCBCCOUNT
} CC1352RSTK_AESCBCName;

/*!
 *  @def    CC1352RSTK_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum CC1352RSTK_AESCTRName {
    CC1352RSTK_AESCTR0 = 0,

    CC1352RSTK_AESCTRCOUNT
} CC1352RSTK_AESCTRName;

/*!
 *  @def    CC1352RSTK_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum CC1352RSTK_AESECBName {
    CC1352RSTK_AESECB0 = 0,

    CC1352RSTK_AESECBCOUNT
} CC1352RSTK_AESECBName;

/*!
 *  @def    CC1352RSTK_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum CC1352RSTK_AESCTRDRBGName {
    CC1352RSTK_AESCTRDRBG0 = 0,

    CC1352RSTK_AESCTRDRBGCOUNT
} CC1352RSTK_AESCTRDRBGName;

/*!
 *  @def    CC1352RSTK_TRNGName
 *  @brief  Enum of TRNG names
 */
typedef enum CC1352RSTK_TRNGName {
    CC1352RSTK_TRNG0 = 0,

    CC1352RSTK_TRNGCOUNT
} CC1352RSTK_TRNGName;

/*!
 *  @def    CC1352RSTK_GPIOName
 *  @brief  Enum of GPIO names
 */
typedef enum CC1352RSTK_GPIOName {
    CC1352RSTK_GPIO_S1 = 0,
    CC1352RSTK_GPIO_S2,
    CC1352RSTK_GPIO_LED0,
    CC1352RSTK_GPIO_LED1,
    CC1352RSTK_GPIO_LED2,
    CC1352RSTK_GPIO_SPI_FLASH_CS,
    CC1352RSTK_GPIO_CAPTOUCH1,
    CC1352RSTK_GPIO_CAPTOUCH2,

    CC1352RSTK_GPIOCOUNT
} CC1352RSTK_GPIOName;

/*!
 *  @def    CC1352RSTK_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC1352RSTK_GPTimerName {
    CC1352RSTK_GPTIMER0A = 0,
    CC1352RSTK_GPTIMER0B,
    CC1352RSTK_GPTIMER1A,
    CC1352RSTK_GPTIMER1B,
    CC1352RSTK_GPTIMER2A,
    CC1352RSTK_GPTIMER2B,
    CC1352RSTK_GPTIMER3A,
    CC1352RSTK_GPTIMER3B,

    CC1352RSTK_GPTIMERPARTSCOUNT
} CC1352RSTK_GPTimerName;

/*!
 *  @def    CC1352RSTK_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC1352RSTK_GPTimers {
    CC1352RSTK_GPTIMER0 = 0,
    CC1352RSTK_GPTIMER1,
    CC1352RSTK_GPTIMER2,
    CC1352RSTK_GPTIMER3,

    CC1352RSTK_GPTIMERCOUNT
} CC1352RSTK_GPTimers;

/*!
 *  @def    CC1352RSTK_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC1352RSTK_I2CName {
#if TI_I2C_CONF_I2C0_ENABLE
    CC1352RSTK_I2C0 = 0,
#endif

    CC1352RSTK_I2CCOUNT
} CC1352RSTK_I2CName;

/*!
 *  @def    CC1352RSTK_I2SName
 *  @brief  Enum of I2S names
 */
typedef enum CC1352RSTK_I2SName {
    CC1352RSTK_I2S0 = 0,

    CC1352RSTK_I2SCOUNT
} CC1352RSTK_I2SName;

/*!
 *  @def    CC1352RSTK_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC1352RSTK_NVSName {
#if TI_NVS_CONF_NVS_INTERNAL_ENABLE
    CC1352RSTK_NVSCC26XX0 = 0,
#endif
#if TI_NVS_CONF_NVS_EXTERNAL_ENABLE
    CC1352RSTK_NVSSPI25X0,
#endif

    CC1352RSTK_NVSCOUNT
} CC1352RSTK_NVSName;

/*!
 *  @def    CC1352RSTK_PWMName
 *  @brief  Enum of PWM outputs
 */
typedef enum CC1352RSTK_PWMName {
    CC1352RSTK_PWM0 = 0,
    CC1352RSTK_PWM1,
    CC1352RSTK_PWM2,
    CC1352RSTK_PWM3,
    CC1352RSTK_PWM4,
    CC1352RSTK_PWM5,
    CC1352RSTK_PWM6,
    CC1352RSTK_PWM7,

    CC1352RSTK_PWMCOUNT
} CC1352RSTK_PWMName;

/*!
 *  @def    CC1352RSTK_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC1352RSTK_SPIName {
#if TI_SPI_CONF_SPI0_ENABLE
    CC1352RSTK_SPI0 = 0,
#endif
#if TI_SPI_CONF_SPI1_ENABLE
    CC1352RSTK_SPI1,
#endif

    CC1352RSTK_SPICOUNT
} CC1352RSTK_SPIName;

/*!
 *  @def    CC1352RSTK_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC1352RSTK_UARTName {
#if TI_UART_CONF_UART0_ENABLE
    CC1352RSTK_UART0 = 0,
#endif

    CC1352RSTK_UARTCOUNT
} CC1352RSTK_UARTName;

/*!
 *  @def    CC1352RSTK_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1352RSTK_UDMAName {
    CC1352RSTK_UDMA0 = 0,

    CC1352RSTK_UDMACOUNT
} CC1352RSTK_UDMAName;

/*!
 *  @def    CC1352RSTK_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC1352RSTK_WatchdogName {
    CC1352RSTK_WATCHDOG0 = 0,

    CC1352RSTK_WATCHDOGCOUNT
} CC1352RSTK_WatchdogName;

#ifdef __cplusplus
}
#endif

#endif /* __CC1352RSTK_BOARD_H__ */
