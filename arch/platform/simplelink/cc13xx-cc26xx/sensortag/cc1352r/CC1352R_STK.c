/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
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

/*
 *  ============================ CC1352RSTK.c ============================
 *  This file is responsible for setting up the board specific items for the
 *  CC1352RSTK board.
 */

#include "CC1352R_STK.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//#include <ti/drivers/Power.h>
//#include <ti/drivers/power/PowerCC26XX.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/udma.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)


/*
 *  =============================== ADCBuf ===============================
 */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26X2.h>

ADCBufCC26X2_Object adcBufCC26X2objects[CC1352RSTK_ADCBUFCOUNT];

/*
 *  This table converts a virtual adc channel into a dio and internal analogue
 *  input signal. This table is necessary for the functioning of the adcBuf
 *  driver. Comment out unused entries to save flash. Dio and internal signal
 *  pairs are hardwired. Do not remap them in the table. You may reorder entire
 *  entries. The mapping of dio and internal signals is package dependent.
 */
const ADCBufCC26X2_AdcChannelLutEntry ADCBufCC26X2_adcChannelLut[CC1352RSTK_ADCBUF0CHANNELCOUNT] = {
    {CC1352RSTK_DIO23_ANALOG, ADC_COMPB_IN_AUXIO7},
    {CC1352RSTK_DIO24_ANALOG, ADC_COMPB_IN_AUXIO6},
    {CC1352RSTK_DIO25_ANALOG, ADC_COMPB_IN_AUXIO5},
    {CC1352RSTK_DIO26_ANALOG, ADC_COMPB_IN_AUXIO4},
    {CC1352RSTK_DIO27_ANALOG, ADC_COMPB_IN_AUXIO3},
    {CC1352RSTK_DIO28_ANALOG, ADC_COMPB_IN_AUXIO2},
    {CC1352RSTK_DIO29_ANALOG, ADC_COMPB_IN_AUXIO1},
    {CC1352RSTK_DIO30_ANALOG, ADC_COMPB_IN_AUXIO0},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VDDS},
    {PIN_UNASSIGNED, ADC_COMPB_IN_DCOUPL},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VSS},
};

const ADCBufCC26X2_HWAttrs adcBufCC26X2HWAttrs[CC1352RSTK_ADCBUFCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .adcChannelLut     = ADCBufCC26X2_adcChannelLut
    }
};

const ADCBuf_Config ADCBuf_config[CC1352RSTK_ADCBUFCOUNT] = {
    {
        &ADCBufCC26X2_fxnTable,
        &adcBufCC26X2objects[CC1352RSTK_ADCBUF0],
        &adcBufCC26X2HWAttrs[CC1352RSTK_ADCBUF0]
    },
};

const uint_least8_t ADCBuf_count = CC1352RSTK_ADCBUFCOUNT;

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

ADCCC26XX_Object adcCC26x2Objects[CC1352RSTK_ADCCOUNT];

const ADCCC26XX_HWAttrs adcCC26x2HWAttrs[CC1352RSTK_ADCCOUNT] = {
    {
        .adcDIO              = CC1352RSTK_DIO23_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO7,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1352RSTK_DIO24_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO6,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1352RSTK_DIO25_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO5,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1352RSTK_DIO26_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO4,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1352RSTK_DIO27_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO3,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1352RSTK_DIO28_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO2,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1352RSTK_DIO29_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO1,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_DCOUPL,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_DCOUPL,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VSS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    }
};

const ADC_Config ADC_config[CC1352RSTK_ADCCOUNT] = {
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADC0], &adcCC26x2HWAttrs[CC1352RSTK_ADC0]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADC1], &adcCC26x2HWAttrs[CC1352RSTK_ADC1]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADC2], &adcCC26x2HWAttrs[CC1352RSTK_ADC2]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADC3], &adcCC26x2HWAttrs[CC1352RSTK_ADC3]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADC4], &adcCC26x2HWAttrs[CC1352RSTK_ADC4]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADC5], &adcCC26x2HWAttrs[CC1352RSTK_ADC5]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADC6], &adcCC26x2HWAttrs[CC1352RSTK_ADC6]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADCDCOUPL], &adcCC26x2HWAttrs[CC1352RSTK_ADCDCOUPL]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADCVSS], &adcCC26x2HWAttrs[CC1352RSTK_ADCVSS]},
    {&ADCCC26XX_fxnTable, &adcCC26x2Objects[CC1352RSTK_ADCVDDS], &adcCC26x2HWAttrs[CC1352RSTK_ADCVDDS]},
};

const uint_least8_t ADC_count = CC1352RSTK_ADCCOUNT;




/*
 *  =============================== ECDH ===============================
 */
#include <ti/drivers/ECDH.h>
#include <ti/drivers/ecdh/ECDHCC26X2.h>

ECDHCC26X2_Object ecdhCC26X2Objects[CC1352RSTK_ECDHCOUNT];

const ECDHCC26X2_HWAttrs ecdhCC26X2HWAttrs[CC1352RSTK_ECDHCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const ECDH_Config ECDH_config[CC1352RSTK_ECDHCOUNT] = {
    {
         .object  = &ecdhCC26X2Objects[CC1352RSTK_ECDH0],
         .hwAttrs = &ecdhCC26X2HWAttrs[CC1352RSTK_ECDH0]
    },
};

const uint_least8_t ECDH_count = CC1352RSTK_ECDHCOUNT;

/*
 *  =============================== ECJPAKE ===============================
 */
#include <ti/drivers/ECJPAKE.h>
#include <ti/drivers/ecjpake/ECJPAKECC26X2.h>

ECJPAKECC26X2_Object ecjpakeCC26X2Objects[CC1352RSTK_ECJPAKECOUNT];

const ECJPAKECC26X2_HWAttrs ecjpakeCC26X2HWAttrs[CC1352RSTK_ECJPAKECOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const ECJPAKE_Config ECJPAKE_config[CC1352RSTK_ECJPAKECOUNT] = {
    {
         .object  = &ecjpakeCC26X2Objects[CC1352RSTK_ECJPAKE0],
         .hwAttrs = &ecjpakeCC26X2HWAttrs[CC1352RSTK_ECJPAKE0]
    },
};

const uint_least8_t ECJPAKE_count = CC1352RSTK_ECJPAKECOUNT;


/*
 *  =============================== SHA2 ===============================
 */
#include <ti/drivers/SHA2.h>
#include <ti/drivers/sha2/SHA2CC26X2.h>

SHA2CC26X2_Object sha2CC26X2Objects[CC1352RSTK_SHA2COUNT];

const SHA2CC26X2_HWAttrs sha2CC26X2HWAttrs[CC1352RSTK_SHA2COUNT] = {
    {
        .intPriority       = ~0,
    }
};

const SHA2_Config SHA2_config[CC1352RSTK_SHA2COUNT] = {
    {
         .object  = &sha2CC26X2Objects[CC1352RSTK_SHA20],
         .hwAttrs = &sha2CC26X2HWAttrs[CC1352RSTK_SHA20]
    },
};

const uint_least8_t SHA2_count = CC1352RSTK_SHA2COUNT;
/*
 *  =============================== ECDSA ===============================
 */
#include <ti/drivers/ECDSA.h>
#include <ti/drivers/ecdsa/ECDSACC26X2.h>

ECDSACC26X2_Object ecdsaCC26X2Objects[CC1352RSTK_ECDSACOUNT];

const ECDSACC26X2_HWAttrs ecdsaCC26X2HWAttrs[CC1352RSTK_ECDSACOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const ECDSA_Config ECDSA_config[CC1352RSTK_ECDSACOUNT] = {
    {
         .object  = &ecdsaCC26X2Objects[CC1352RSTK_ECDSA0],
         .hwAttrs = &ecdsaCC26X2HWAttrs[CC1352RSTK_ECDSA0]
    },
};

const uint_least8_t ECDSA_count = CC1352RSTK_ECDSACOUNT;


/*
 *  =============================== Crypto ===============================
 */
/*
#include <ti/drivers/crypto/CryptoCC26XX.h>

CryptoCC26XX_Object cryptoCC26XXObjects[CC1352RSTK_CRYPTOCOUNT];

const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC1352RSTK_CRYPTOCOUNT] = {
    {
        .baseAddr       = CRYPTO_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_CRYPTO,
        .intNum         = INT_CRYPTO_RESULT_AVAIL_IRQ,
        .intPriority    = ~0,
    }
};

const CryptoCC26XX_Config CryptoCC26XX_config[CC1352RSTK_CRYPTOCOUNT] = {
    {
         .object  = &cryptoCC26XXObjects[CC1352RSTK_CRYPTO0],
         .hwAttrs = &cryptoCC26XXHWAttrs[CC1352RSTK_CRYPTO0]
    },
};
*/
/*
 *  =============================== AESCCM ===============================
 */
#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

AESCCMCC26XX_Object aesccmCC26XXObjects[CC1352RSTK_AESCCMCOUNT];

const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[CC1352RSTK_AESCCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCCM_Config AESCCM_config[CC1352RSTK_AESCCMCOUNT] = {
    {
         .object  = &aesccmCC26XXObjects[CC1352RSTK_AESCCM0],
         .hwAttrs = &aesccmCC26XXHWAttrs[CC1352RSTK_AESCCM0]
    },
};

const uint_least8_t AESCCM_count = CC1352RSTK_AESCCMCOUNT;


/*
 *  =============================== AESGCM ===============================
 */
#include <ti/drivers/AESGCM.h>
#include <ti/drivers/aesgcm/AESGCMCC26XX.h>

AESGCMCC26XX_Object aesgcmCC26XXObjects[CC1352RSTK_AESGCMCOUNT];

const AESGCMCC26XX_HWAttrs aesgcmCC26XXHWAttrs[CC1352RSTK_AESGCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESGCM_Config AESGCM_config[CC1352RSTK_AESGCMCOUNT] = {
    {
         .object  = &aesgcmCC26XXObjects[CC1352RSTK_AESGCM0],
         .hwAttrs = &aesgcmCC26XXHWAttrs[CC1352RSTK_AESGCM0]
    },
};

const uint_least8_t AESGCM_count = CC1352RSTK_AESGCMCOUNT;

/*
 *  =============================== AESCBC ===============================
 */
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/aescbc/AESCBCCC26XX.h>

AESCBCCC26XX_Object aescbcCC26XXObjects[CC1352RSTK_AESCBCCOUNT];

const AESCBCCC26XX_HWAttrs aescbcCC26XXHWAttrs[CC1352RSTK_AESCBCCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCBC_Config AESCBC_config[CC1352RSTK_AESCBCCOUNT] = {
    {
         .object  = &aescbcCC26XXObjects[CC1352RSTK_AESCBC0],
         .hwAttrs = &aescbcCC26XXHWAttrs[CC1352RSTK_AESCBC0]
    },
};

const uint_least8_t AESCBC_count = CC1352RSTK_AESCBCCOUNT;

/*
 *  =============================== AESCTR ===============================
 */
#include <ti/drivers/AESCTR.h>
#include <ti/drivers/aesctr/AESCTRCC26XX.h>

AESCTRCC26XX_Object aesctrCC26XXObjects[CC1352RSTK_AESCTRCOUNT];

const AESCTRCC26XX_HWAttrs aesctrCC26XXHWAttrs[CC1352RSTK_AESCTRCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCTR_Config AESCTR_config[CC1352RSTK_AESCTRCOUNT] = {
    {
         .object  = &aesctrCC26XXObjects[CC1352RSTK_AESCTR0],
         .hwAttrs = &aesctrCC26XXHWAttrs[CC1352RSTK_AESCTR0]
    },
};

const uint_least8_t AESCTR_count = CC1352RSTK_AESCTRCOUNT;

/*
 *  =============================== AESECB ===============================
 */
#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

AESECBCC26XX_Object aesecbCC26XXObjects[CC1352RSTK_AESECBCOUNT];

const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[CC1352RSTK_AESECBCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESECB_Config AESECB_config[CC1352RSTK_AESECBCOUNT] = {
    {
         .object  = &aesecbCC26XXObjects[CC1352RSTK_AESECB0],
         .hwAttrs = &aesecbCC26XXHWAttrs[CC1352RSTK_AESECB0]
    },
};

const uint_least8_t AESECB_count = CC1352RSTK_AESECBCOUNT;

/*
 *  =============================== AESCTRDRBG ===============================
 */
#include <ti/drivers/AESCTRDRBG.h>
#include <ti/drivers/aesctrdrbg/AESCTRDRBGXX.h>

AESCTRDRBGXX_Object aesctrdrbgXXObjects[CC1352RSTK_AESCTRDRBGCOUNT];

const AESCTRDRBGXX_HWAttrs aesctrdrbgXXHWAttrs[CC1352RSTK_AESCTRDRBGCOUNT] = {
    {
        .aesctrIndex       = CC1352RSTK_AESCTR0,
    }
};

const AESCTRDRBG_Config AESCTRDRBG_config[CC1352RSTK_AESCTRDRBGCOUNT] = {
    {
         .object  = &aesctrdrbgXXObjects[CC1352RSTK_AESCTRDRBG0],
         .hwAttrs = &aesctrdrbgXXHWAttrs[CC1352RSTK_AESCTRDRBG0]
    },
};

const uint_least8_t AESCTRDRBG_count = CC1352RSTK_AESCTRDRBGCOUNT;

/*
 *  =============================== TRNG ===============================
 */
#include <ti/drivers/TRNG.h>
#include <ti/drivers/trng/TRNGCC26XX.h>

TRNGCC26XX_Object trngCC26XXObjects[CC1352RSTK_TRNGCOUNT];

const TRNGCC26XX_HWAttrs trngCC26X2HWAttrs[CC1352RSTK_TRNGCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .samplesPerCycle   = 240000,
    }
};

const TRNG_Config TRNG_config[CC1352RSTK_TRNGCOUNT] = {
    {
         .object  = &trngCC26XXObjects[CC1352RSTK_TRNG0],
         .hwAttrs = &trngCC26X2HWAttrs[CC1352RSTK_TRNG0]
    },
};

const uint_least8_t TRNG_count = CC1352RSTK_TRNGCOUNT;

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1352RSTK.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array. Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {

    /* Input pins */
    GPIOCC26XX_DIO_15 | GPIO_DO_NOT_CONFIG,  /* Button 0 */
    GPIOCC26XX_DIO_14 | GPIO_DO_NOT_CONFIG,  /* Button 1 */

    /* Output pins */
    GPIOCC26XX_DIO_21 | GPIO_DO_NOT_CONFIG,  /* LED */
    GPIOCC26XX_DIO_07 | GPIO_DO_NOT_CONFIG,  /* LED */
    GPIOCC26XX_DIO_06 | GPIO_DO_NOT_CONFIG,  /* LED */

    /* SPI Flash CSN */
    GPIOCC26XX_DIO_20 | GPIO_DO_NOT_CONFIG,

    /* CapTIvate Interrupts */
    GPIOCC26XX_DIO_22 | GPIO_DO_NOT_CONFIG,
    GPIOCC26XX_DIO_24 | GPIO_DO_NOT_CONFIG,
};





/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1352RSTK.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /*  Button 0 */
    NULL,  /*  Button 1 */
    NULL,  /*  LED 0 */
    NULL,  /*  LED 1 */
    NULL,  /*  LED 2 */
    NULL,   /* SPI */
    NULL,
    NULL


};

const GPIOCC26XX_Config GPIOCC26XX_config = {
    .pinConfigs         = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks          = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = CC1352RSTK_GPIOCOUNT,
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority        = (~0)
};

/*
 *  =============================== GPTimer ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/timer/GPTimerCC26XX.h>

GPTimerCC26XX_Object gptimerCC26XXObjects[CC1352RSTK_GPTIMERCOUNT];

const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMERPARTSCOUNT] = {
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, },
};

const GPTimerCC26XX_Config GPTimerCC26XX_config[CC1352RSTK_GPTIMERPARTSCOUNT] = {
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER0], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER0A], GPT_A },
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER0], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER0B], GPT_B },
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER1], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER1A], GPT_A },
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER1], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER1B], GPT_B },
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER2], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER2A], GPT_A },
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER2], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER2B], GPT_B },
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER3], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER3A], GPT_A },
    { &gptimerCC26XXObjects[CC1352RSTK_GPTIMER3], &gptimerCC26xxHWAttrs[CC1352RSTK_GPTIMER3B], GPT_B },
};

/*
 *  =============================== I2C ===============================
*/
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

#if TI_I2C_CONF_ENABLE

I2CCC26XX_Object i2cCC26xxObjects[CC1352RSTK_I2CCOUNT];

const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC1352RSTK_I2CCOUNT] = {
#if TI_I2C_CONF_I2C0_ENABLE
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = ~0,
        .swiPriority = 0,
        .sdaPin      = CC1352RSTK_I2C0_SDA0,
        .sclPin      = CC1352RSTK_I2C0_SCL0,
    },
#endif
};

const I2C_Config I2C_config[CC1352RSTK_I2CCOUNT] = {
#if TI_I2C_CONF_I2C0_ENABLE
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
        .object      = &i2cCC26xxObjects[CC1352RSTK_I2C0],
        .hwAttrs     = &i2cCC26xxHWAttrs[CC1352RSTK_I2C0]
    },
#endif
};

const uint_least8_t I2C_count = CC1352RSTK_I2CCOUNT;

#endif /* TI_I2C_CONF_ENABLE */

/*
 *  =============================== I2S ===============================
*/
/*
#include <ti/drivers/I2S.h>
#include <ti/drivers/i2s/I2SCC26XX.h>

I2SCC26XX_Object i2sCC26XXObjects[CC1352RSTK_I2SCOUNT];

const I2SCC26XX_HWAttrs i2sCC26XXHWAttrs[CC1352RSTK_I2SCOUNT] = {
    {
        .pinSD1      =  CC1352RSTK_I2S_ADI,
        .pinSD0      =  CC1352RSTK_I2S_ADO,
        .pinSCK      =  CC1352RSTK_I2S_BCLK,
        .pinMCLK     =  CC1352RSTK_I2S_MCLK,
        .pinWS       =  CC1352RSTK_I2S_WCLK,
        .intPriority = ~0,
    }
};

const I2S_Config I2S_config[CC1352RSTK_I2SCOUNT] = {
    {
        .object      = &i2sCC26XXObjects[CC1352RSTK_I2S0],
        .hwAttrs     = &i2sCC26XXHWAttrs[CC1352RSTK_I2S0]
    },
};

const uint_least8_t I2S_count = CC1352RSTK_I2SCOUNT;
*/
/*
 *  =============================== NVS ===============================
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSSPI25X.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

#define NVS_REGIONS_BASE 0x48000
#define SECTORSIZE       0x2000
#define REGIONSIZE       (SECTORSIZE * 4)

#define  TI_NVS_CONF_ENABLE 1


#if TI_NVS_CONF_ENABLE

#if TI_NVS_CONF_NVS_INTERNAL_ENABLE

/*
 * Reserve flash sectors for NVS driver use by placing an uninitialized byte
 * array at the desired flash address.
 */
#if defined(__TI_COMPILER_VERSION__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
#pragma LOCATION(flashBuf, NVS_REGIONS_BASE);
#pragma NOINIT(flashBuf);
static char flashBuf[REGIONSIZE];

#elif defined(__IAR_SYSTEMS_ICC__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
static __no_init char flashBuf[REGIONSIZE] @ NVS_REGIONS_BASE;

#elif defined(__GNUC__)

/*
 * Place the flash buffers in the .nvs section created in the gcc linker file.
 * The .nvs section enforces alignment on a sector boundary but may
 * be placed anywhere in flash memory.  If desired the .nvs section can be set
 * to a fixed address by changing the following in the gcc linker file:
 *
 * .nvs (FIXED_FLASH_ADDR) (NOLOAD) : AT (FIXED_FLASH_ADDR) {
 *      *(.nvs)
 * } > REGION_TEXT
 */
__attribute__ ((section (".nvs")))
static char flashBuf[REGIONSIZE];

#endif

/* Allocate objects for NVS Internal Regions */
NVSCC26XX_Object nvsCC26xxObjects[1];

/* Hardware attributes for NVS Internal Regions */
const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[1] = {
    {
        .regionBase = (void *)flashBuf,
        .regionSize = REGIONSIZE,
    },
};

#endif /* TI_NVS_CONF_NVS_INTERNAL_ENABLE */

#if TI_NVS_CONF_NVS_EXTERNAL_ENABLE

#define SPISECTORSIZE    0x1000
#define SPIREGIONSIZE    (SPISECTORSIZE * 32)
#define VERIFYBUFSIZE    64

static uint8_t verifyBuf[VERIFYBUFSIZE];

/* Allocate objects for NVS External Regions */
NVSSPI25X_Object nvsSPI25XObjects[1];

/* Hardware attributes for NVS External Regions */
const NVSSPI25X_HWAttrs nvsSPI25XHWAttrs[1] = {
    {
        .regionBaseOffset = 0,
        .regionSize = SPIREGIONSIZE,
        .sectorSize = SPISECTORSIZE,
        .verifyBuf = verifyBuf,
        .verifyBufSize = VERIFYBUFSIZE,
        .spiHandle = NULL,
        .spiIndex = 0,
        .spiBitRate = 4000000,
        .spiCsnGpioIndex = CC1352RSTK_GPIO_SPI_FLASH_CS,
        .statusPollDelayUs = 100,
    },
};

#endif /* Board_EXCLUDE_NVS_EXTERNAL_FLASH */

/* NVS Region index 0 and 1 refer to NVS and NVS SPI respectively */
const NVS_Config NVS_config[CC1352RSTK_NVSCOUNT] = {
#if TI_NVS_CONF_NVS_INTERNAL_ENABLE
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26xxObjects[0],
        .hwAttrs = &nvsCC26xxHWAttrs[0],
    },
#endif
#if TI_NVS_CONF_NVS_EXTERNAL_ENABLE
    {
        .fxnTablePtr = &NVSSPI25X_fxnTable,
        .object = &nvsSPI25XObjects[0],
        .hwAttrs = &nvsSPI25XHWAttrs[0],
    },
#endif
};

const uint_least8_t NVS_count = CC1352RSTK_NVSCOUNT;

#endif /* TI_NVS_CONF_ENABLE */

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

const PIN_Config BoardGpioInitTable[] = {

    CC1352RSTK_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,           /* LED initially off */
    CC1352RSTK_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,           /* LED initially off */
    CC1352RSTK_PIN_BLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,           /* LED initially off */
    CC1352RSTK_PIN_BTN1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,          /* Button is active low */
    CC1352RSTK_PIN_BTN2 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button is active low */
    CC1352STK_CAPTOUCH_INT1 | PIN_INPUT_EN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button is active low */
    CC1352STK_CAPTOUCH_INT2 | PIN_INPUT_EN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button is active low */
    CC1352RSTK_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,  /* External flash chip select */
    CC1352RSTK_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,                        /* DevPack */
    CC1352RSTK_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* DevPack */
    CC1352RSTK_SPI0_MOSI | PIN_INPUT_EN | PIN_PULLDOWN,                                            /* SPI master out - slave in */
    CC1352RSTK_SPI0_MISO | PIN_INPUT_EN | PIN_PULLDOWN,                                            /* SPI master in - slave out */
    CC1352RSTK_SPI0_CLK | PIN_INPUT_EN | PIN_PULLDOWN,                                             /* SPI clock */
   // CC1352RSTK_RELAY | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,           /* Relay is active high */
    CC1352RSTK_MPU_INT | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,           /* MPU_INT is active low */
    CC1352RSTK_TMP_RDY | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,                               /* TMP_RDY is active high */
    //CC1352RSTK_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* Buzzer initially off */
    //CC1352RSTK_MPU_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* MPU initially on */
    //CC1352RSTK_MIC_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,      /* MIC initially off */
    PIN_TERMINATE
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
*  =============================== Power ===============================
*/
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include "clock-arch.h"

const PowerCC26X2_Config PowerCC26X2_config = {
   .policyInitFxn      = NULL,
   .policyFxn          = &clock_arch_standby_policy,
   .calibrateFxn       = &PowerCC26XX_calibrate,
   .enablePolicy       = true,
   .calibrateRCOSC_LF  = true,
   .calibrateRCOSC_HF  = true,
};
/*
 *  =============================== PWM ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>

PWMTimerCC26XX_Object pwmtimerCC26xxObjects[CC1352RSTK_PWMCOUNT];

const PWMTimerCC26XX_HwAttrs pwmtimerCC26xxHWAttrs[CC1352RSTK_PWMCOUNT] = {
    { .pwmPin = CC1352RSTK_PWMPIN0, .gpTimerUnit = CC1352RSTK_GPTIMER0A },
    { .pwmPin = CC1352RSTK_PWMPIN1, .gpTimerUnit = CC1352RSTK_GPTIMER0B },
    { .pwmPin = CC1352RSTK_PWMPIN2, .gpTimerUnit = CC1352RSTK_GPTIMER1A },
    { .pwmPin = CC1352RSTK_PWMPIN3, .gpTimerUnit = CC1352RSTK_GPTIMER1B },
    { .pwmPin = CC1352RSTK_PWMPIN4, .gpTimerUnit = CC1352RSTK_GPTIMER2A },
    { .pwmPin = CC1352RSTK_PWMPIN5, .gpTimerUnit = CC1352RSTK_GPTIMER2B },
    { .pwmPin = CC1352RSTK_PWMPIN6, .gpTimerUnit = CC1352RSTK_GPTIMER3A },
    { .pwmPin = CC1352RSTK_PWMPIN7, .gpTimerUnit = CC1352RSTK_GPTIMER3B },
};

const PWM_Config PWM_config[CC1352RSTK_PWMCOUNT] = {
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM0], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM0] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM1], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM1] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM2], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM2] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM3], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM3] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM4], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM4] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM5], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM5] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM6], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM6] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1352RSTK_PWM7], &pwmtimerCC26xxHWAttrs[CC1352RSTK_PWM7] },
};

const uint_least8_t PWM_count = CC1352RSTK_PWMCOUNT;

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = ~0,       /* Lowest HWI priority */
    .swiPriority        = 0,        /* Lowest SWI priority */
    .xoscHfAlwaysNeeded = true,     /* Keep XOSC dependency while in stanby */
    .globalCallback     = NULL,     /* No board specific callback */
    .globalEventMask    = 0         /* No events subscribed to */
};

/*
 *  =============================== SPI DMA ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

#if TI_SPI_CONF_ENABLE

SPICC26XXDMA_Object spiCC26XXDMAObjects[CC1352RSTK_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to 0xFF
 * to satisfy the SDSPI driver requirement.
 */
const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC1352RSTK_SPICOUNT] = {
#if TI_SPI_CONF_SPI0_ENABLE
    {
        .baseAddr           = SSI0_BASE,
        .intNum             = INT_SSI0_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin            = CC1352RSTK_SPI0_MOSI,
        .misoPin            = CC1352RSTK_SPI0_MISO,
        .clkPin             = CC1352RSTK_SPI0_CLK,
        .csnPin             = CC1352RSTK_SPI0_CSN,
        .minDmaTransferSize = 10
    },
#endif
#if TI_SPI_CONF_SPI1_ENABLE
    {
        .baseAddr           = SSI1_BASE,
        .intNum             = INT_SSI1_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI1,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI1_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI1_TX,
        .mosiPin            = CC1352RSTK_SPI1_MOSI,
        .misoPin            = CC1352RSTK_SPI1_MISO,
        .clkPin             = CC1352RSTK_SPI1_CLK,
        .csnPin             = CC1352RSTK_SPI1_CSN,
        .minDmaTransferSize = 10
    },
#endif
};

const SPI_Config SPI_config[CC1352RSTK_SPICOUNT] = {
#if TI_SPI_CONF_SPI0_ENABLE
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC1352RSTK_SPI0],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC1352RSTK_SPI0]
    },
#endif
#if TI_SPI_CONF_SPI1_ENABLE
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC1352RSTK_SPI1],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC1352RSTK_SPI1]
    },
#endif
};

const uint_least8_t SPI_count = CC1352RSTK_SPICOUNT;

#endif /* TI_SPI_CONF_ENABLE */

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

#if TI_UART_CONF_ENABLE

UARTCC26XX_Object uartCC26XXObjects[CC1352RSTK_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[CC1352RSTK_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CC1352RSTK_UARTCOUNT] = {
#if TI_UART_CONF_UART0_ENABLE
    {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = CC1352RSTK_UART_TX,
        .rxPin          = CC1352RSTK_UART_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[CC1352RSTK_UART0],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[CC1352RSTK_UART0]),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
    },
#endif
};

const UART_Config UART_config[CC1352RSTK_UARTCOUNT] = {
#if TI_UART_CONF_UART0_ENABLE
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[CC1352RSTK_UART0],
        .hwAttrs     = &uartCC26XXHWAttrs[CC1352RSTK_UART0]
    },
#endif
};

const uint_least8_t UART_count = CC1352RSTK_UARTCOUNT;

#endif /* TI_UART_CONF_ENABLE */

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[CC1352RSTK_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[CC1352RSTK_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[CC1352RSTK_UDMACOUNT] = {
    {
         .object  = &udmaObjects[CC1352RSTK_UDMA0],
         .hwAttrs = &udmaHWAttrs[CC1352RSTK_UDMA0]
    },
};

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[CC1352RSTK_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[CC1352RSTK_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WDT_BASE,
        .reloadValue = 1000 /* Reload value in milliseconds */
    },
};

const Watchdog_Config Watchdog_config[CC1352RSTK_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object      = &watchdogCC26XXObjects[CC1352RSTK_WATCHDOG0],
        .hwAttrs     = &watchdogCC26XXHWAttrs[CC1352RSTK_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = CC1352RSTK_WATCHDOGCOUNT;

/*
 *  Board-specific initialization function to disable external flash.
 *  This function is defined in the file CC1352RSTK_fxns.c
 */
extern void Board_initHook(void);

/*
 *  ======== CC1352RSTK_initGeneral ========
 */
void CC1352RSTK_initGeneral(void)
{
    Power_init();

    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        while (1);
    }

    /* Perform board-specific initialization */
    Board_initHook();
}

/*
 *  ======== Board_init ========
 */
void Board_init(void)
{
    CC1352RSTK_initGeneral();
}