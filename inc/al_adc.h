/* Copyright 2023, Adan Lema <adanlema@hotmail.com> */
#ifndef AL_ADC_H
#define AL_ADC_H

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "hal.h"
/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
void     ADCConvertInit();
void     ADC_StartConvert();
uint16_t ADC_GetValue();

/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif