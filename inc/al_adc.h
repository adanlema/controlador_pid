/* Copyright 2023, Adan Lema <adanlema@hotmail.com> */
#ifndef AL_ADC_H
#define AL_ADC_H

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "hal.h"
/*==================[macros]=================================================*/
//! Maximo valor que soporta el ADC
#define ADC_MAX_VALOR 4095
/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/**
 * @brief Inicializar el ADC
 * Funcion que sirve para configurar los registros del ADC1. Se utilizara el pin 0 del GPIOA.
 */
void ADCConvertInit();
/**
 * @brief Captura de datos
 * Funcion que sirve para activar la captura de datos del ADC por el PA0.
 */
void ADC_StartConvert();
/**
 * @brief Obtencion del valor capturado.
 * Retorna el valor del ADC capturado.
 * @return uint16_t
 */
uint16_t ADC_GetValue();

/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif