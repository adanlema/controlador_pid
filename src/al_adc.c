// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "stm32f1xx_hal.h"
#include "hal.h"
#include "al_adc.h"
#include "al_bluepill.h"
#include <stdint.h>
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void ADCConvertInit() {
    // Habilitar el reloj para el ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Pin PA0 como entrada analogica - PA0 es el canal1 del ADC1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~((GPIO_MOD_MASK | GPIO_CNF_MASK) << (ADC_PIN * 4));
    GPIOA->CRL |= ((GPIO_MOD_INP | GPIO_CNF_INP_ANG) << (ADC_PIN * 4));

    // Configurar el canal de entrada del ADC (PA0)
    ADC1->SQR3 = 0;
    // Configuracion para una Fs = 20 [kHz] aproximadamente
    ADC1->SMPR2 = ADC_SMPR2_SMP0;
    // Habilitar el ADC
    ADC1->CR2 = ADC_CR2_ADON;
}
void ADC_StartConvert() {
    ADC1->CR2 |= ADC_CR2_ADON;
    while (!(ADC1->SR & ADC_SR_EOC))
        continue;
}
uint16_t ADC_GetValue() {
    return ADC1->DR;
}

/**  doxygen end group definition */
/*==================[end of file]============================================*/