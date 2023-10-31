// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "stm32f1xx_hal.h"
#include "hal.h"
#include <stdint.h>
#include "al_pwm.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void PWM_Init() {
    // Habilitar el reloj para TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // Configurar TIM2
    TIM2->CR1 = 0;             // Deshabilitar temporizador
    TIM2->PSC = 0;             // Sin preescaler
    TIM2->ARR = PWM_PERIOD;    // Valor de recarga (período)
    TIM2->CR1 |= TIM_CR1_ARPE; // Habilitar registro de recarga automática
    TIM2->EGR = TIM_EGR_UG;    // Generar evento de actualización
    // Configurar canal 1 de TIM2 para PWM
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // Modo PWM1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;           // Habilitar registro de preload
    TIM2->CCER |= TIM_CCER_CC1E;              // Habilitar salida del canal 1
    // Habilitar el temporizador TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
}
void PWM_ChangeDuty(uint32_t duty) {
    TIM2->CCR1 = duty;
}
/**  doxygen end group definition */
/*==================[end of file]============================================*/
