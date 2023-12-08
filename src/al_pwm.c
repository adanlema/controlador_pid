// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "stm32f1xx_hal.h"
#include "hal.h"
#include <stdint.h>
#include "al_pwm.h"
#include "al_bluepill.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void PWM_Init(void) {
    // Habilitar el reloj para TIM3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Pin PA6 como salida alternativa en modo push-pull - PA6 es el canal1 del TIM3
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~((GPIO_MOD_MASK | GPIO_CNF_MASK) << (PWM_PIN * 4));
    GPIOA->CRL |= ((GPIO_MOD_OUT_2M | GPIO_CNF_OUT_APP) << (PWM_PIN * 4));

    // Configura TIM3
    TIM3->PSC = 0;                                      // Sin preescalador
    TIM3->ARR = PWM_PERIOD;                             // Periodo
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Modo PWM1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;                     // Habilita preload para el canal 1
    TIM3->CCER |= TIM_CCER_CC1E;                        // Habilita la salida para el canal 1
    TIM3->CR1 |= TIM_CR1_ARPE;                          // Habilita preload en el contador

    // Habilitar TIM3
    TIM3->CR1 |= TIM_CR1_CEN;
}
void PWM_ChangeDuty(uint32_t duty) {
    if (duty <= PWM_PERIOD) {
        TIM3->CCR1 = duty;
    } else {
        TIM3->CCR1 = PWM_PERIOD;
    }
}
/**  doxygen end group definition */
/*==================[end of file]============================================*/
