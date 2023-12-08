// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "al_gpio.h"
#include "al_bsp.h"
#include "al_bluepill.h"
#include "al_adc.h"
#include "al_pwm.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
// static uint32_t msTicks = 0;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
int main(void) {

    board_t  bp          = board_Create();
    uint32_t cambio_duty = 0;
    while (true) {
        ADC_StartConvert();
        uint16_t adcValue = ADC_GetValue();

        // Prender el LED
        if (adcValue > 2048) {
            DigitalOutput_Activate(bp->led);
        } else {
            DigitalOutput_Desactivate(bp->led);
        }
        // Cambiar el dutty del PWM
        cambio_duty = (adcValue * PWM_PERIOD) / ADC_MAX_VALOR;
        PWM_ChangeDuty(cambio_duty);
    }
}
/*
void SysTick_Handler(void) {
    // msTicks++;
}
*/
/**  doxygen end group definition */
/*==================[end of file]============================================*/