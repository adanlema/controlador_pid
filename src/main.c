// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "al_gpio.h"
#include "al_bsp.h"
#include "al_bluepill.h"
#include "al_adc.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
// static uint32_t msTicks = 0;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
int main(void) {

    board_t bp = board_Create();
    while (true) {
        ADC_StartConvert();
        uint16_t adcValue = ADC_GetValue();
        if (adcValue > 2048) {
            DigitalOutput_Activate(bp->led);
        } else {
            DigitalOutput_Desactivate(bp->led);
        }
    }
}
/*
void SysTick_Handler(void) {
    // msTicks++;
}
*/
/**  doxygen end group definition */
/*==================[end of file]============================================*/