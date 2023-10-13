// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "al_gpio.h"
#include "al_bsp.h"
#include "al_bluepill.h"
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
        if (DigitalInput_GetState(bp->boton)) {
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