// Copyright 2022, Adan Lema <adanlema@hotmail.com>

/*==================[inclusions]=============================================*/
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "hal.h"

#include "al_gpio.h"
#include "al_bsp.h"
#include "al_adc.h"
#include "al_bluepill.h"
#include "al_pwm.h"
#include "al_uart.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static struct board_s board = {0};
/*==================[internal functions declaration]=========================*/
static void config_relojext_init(void);
/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void config_relojext_init(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        continue;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSE;
    while (!((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_0))
        continue;
    RCC->CFGR &= ~(1 << 7);
    RCC->CFGR &= ~(1 << 10);
    RCC->CFGR &= ~(1 << 13);
    SystemCoreClockUpdate();
}

/*==================[external functions definition]==========================*/
board_t board_Create(void) {

    /*  Configuracion de pines*/
    config_relojext_init();
    ADCConvertInit();
    PWM_Init();
    UART_Init();
    SysTick_Config(SystemCoreClock / 1000);
    /*  Entradas  */
    board.boton = DigitalInput_Create(BOTON_PUERTO, BOTON_PIN, true);
    /*  Salidas  */
    board.led    = DigitalOutput_Create(LED_PUERTO, LED_PIN, true);
    board.salida = DigitalOutput_Create(SAL_PUERTO, SAL_PIN, false);
    return &board;
}
/**  doxygen end group definition */
/*==================[end of file]============================================*/