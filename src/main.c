// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "al_gpio.h"
#include "al_bsp.h"
#include "al_bluepill.h"
#include "al_adc.h"
#include "al_pwm.h"
#include "al_uart.h"

#include <stdio.h>
#include <stdint.h>
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
static uint32_t msTicks = 0;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
int main(void) {

    board_t  bp          = board_Create();
    uint32_t cambio_duty = 0;
    uint8_t  num_datos   = 0;
    char     txData[50];

    DigitalOutput_Desactivate(bp->led);
    while (true) {
        ADC_StartConvert();
        uint16_t adcValue = ADC_GetValue();

        /* Cambiar el dutty del PWM */
        cambio_duty = (adcValue * PWM_PERIOD) / ADC_MAX_VALOR;
        PWM_ChangeDuty(cambio_duty);

        /* Transmitimos por la UART1 cada 1seg */
        if (msTicks >= 1000) {
            msTicks          = 0;
            uint16_t voltaje = (adcValue * 3300) / ADC_MAX_VALOR; // 3300mV

            if (num_datos == 0) {
                uint8_t inicio[] = "\r\nNumero de Muestras  /   Datos Obtenidos\r\n";
                UART_Transmit(inicio, sizeof(inicio));
            }
            num_datos++;
            if (num_datos <= 30) {
                size_t len =
                    snprintf(txData, sizeof(txData), "N%u : %u [mV]\r\n", num_datos, voltaje);
                UART_Transmit((uint8_t *)txData, len);
            } else {
                num_datos = 0;
            }
        }
    }
}

void SysTick_Handler(void) {
    msTicks++;
}

/**  doxygen end group definition */
/*==================[end of file]============================================*/