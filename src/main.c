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
#define TS                10 // [ms]
#define TON               10 // Ciclos
#define T2                50
#define TOFF              90 // Ciclos
#define CANTIDAD_MUESTRAS 100
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
static uint32_t msTicks        = 0;
static bool     bandera_tx     = false;
static bool     bandera_finish = false;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
int main(void) {

    char     txData[50];
    uint8_t  num_datos                           = 0;
    uint32_t cambio_duty                         = 0;
    uint8_t  valor_entrada[CANTIDAD_MUESTRAS]    = {0};
    uint16_t valor_muestreado[CANTIDAD_MUESTRAS] = {0};
    board_t  bp                                  = board_Create();

    DigitalOutput_Desactivate(bp->led);
    while (true) {
        ADC_StartConvert();
        uint16_t adcValue = ADC_GetValue();
        uint16_t voltaje  = (adcValue * 3300) / ADC_MAX_VALOR; // 3300mV
        uint8_t  entrada  = (uint8_t)DigitalOutput_GetState(bp->salida);

        //========================================================================================
        // Unico Pulso al 100%
        // if (DigitalInput_HasActivate(bp->boton)) {
        //     bandera_tx     = true;
        //     bandera_finish = false;
        //     num_datos      = 0;
        //     msTicks        = 0;
        //     DigitalOutput_Desactivate(bp->salida);
        // }

        // if (msTicks >= TS) {
        //     msTicks = 0;
        //     /* Se genera el escalon de encendido y apagado */
        //     if (num_datos == TON) {
        //         DigitalOutput_Activate(bp->salida);
        //     }
        //     if (num_datos == TOFF) {
        //         DigitalOutput_Desactivate(bp->salida);
        //     }
        //     /* Guardamos las muestras en los instantes de tiempo deseados */
        //     if (num_datos < CANTIDAD_MUESTRAS) {
        //         valor_entrada[num_datos]    = entrada;
        //         valor_muestreado[num_datos] = voltaje;
        //         num_datos++;
        //     } else {
        //         num_datos      = 0;
        //         bandera_tx     = false;
        //         bandera_finish = true;
        //     }
        // }
        //========================================================================================
        // Pulso PWM al 0%, 75%, 50%, 0%
        if (DigitalInput_HasActivate(bp->boton)) {
            bandera_tx     = true;
            bandera_finish = false;
            num_datos      = 0;
            msTicks        = 0;
            cambio_duty    = 0;
            PWM_ChangeDuty(cambio_duty);
        }
        if (msTicks >= TS) {
            msTicks = 0;

            /* Se genera el escalon de encendido y apagado */
            if (num_datos == TON) {
                cambio_duty = (3 * PWM_PERIOD) / 4;
                PWM_ChangeDuty(cambio_duty);
            }
            if (num_datos == T2) {
                cambio_duty = PWM_PERIOD / 2;
                PWM_ChangeDuty(cambio_duty);
            }
            if (num_datos == TOFF) {
                cambio_duty = 0;
                PWM_ChangeDuty(cambio_duty);
            }

            /* Guardamos las muestras en los instantes de tiempo deseados */
            if (num_datos < CANTIDAD_MUESTRAS) {
                valor_entrada[num_datos]    = entrada;
                valor_muestreado[num_datos] = voltaje;
                num_datos++;
            } else {
                num_datos      = 0;
                bandera_tx     = false;
                bandera_finish = true;
            }
        }
        //========================================================================================
        if (bandera_finish) {
            uint8_t inicio[] =
                "\r\nNumero de Muestras / Datos Obtenidos [mV] / Entrada / TS = 10[ms] \r\n";
            UART_Transmit(inicio, sizeof(inicio));

            for (int i = 0; i < CANTIDAD_MUESTRAS; i++) {
                size_t len = snprintf(txData, sizeof(txData), "N%u - %u - %u\r\n", i,
                                      valor_muestreado[i], valor_entrada[i]);
                UART_Transmit((uint8_t *)txData, len);
            }
            bandera_finish = false;
        }
    }
}

void SysTick_Handler(void) {
    if (bandera_tx) {
        msTicks++;
    }
}

/**  doxygen end group definition */
/*==================[end of file]============================================*/