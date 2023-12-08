// Copyright 2023, Adan Lema <adanlema@hotmail.com>
/*==================[inclusions]=============================================*/
#include "stm32f1xx_hal.h"
#include "hal.h"
#include "al_bluepill.h"
#include "al_uart.h"
#include <stdint.h>
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void UART_Init(void) {
    // Habilitar el reloj para la UART1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Configurar los pines TX (PA9) y RX (PA10)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRH &= ~((GPIO_MOD_MASK | GPIO_CNF_MASK) << ((TX_PIN - 8) * 4));
    GPIOA->CRH |= ((GPIO_MOD_OUT_2M | GPIO_CNF_OUT_APP) << ((TX_PIN - 8) * 4));
    GPIOA->CRH &= ~((GPIO_MOD_MASK | GPIO_CNF_MASK) << ((RX_PIN - 8) * 4));
    GPIOA->CRH |= ((GPIO_MOD_INP | GPIO_CNF_INP_PPL) << ((RX_PIN - 8) * 4));

    USART1->BRR = SystemCoreClock / UART_SPEED;

    // Habilitar la transmisión y recepción, habilitar la UART
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART_Transmit(uint8_t * data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        while (!(USART1->SR & USART_SR_TXE))
            ;                 // Esperar a que el registro de transmisión esté vacío
        USART1->DR = data[i]; // Cargar el dato a transmitir
    }
}

void UART_Receive(uint8_t * data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        while (!(USART1->SR & USART_SR_RXNE))
            ;                 // Esperar a que el registro de recepción esté lleno
        data[i] = USART1->DR; // Leer el dato recibido
    }
}
/**  doxygen end group definition */
/*==================[end of file]============================================*/