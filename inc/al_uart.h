/* Copyright 2023, Adan Lema <adanlema@hotmail.com> */
#ifndef AL_UART_H
#define AL_UART_H

/*==================[inclusions]=============================================*/
#include <stdint.h>
/*==================[macros]=================================================*/
#define UART_SPEED 9600 // baudios
/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
void UART_Init(void);
void UART_Transmit(uint8_t * data, uint16_t size);
void UART_Receive(uint8_t * data, uint16_t size);
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif