/* Copyright 2023, Adan Lema <adanlema@hotmail.com> */
#ifndef AL_UART_H
#define AL_UART_H

/*==================[inclusions]=============================================*/
#include <stdint.h>
/*==================[macros]=================================================*/
//! Velocidad de transmision/recepcion de la usart.
#define UART_SPEED 9600 // baudios
/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/**
 * @brief Inicializacion de la UART1
 * Esta funcion configura los registros para usar la UART1. Se empleara como transmisor y receptor
 * los pines PA9 y PA10, respectivamente.
 */
void UART_Init(void);
/**
 * @brief Transmitir por la UART1.
 *
 * @param data Puntero a la cadena de datos que se desea transmitir.
 * @param size Tamaño de la cadena de datos.
 */
void UART_Transmit(uint8_t * data, uint16_t size);
/**
 * @brief Recibir por la UART1
 *
 * @param data Puntero a la cadena de datos donde se almacenara lo recibido.
 * @param size Tamaño de la cadena de datos.
 */
void UART_Receive(uint8_t * data, uint16_t size);
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif