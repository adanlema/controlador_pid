/* Copyright 2023, Adan Lema <adanlema@hotmail.com> */

#ifndef AL_BSP_H
#define AL_BSP_H

/*==================[inclusions]=============================================*/
#include "al_gpio.h"
/*==================[macros]=================================================*/

/*==================[typedef]================================================*/
//! Estructura de la placa que utilizaremos para el controlador PID.
struct board_s {
    DigitalInput_t  boton;
    DigitalOutput_t led;
    DigitalOutput_t salida;
};
//! Creacion de un tipo de dato que es un puntero a una estructura de la placa.
typedef struct board_s * board_t;
/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/**
 * @brief Board Create
 * Se encarga de crear una placa, la cual contendra cuatro entradas digitales y
 * seis salidas digitales.
 * @return board_t Retorna el puntero de la estructura de la placa.
 */
board_t board_Create(void);

/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif