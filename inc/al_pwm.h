/* Copyright 2023, Adan Lema <adanlema@hotmail.com> */
#ifndef AL_PWM_H
#define AL_PWM_H
/*==================[inclusions]=============================================*/
#include <stdint.h>
/*==================[macros]=================================================*/
/*
El reloj interno del micro esta a 8MHz y el periodo del PWM se calcula como:
T_PWM = (1+ARR) * T_CLK
Lo que significa que el ARR debe ser igual a: ARR = (T_PWM / T_CLK) - 1
Si F_PWM = 5KHz entonces ARR = 1599
*/
//! Periodo del PWM para tener una frecuencia de 5KHz.
#define PWM_PERIOD 1599
/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/**
 * @brief Configurar el PWM
 * Esta funcion se encarga de configurar el PWM, donde establece como salida el pin 6 del puerto
 * GPIOA. Utiliza el TIM3.
 */
void PWM_Init();
/**
 * @brief Cambiar el ciclo de trabajo.
 * Esta funcion me permite cambiar el ciclo de trabajo del PWM, debo ingresar un valor del duty
 * menor al maximo permitido (PWM_PERIOD).
 * @param duty Nuevo ciclo de trabajo.
 */
void PWM_ChangeDuty(uint32_t duty);

/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif