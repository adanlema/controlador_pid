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
Si F_PWM = 200KHz entonces ARR = 39
*/
#define PWM_PERIOD 39
/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
void PWM_Init();
void PWM_ChangeDuty(uint32_t duty);

/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif