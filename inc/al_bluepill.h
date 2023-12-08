/* Copyright 2023, Adan Lema <adanlema@hotmail.com> */

#ifndef AL_BLUEPILL_H
#define AL_BLUEPILL_H

/*==================[inclusions]=============================================*/
#include "hal.h"
#include "stm32f1xx_hal.h"
/*==================[macros]=================================================*/
/* Macros para configurar GPIO */
#define GPIO_MOD_MASK    0b11
#define GPIO_MOD_INP     0b00
#define GPIO_MOD_OUT_10M 0b01
#define GPIO_MOD_OUT_2M  0b10
#define GPIO_MOD_OUT_50M 0b11

#define GPIO_CNF_MASK    0b1100
#define GPIO_CNF_INP_ANG 0b0000 // Analog Mode
#define GPIO_CNF_INP_FLT 0b0100 // Floating Input
#define GPIO_CNF_INP_PPL 0b1000 // Pull-up/Pull-down
#define GPIO_CNF_OUT_GPP 0b0000 // General output Push-pull
#define GPIO_CNF_OUT_GOD 0b0100 // General output Open-drain
#define GPIO_CNF_OUT_APP 0b1000 // Alternative output Push-pull
#define GPIO_CNF_OUT_AOD 0b1100 // Alternative output Open-drain

/*  Pines a utilizar */
#define PUERTO_GPIOA 0
#define PUERTO_GPIOB 1
#define PUERTO_GPIOC 2
#define PUERTO_GPIOD 3

#define LED_PUERTO   PUERTO_GPIOC
#define LED_PIN      13

#define PWM_PUERTO   PUERTO_GPIOA
#define PWM_PIN      6

#define ADC_PUERTO   PUERTO_GPIOA
#define ADC_PIN      0

#define TX_PUERTO    PUERTO_GPIOA
#define TX_PIN       9

#define RX_PUERTO    PUERTO_GPIOA
#define RX_PIN       10

/*==================[typedef]================================================*/
static GPIO_TypeDef * const puertos_gpio[] = {GPIOA, GPIOB, GPIOC, GPIOD};
/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/** @ doxygen end group definition */
/** @ doxygen end group definition */
/** @ doxygen end group definition */
/*==================[end of file]============================================*/
#endif