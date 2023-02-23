/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/leds.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Cabeçalho para o módulo leds.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __LEDS_H
#define __LEDS_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"

/* Constantes ----------------------------------------------------------------*/
#define LOW		GPIO_PIN_RESET
#define HIGH	GPIO_PIN_SET
#define N_LEDS	5
#define LEDS_CLK	__GPIOA_CLK_ENABLE(); __GPIOB_CLK_ENABLE(); __GPIOC_CLK_ENABLE()
#define LED1_PORT	GPIOB
#define LED1_PIN	GPIO_PIN_5
#define LED2_PORT	GPIOC
#define LED2_PIN	GPIO_PIN_0
#define LED3_PORT	GPIOA
#define LED3_PIN	GPIO_PIN_5
#define LED4_PORT	GPIOB
#define LED4_PIN	GPIO_PIN_10
#define LED5_PORT	GPIOB
#define LED5_PIN	GPIO_PIN_11

/* Tipos ---------------------------------------------------------------------*/
typedef enum
{
	LED1 = 0,
	LED2 = 1,
	LED3 = 2,
	LED4 = 3,
	LED5 = 4
} Led_TypeDef;

/* Protótipos das Funções --------------------------------------------------- */
void configLEDs(void);
void setLED(Led_TypeDef led, GPIO_PinState estado);
void toggleLED(Led_TypeDef led);
void allLEDs(GPIO_PinState estado);


#endif /* __LEDS_H */
