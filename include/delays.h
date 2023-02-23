/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/delays.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Cabeçalho para o módulo delays.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __DELAYS_H
#define __DELAYS_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Constantes ----------------------------------------------------------------*/
#define SYSTEM_FREQ 168

/* Protótipos das Funções --------------------------------------------------- */
uint32_t micros(void);
uint32_t millis(void);
void elapse_us(uint32_t targetTime, uint32_t t0);
void delay_ms(uint32_t delay);
void delay_us(uint32_t delay);


#endif /* __DELAYS_H */
