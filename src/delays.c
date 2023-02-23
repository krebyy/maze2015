/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/delays.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções de atrasos
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "delays.h"


/**
  * @brief Realiza a leitura do tempo passado desde o inicio da operação
  * @param Nenhum
  * @return microsegundos desde o inicio da operação
  */
uint32_t micros(void)
{
	return (HAL_GetTick()*1000 + 1000 - SysTick->VAL/SYSTEM_FREQ);
}


/**
  * @brief Realiza a leitura do tempo passado desde o inicio da operação
  * @param Nenhum
  * @return milisegundos desde o inicio da operação
  */
uint32_t millis(void)
{
	return HAL_GetTick();
}


/**
  * @brief Aguarda um tempo em microsegundos em relação a um tempo inicial
  * @param targetTime: Tempo que será aguardado em microsegundos
  * @param t0: Tempo inicial em microsegundos
  * @return Nenhum
  */
void elapse_us(uint32_t targetTime, uint32_t t0)
{
	while ((micros() - t0) < targetTime);
}


/**
  * @brief Gera um atraso
  * @param delay: Tempo do atraso em milisegundos
  * @return Nenhum
  */
void delay_ms(uint32_t delay)
{
	uint32_t ms_start = HAL_GetTick();

	while ((HAL_GetTick() - ms_start) < delay);
}


/**
  * @brief Gera um atraso
  * @param delay: Tempo do atraso em micrsegundos
  * @return Nenhum
  */
void delay_us(uint32_t delay)
{
	uint32_t us_start = HAL_GetTick();

	while ((micros() - us_start) < delay);
}

