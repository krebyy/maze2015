/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/botao.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções para leitura do Botão SW1
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "botao.h"


/**
  * @brief Configuração do GPIO do botão SW1
  * @param Nenhum
  * @return Nenhum
  */
void configSW1(void)
{
	BOTAO_CLK;	// Habilita o barramento de clock do GPIO do botão

	// Configura o GPIO do Botão como entrada sem resistor interno
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = BOTAO_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(BOTAO_PORT, &GPIO_InitStructure);
}


/**
  * @brief Verifica o estado do botão SW1
  * @param Nenhum
  * @return estado LOW (liberado) ou HIGH (pressionado)
  */
GPIO_PinState getSW1(void)
{
	return HAL_GPIO_ReadPin(BOTAO_PORT, BOTAO_PIN);
}
