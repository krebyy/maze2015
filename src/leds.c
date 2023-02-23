/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/leds.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções para acionamento dos LEDs
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "leds.h"


/** @defgroup Variáveis privadas
  * @{
  */
GPIO_TypeDef* LEDS_PORT[N_LEDS] =
			{LED1_PORT, LED2_PORT, LED3_PORT, LED4_PORT, LED5_PORT};

const uint16_t LEDS_PIN[N_LEDS] =
			{LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN};
/**
  * @}
  */


/**
  * @brief Configuração dos GPIOs para acionamento dos LEDs
  * @param Nenhum
  * @return Nenhum
  */
void configLEDs(void)
{
	LEDS_CLK;	// Habilita o barramento de clock do GPIO dos LEDs

	// Configura os GPIOs dos LEDs como saídas push/pull
	GPIO_InitTypeDef GPIO_InitStructure;
	for (int i = 0; i < N_LEDS; i++)
	{
		GPIO_InitStructure.Pin = LEDS_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(LEDS_PORT[i], &GPIO_InitStructure);
	}
}


/**
  * @brief Altera o estado de um LED
  * @param led: Número do LED
  * @param estado: LOW - apaga o LED ou HIGH - liga o LED
  * @return Nenhum
  */
void setLED(Led_TypeDef led, GPIO_PinState estado)
{
	HAL_GPIO_WritePin(LEDS_PORT[led], LEDS_PIN[led], estado);
}


/**
  * @brief Altera o estado de um LED para o estado complementar
  * @param led: Número do LED
  * @return Nenhum
  */
void toggleLED(Led_TypeDef led)
{
	HAL_GPIO_TogglePin(LEDS_PORT[led], LEDS_PIN[led]);
}


/**
  * @brief Altera o estado de todos os LEDs
  * @param estado: LOW - apaga todos os LEDs ou HIGH - liga todos os LEDs
  * @return Nenhum
  */
void allLEDs(GPIO_PinState estado)
{
	for (int i = 0; i < N_LEDS; i++)
	{
		HAL_GPIO_WritePin(LEDS_PORT[i], LEDS_PIN[i], estado);
	}
}
