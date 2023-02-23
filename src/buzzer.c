/**
  ******************************************************************************
  * @file    ../src/buzzer.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções de acesso ao buzzer
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "buzzer.h"


/* Variáveis Externas ------------------------------------------------------- */
int32_t buzzerTime;


/**
  * @brief Configuração do pino para acionar o buzzer
  * @param Nenhum
  * @return Nenhum
  */
void configBuzzer(void)
{
	BUZZER_CLK;	// Habilita o barramento de clock do GPIO do buzzer

	// Configura o pino do buzzer como saída push/pull
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = BUZZER_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);
}


/**
  * @brief Gera vários beeps, o programa fica nesta rotina enquanto os beeps
  * são gerados
  * @param vezes: Número de beeps a serem gerados
  * @param t_on: Tempo que o buzzer fica ligado [ms]
  * @param t_off: Tempo que o buzzer fica desligado [ms]
  * @return Nenhum
  */
void beeps(int vezes, int t_on, int t_off)
{
	buzzerTime = vezes * (t_on + t_off);

	for (int i = 0; i < vezes; i++)
	{
		BEEP_ON;
		delay_ms(t_on);
		BEEP_OFF;
		delay_ms(t_off);
	}
}


/**
  * @brief Gera um beep que pode ser usado durante o funcionamento do robô
  * @param duracao: Tempo do beep [ms]
  * @return Nenhum
  */
void beep(int duracao)
{
	BEEP_ON;
	buzzerTime = duracao;
}


void finishSong(void)
{
	beeps(25, 2, 2);
	beeps(10, 2, 1);
	beeps(10, 1, 2);
}
