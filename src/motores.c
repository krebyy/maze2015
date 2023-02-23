/**
  ******************************************************************************
  * @file    src/motores.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    17-Maio-2015
  * @brief   Funções para acionamento dos motores
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motores.h"


/** @defgroup Variáveis privadas
  * @{
  */
GPIO_TypeDef* MOTORES_PORT[N_PINS] =
			{L_IN1_PORT, L_IN2_PORT, R_IN1_PORT, R_IN2_PORT, STBY_PORT};

const uint16_t MOTORES_PIN[N_PINS] =
			{L_IN1_PIN, L_IN2_PIN, R_IN1_PIN, R_IN2_PIN, STBY_PIN};

TIM_HandleTypeDef TimHandle;
/**
  * @}
  */


/**
  * @brief Configuração dos GPIOs e PWMs para acionamento dos Motores
  * @param Nenhum
  * @return Nenhum
  */
void configMotors(void)
{
	MOTORES_CLK; // Habilita o barramento de clock do GPIO do drive dos motores

	// Configura os GPIOs do drive dos motores como saídas push/pull
	GPIO_InitTypeDef GPIO_InitStructure;
	for (int i = 0; i < N_PINS; i++)
	{
		GPIO_InitStructure.Pin = MOTORES_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(MOTORES_PORT[i], &GPIO_InitStructure);
	}
	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

	// Configura os GPIOs de PWM como Alternate Function
	GPIO_InitStructure.Pin = L_PWM_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = PWM_AF_TIM;
	HAL_GPIO_Init(L_PWM_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = R_PWM_PIN;
	HAL_GPIO_Init(R_PWM_PORT, &GPIO_InitStructure);

	// Configuração da base de tempo do PWM
	// Frequência do PWM:	f = [(168M / 2) / (PS + 1) ] / T
	TimHandle.Instance = PWM_TIM;
	TimHandle.Init.Period = PWM_PERIOD;
	TimHandle.Init.Prescaler = PWM_PRESCALER;
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle);

	// Configuração do Output Compare do PWM
	TIM_OC_InitTypeDef sConfig;
	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.Pulse = 999;
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, L_PWM_CHANNEL);
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, R_PWM_CHANNEL);

	HAL_TIM_PWM_Start(&TimHandle, L_PWM_CHANNEL);
	HAL_TIM_PWM_Start(&TimHandle, R_PWM_CHANNEL);
}


/**
  * @brief Atualiza o PWM dos motores da esquerda e da direita
  * @param pwm_esquerda: -1000 a 1000
  * @param pwm_direita: -1000 a 1000
  * @return Nenhum
  */
void setMotores(int16_t pwm_esquerda, int16_t pwm_direita)
{
	// Acionamento do motor da esquerda
	if (pwm_esquerda < 0)
	{
		pwm_esquerda = -pwm_esquerda;

		HAL_GPIO_WritePin(L_IN1_PORT, L_IN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_IN2_PORT, L_IN2_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(L_IN1_PORT, L_IN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_IN2_PORT, L_IN2_PIN, GPIO_PIN_RESET);
	}

	// Acionamento do motor da direita
	if (pwm_direita < 0)
	{
		pwm_direita = -pwm_direita;

		HAL_GPIO_WritePin(R_IN1_PORT, R_IN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_IN2_PORT, R_IN2_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(R_IN1_PORT, R_IN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_IN2_PORT, R_IN2_PIN, GPIO_PIN_RESET);
	}

	// Trata o estouro de escala
	if (pwm_esquerda > 1000)
	{
		pwm_esquerda = 1000;
	}

	if (pwm_direita > 1000)
	{
		pwm_direita = 1000;
	}

	// Atualiza os PWMs
	L_PWM_CCR = pwm_esquerda;
	R_PWM_CCR = pwm_direita;
}


