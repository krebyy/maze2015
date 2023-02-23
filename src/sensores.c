/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/sensores.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções para acionamento do Botão SW1
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensores.h"

// Variáveis Externas ----------------------------------------------------------
int16_t frontal_esquerdo = 0;
int16_t lateral_esquerdo = 0;
int16_t lateral_direito = 0;
int16_t frontal_direito = 0;


/** @defgroup Variáveis privadas
  * @{
  */
GPIO_TypeDef* EMISSORES_PORT[N_EMISSORES] =
			{LF_E_PORT, POWER1_PORT, POWER2_PORT, RF_E_PORT,
			L_LINE_PORT, R_LINE_PORT};
const uint16_t EMISSORES_PIN[N_EMISSORES] =
			{LF_E_PIN, POWER1_PIN, POWER2_PIN, RF_E_PIN,
			L_LINE_PIN, R_LINE_PIN};

GPIO_TypeDef* RECEPTORES_PORT[N_RECEPTORES] =
			{LINE1_PORT, LINE2_PORT, LINE3_PORT, LINE4_PORT,
			LINE5_PORT, LINE6_PORT, LINE7_PORT, LINE8_PORT};
const uint16_t RECEPTORES_PIN[N_RECEPTORES] =
			{LINE1_PIN, LINE2_PIN, LINE3_PIN, LINE4_PIN,
			LINE5_PIN, LINE6_PIN, LINE7_PIN, LINE8_PIN};

GPIO_TypeDef* ANALOGICAS_PORT[N_ANALOGICAS] =
			{LF_R_PORT, L_R_PORT, R_R_PORT, RF_R_PORT,
			G_OUTZ_PORT, G_VREF_PORT, VBAT_PORT};
const uint16_t ANALOGICAS_PIN[N_ANALOGICAS] =
			{LF_R_PIN, L_R_PIN, R_R_PIN, RF_R_PIN,
			G_OUTZ_PIN, G_VREF_PIN, VBAT_PIN};

ADC_HandleTypeDef hadc1;
/**
  * @}
  */


/**
  * @brief Configuração dos GPIOs e ADCs dos sensores
  * @param Nenhum
  * @return Nenhum
  */
void configSensors(void)
{
	SENSORES_CLK;	// Habilita o barramento de clock do GPIO dos Sensores
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configura os GPIOs dos emissores como saída push/pull
	for (int i = 0; i < N_EMISSORES; i++)
	{
		GPIO_InitStructure.Pin = EMISSORES_PIN[i];;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
		HAL_GPIO_Init(EMISSORES_PORT[i], &GPIO_InitStructure);
		HAL_GPIO_WritePin(EMISSORES_PORT[i], EMISSORES_PIN[i], LOW);
	}


	// Configura os GPIOs dos receptores como entrada sem resistor interno
	for (int i = 0; i < N_RECEPTORES; i++)
	{
		GPIO_InitStructure.Pin = RECEPTORES_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(RECEPTORES_PORT[i], &GPIO_InitStructure);
	}


	// Configura os pinos analógicos
	for (int i = 0; i < N_ANALOGICAS; i++)
	{
		GPIO_InitStructure.Pin = ANALOGICAS_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ANALOGICAS_PORT[i], &GPIO_InitStructure);
	}

	// Configuração do ADC
	__ADC1_CLK_ENABLE();
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);


	// Leitura da Bateria, gera um alerta quando a tensão for menor que 7,00V
	int32_t temp_bat = 0;
	for (uint8_t i = 0; i < 100; i++)
	{
		temp_bat += getTensao();
		delay_ms(1);
	}
	if ((temp_bat / 100) > VBAT_ALERTA)
	{
#ifdef DEBUG_PRINTS
		printf("Bateria: %d mV\r\n", getTensao());
#endif
		beeps(1, 50, 50);
	}
	else
	{
#ifdef DEBUG_PRINTS
		printf("Bateria BAIXA!\r\n");
#endif
		beeps(5, 50, 50);
		beep(500);
	}
}


/* Função para leitura dos sensores de parede ----------------------------------
 * Atualiza as leituras dos sensores frontais e laterais.
 * Retorna uma máscara de bits indicando presença (1) ou não (0)
 * de paredes. O bit mais significativo representa a parede da
 * esquerda. Ex.: 011 = presença de parede frontal e direita.
 */
uint8_t getSensoresParede(void)
{
	uint8_t paredes = 0;

	frontal_esquerdo = getRawADC(LF_R_CH);
	lateral_esquerdo = getRawADC(L_R_CH);
	lateral_direito = getRawADC(R_R_CH);
	frontal_direito = getRawADC(RF_R_CH);

	// Registra o tempo atual
	uint32_t t0 = micros();

	// Sensor frontal esquerdo
	HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, HIGH);
	while((micros() - t0) < 60);
	frontal_esquerdo = getRawADC(LF_R_CH) - frontal_esquerdo;
	HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, LOW);
	if(frontal_esquerdo < 0) frontal_esquerdo = 0;
	while((micros() - t0) < 140);

	// Sensor frontal direito
	HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, HIGH);
	while((micros() - t0) < 200);
	frontal_direito = getRawADC(RF_R_CH) - frontal_direito;
	HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, LOW);
	if(frontal_direito < 0) frontal_direito = 0;
	while((micros() - t0) < 280);

	// Sensores laterais
	HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, HIGH);
#ifdef POWER2
	HAL_GPIO_WritePin(POWER2_PORT, POWER2_PIN, HIGH);
#endif
	while((micros() - t0) < 340);
	lateral_esquerdo = getRawADC(L_R_CH) - lateral_esquerdo;
	lateral_direito = getRawADC(R_R_CH) - lateral_direito;
	HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, LOW);
#ifdef POWER2
	HAL_GPIO_WritePin(POWER2_PORT, POWER2_PIN, LOW);
#endif
	if(lateral_esquerdo < 0) lateral_esquerdo = 0;
	if(lateral_direito < 0) lateral_direito = 0;

	// Realiza a máscara de bits
	if(frontal_esquerdo > FRONTAL_TH || frontal_direito > FRONTAL_TH)
	{
		paredes |= 0b010;
	}

	if(lateral_esquerdo > LATERAL_TH)
	{
		paredes |= 0b100;
	}

	if(lateral_direito > LATERAL_TH)
	{
		paredes |= 0b001;
	}

	return paredes;
}


/* Retorna o erro de alinhamento dos sensores ----------------------------------
 *
 */
int16_t getErroSensores(void)
{
	int16_t erro = 0;

	/* ------ ALINHAMENTO LATERAL ------ */
	if (frontal_esquerdo < FRONTAL_TH && frontal_direito < FRONTAL_TH)
	{
#if PID_TECHNIQUE == 1
		if (lateral_esquerdo > CENTRO_LATERAL && lateral_direito < CENTRO_LATERAL)
		{
			erro = lateral_esquerdo - CENTRO_LATERAL;
		}
		else if (lateral_direito > CENTRO_LATERAL && lateral_esquerdo < CENTRO_LATERAL)
		{
			erro = CENTRO_LATERAL - lateral_direito;
		}
#elif PID_TECHNIQUE == 2
		if (lateral_esquerdo > LATERAL_TH && lateral_direito > LATERAL_TH)
		{
			erro = lateral_esquerdo - lateral_direito;
		}
		else if (lateral_esquerdo > LATERAL_TH)
		{
			erro = 2 * (lateral_esquerdo - CENTRO_LATERAL);
		}
		else if (lateral_direito > LATERAL_TH)
		{
			erro = 2 * (CENTRO_LATERAL - lateral_direito);
		}
#endif
	}
	/* ------ ALINHAMENTO FRONTAL ------ */
	else if (frontal_esquerdo > ALINHAMENTO_FRONTAL && frontal_direito > ALINHAMENTO_FRONTAL)
	{
		erro = (frontal_direito - frontal_esquerdo);
	}

	return erro;
}


/**
  * @brief Verifica a leitura do giroscópio
  * @param Nenhum
  * @return w: Gyro Raw
  */
int32_t getRawGyro()
{
	int32_t w = 0;

	w = getRawADC(G_OUTZ_CH) - getRawADC(G_VREF_CH);

	return w;
}


// return in deg/s
float getGyro()
{
	int32_t acumulator_aSpeed = 0;

	for (uint8_t i = 0; i < 30; i++)
	{
		acumulator_aSpeed += getRawGyro();
	}

	return T_GYRO((float)acumulator_aSpeed / 30.0);	// return in deg/s
}


/**
  * @brief Verifica a tensão da bateria
  * @param Nenhum
  * @return Tensão da bateria em mV
  */
int32_t getTensao()
{
	return ((getRawADC(VBAT_CH) * VBAT_V) / VBAT_RAW);
}


/**
  * @brief Realiza a conversão de um canal analógico
  * @param canal: Canal analógico a ser realizado a conversão
  * @return rawADC: Resultado da conversão (valor de 12 bits)
  */
uint32_t getRawADC(uint32_t canal)
{
	uint32_t rawADC;
	ADC_ChannelConfTypeDef sConfig;

	sConfig.Channel = canal;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	rawADC = HAL_ADC_GetValue(&hadc1);

	return rawADC;
}


bool waitStart(void)
{
	if ((getSensoresParede() & 0b010) == 0)
	{
		delay_ms(100);

		return false;
	}
	delay_ms(2000);

	frontal_esquerdo = 0;
	lateral_esquerdo = 0;
	lateral_direito = 0;
	frontal_direito = 0;

	return true;
}
