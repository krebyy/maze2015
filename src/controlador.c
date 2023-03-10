/*
 * controlador.c
 *
 *  Created on: 30/06/2015
 *      Author: Kleber Lima da Silva (micromousebrasil@gmail.com)
 */

#include "controlador.h"


// Vari?veis Externas ----------------------------------------------------------
TIM_HandleTypeDef htim3;

// Vari?veis Privadas ----------------------------------------------------------
int32_t distancia_mm = 0;
int32_t distancia_deg = 0;
int8_t tipo_movimento = 0;
int32_t setpoint_dist = 0;
int32_t setpoint_speed_esquerda_cnt = 0, setpoint_speed_direita_cnt = 0;
int32_t encoder_esquerda_delta = 0, encoder_direita_delta = 0;
int32_t encoder_esquerda_anterior_cnt = 0, encoder_direita_anterior_cnt = 0;
int32_t pid_esquerda_out = 0, pid_direita_out = 0;
int32_t erro_esquerda_anterior = 0, erro_direita_anterior = 0;
int32_t speed_reta = 200, speed_curva = 200;
bool bUsarSensores = false;



/**
  * @brief Configura??o da Base de Tempo de atualiza??o do speedControl
  * @param Nenhum
  * @return Nenhum
  */
void speedControlConfig(void)
{
	TIM_CLK;

	// Configura??o da base de tempo do speedProfile
	htim3.Instance = TIM;
	htim3.Init.Period = TIM_PERIOD;
	htim3.Init.Prescaler = TIM_PRESCALER;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&htim3);

	HAL_TIM_Base_Start_IT(&htim3);

	HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIMx_IRQn);

	HAL_TIM_IRQHandler(&htim3);
}



/*******************************************************************************
 *Fun??o principal do controle de velocidade -----------------------------------
 */
void controleVelocidade(void)
{
	updateEncoders();	// Updates the distance and speeds
	controladorPID();		// Speed controller
}


/* Fun??o para selecionar os par?metros do movimento ---------------------------
 * raio [mm]: 0 para movimento translacional ou raio para a curva
 * dist_ou_angulo [mm ou deg]: dist?ncia(translational ou angular) do movimento
 * speed [mm/s]: velocidade do movimento
 */
void setMovimento(int32_t raio, int32_t dist_ou_angulo, int32_t speed)
{
	if (raio == 0)
	{
		tipo_movimento = TRANSLACIONAL;

		// Usado para o controle de velocidade
		setpoint_speed_esquerda_cnt = SPEED_TO_COUNTS(speed);
		setpoint_speed_direita_cnt = setpoint_speed_esquerda_cnt;
	}
	else
	{
		tipo_movimento = ROTACIONAL;

		// Calcula as velocidade a partir do raio
		float speedX_mm_s = (float)speed;
		float speedW_rad_s = speedX_mm_s / (float)raio;

		int32_t speedX_cnt = SPEED_TO_COUNTS(abs(speed));
		int32_t speedW_cnt = RAD_S_TO_COUNTS(speedW_rad_s);

		if (raio < RODA_RODA)
		{
			speedX_cnt -= (abs(speedW_cnt) / 2);
			speedW_cnt /= 2;
		}

		// Usado para o controle de velocidade
		setpoint_speed_esquerda_cnt = speedX_cnt - speedW_cnt;
		setpoint_speed_direita_cnt = speedX_cnt + speedW_cnt;
	}

	setpoint_dist = dist_ou_angulo;	// Usado para indica o fim do movimento
}


/* Atualiza a leitura dos encoders ---------------------------------------------
 */
void updateEncoders(void)
{
	int32_t encoder_esquerda_cnt, encoder_direita_cnt;

	// Atualiza a contagem dos encoders
	encoder_esquerda_cnt = LER_ENCODER_E;
	encoder_direita_cnt = LER_ENCODER_D;

	// Atualiza a altera??o dos encoders (speed)
	encoder_esquerda_delta = encoder_esquerda_cnt - encoder_esquerda_anterior_cnt;
	encoder_direita_delta = encoder_direita_cnt - encoder_direita_anterior_cnt;

	// Guarda os valores anteriores
	encoder_esquerda_anterior_cnt = encoder_esquerda_cnt;
	encoder_direita_anterior_cnt = encoder_direita_cnt;

	// Calcula a dist?ncia (mm) e o ?ngulo (deg)
	distancia_mm = COUNTS_TO_MM((encoder_direita_cnt + encoder_esquerda_cnt) / 2);
	distancia_deg = COUNTS_TO_DEG((encoder_direita_cnt - encoder_esquerda_cnt) / 2);
}


/* Controlador PID de velocidade dos motores -----------------------------------
 */
void controladorPID(void)
{
	int32_t erro_esquerda = 0, erro_direita = 0;

	// Alinhamento atrav?s dos sensores
	if (bUsarSensores == true)
	{
		int32_t sensor_feedback = (int32_t)getErroSensores() / K_SENSORES;

		erro_esquerda += sensor_feedback;
		erro_direita -= sensor_feedback;
	}

	// Erro de velocidade
	erro_esquerda += (setpoint_speed_esquerda_cnt - encoder_esquerda_delta);
	erro_direita += (setpoint_speed_direita_cnt - encoder_direita_delta);

	// PD - motor da esquerda
	pid_esquerda_out += (KP * erro_esquerda) +
			(KD * (erro_esquerda - erro_esquerda_anterior));

	// PD - motor da direita
	pid_direita_out += (KP * erro_direita) +
				(KD * (erro_direita - erro_direita_anterior));

	// Guarda os valores anteriores
	erro_esquerda_anterior = erro_esquerda;
	erro_direita_anterior = erro_direita;

	setMotores(pid_esquerda_out, pid_direita_out);
}


/* Indica o fim do movimento ---------------------------------------------------
 */
bool isFinalMovimento(void)
{
	if (tipo_movimento == TRANSLACIONAL && distancia_mm >= setpoint_dist)
	{
		return true;
	}
	else if (tipo_movimento == ROTACIONAL && abs(distancia_deg) >= setpoint_dist)
	{
		return true;
	}
	else if ((tipo_movimento == TRANSLACIONAL)
			&& (frontal_esquerdo > CENTRO_FRONTAL && frontal_direito > CENTRO_FRONTAL))
	{
		return true;
	}

	return false;
}


/* Reseta os encoders e as vari?veis do controlador ----------------------------
 */
void resetControlador(void)
{
	RESET_ENCODER_E;
	RESET_ENCODER_D;

	encoder_esquerda_anterior_cnt = 0;
	encoder_direita_anterior_cnt = 0;

	distancia_mm = 0;
	distancia_deg = 0;

	pid_esquerda_out = 0; pid_direita_out = 0;
	erro_esquerda_anterior = 0; erro_direita_anterior = 0;
	setpoint_speed_esquerda_cnt = 0; setpoint_speed_direita_cnt = 0;
}


/* Movimenta o micromouse para frente uma dist?ncia em [mm] --------------------
 * Se mm == 0: anda para frente at? o pr?ximo comando
 */
void frente(int32_t mm)
{
	bool tem_parede_esquerda = false, tem_parede_direita = false;

	if (mm != 0)
	{
		bUsarSensores = true;
		resetControlador();
		setMovimento(0, mm, speed_reta);

		do
		{
			if (mm == CELL_SIZE)
			{
				/* CALIBRA??O LONGITUDINAL */
				if (lateral_esquerdo > LATERAL_TH)
				{
					tem_parede_esquerda = true;
				}
				if (lateral_direito > LATERAL_TH)
				{
					tem_parede_direita = true;
				}
				if (lateral_esquerdo < LATERAL_FADING_OFF && tem_parede_esquerda == true)
				{
					tem_parede_esquerda = false;
					distancia_mm = CORRECAO_LONGITUDE;
				}
				if (lateral_direito < LATERAL_FADING_OFF && tem_parede_direita == true)
				{
					tem_parede_direita = false;
					distancia_mm = CORRECAO_LONGITUDE;
				}
			}

			delay_ms(1);
		}
		while (isFinalMovimento() == false);
	}
	else
	{
		bUsarSensores = true;
		setMovimento(0, mm, speed_reta);
		delay_ms(1);
	}
}


/* Realiza uma curva em torno do pr?prio eixo (graus < 0: para direita) --------
 */
void curvaPivot(int16_t graus)
{
	bUsarSensores = false;
	resetControlador();
	setMotores(0, 0);
	delay_ms(100);

	// Seleciona o movimento de acordo com o sentido da curva
	if (graus > 0) setMovimento(RODA_RODA / 2, graus, speed_curva);
	else setMovimento(RODA_RODA / 2, -graus, -speed_curva);

	do
	{
		delay_ms(1);
	}
	while (isFinalMovimento() == false);
}


/* Realiza uma curva em torno do pr?prio eixo e anda at? a fronteira da c?lula
 *		(graus < 0: para direita)
 */
void curva(int16_t graus)
{
	frente(CELL_SIZE / 2);
	curvaPivot(graus);
	frente(CELL_SIZE / 2);
}


void setSpeeds(int32_t spd_reta, int32_t spd_curva)
{
	speed_reta = spd_reta;
	speed_curva = spd_curva;
}
