/*
 * controlador.h
 *
 *  Created on: 30/06/2015
 *      Author: Kleber Lima da Silva (micromousebrasil@gmail.com)
 */

#ifndef CONTROLADOR_H_
#define CONTROLADOR_H_


// Bibliotecas -----------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "encoders.h"
#include "motores.h"
#include "sensores.h"
#include "delays.h"


// Constantes ------------------------------------------------------------------
// Ganho dos controladores de velocidade
#define KP	20	// Ganho Proporcional
#define KD	40	// Ganho Derivativo

// Indica o tipo de movimento
#define TRANSLACIONAL	0
#define ROTACIONAL		1

// Parâmetros do robô
#define ENCODER_PPR		360		// Resolução do Encoder pulsos por revolução [ppr]
#define	DIAMETRO_RODA	32		// Diâmetro da roda [mm]
#define RODA_RODA		84		// Distância entre as rodas [mm]
#define CNT_POR_1000MM	3581	// = (1000*ENCODER_PPR) / (DIAMETRO_RODA*PI)
#define CNT_POR_360DEG	910		// = ((PI*W_DISTANCE)*CNT_PER_1000MM)/(1000)	//945

// Parâmetros do controlador
#define TS	10		// Período de amostragem [ms]
#define K_SENSORES	100	// Constante para ajuste do peso dos sensores no controlador
#define SPEED_RETA	200	// Velocidade nas retas [mm/s]
#define SPEED_CURVA	200	// Velocidade nas curvas [mm/s]

#define CELL_SIZE	180	// Tamanho da célula [mm]
#define CORRECAO_LONGITUDE 90 // Valor para correção longitudinal


// Timer para base de tempo de atualização
#define TIM_CLK			__TIM3_CLK_ENABLE()
#define TIM				TIM3
#define TIM_PERIOD		999
#define TIM_PRESCALER	839
#define TIMx_IRQn		TIM3_IRQn
#define TIMx_IRQHandler	TIM3_IRQHandler


// Macros ----------------------------------------------------------------------
#define LER_ENCODER_E	getEncoderEsquerda()
#define LER_ENCODER_D	getEncoderDireita()
#define RESET_ENCODER_E	resetEncoderEsquerda()
#define RESET_ENCODER_D	resetEncoderDireita()

#define COUNTS_TO_MM(cnt)	(((cnt) * 1000) / CNT_POR_1000MM)
#define COUNTS_TO_DEG(cnt)	(((cnt) * 360) / CNT_POR_360DEG)
#define DIST_TO_COUNTS(mm)	(((mm) * CNT_POR_1000MM) / 1000)
#define SPEED_TO_COUNTS(speed)	((CNT_POR_1000MM * (speed) * TS) / 1000000)
#define RAD_S_TO_COUNTS(rad_s)	(SPEED_TO_COUNTS(rad_s * RODA_RODA))


// Protótipo das Funções -------------------------------------------------------
void speedControlConfig(void);
void controleVelocidade(void);
void setMovimento(int32_t raio, int32_t dist_angulo, int32_t speed);
void updateEncoders(void);
void controladorPID(void);
void controleVelocidade(void);
bool isFinalMovimento(void);
void resetControlador(void);
void frente(int32_t distancia);
void curvaPivot(int16_t graus);
void curva(int16_t graus);
void setSpeeds(int32_t spd_reta, int32_t spd_curva);


/* Variáveis externas --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;


#endif /* CONTROLADOR_H_ */
