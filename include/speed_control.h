/**
  ******************************************************************************
  * @file    ../include/speed_control.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    27-Abril-2015
  * @brief   Cabeçalho para o módulo speed_control.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "main.h"

/* Constantes ----------------------------------------------------------------*/
#define KP_X 2
#define KD_X 0	//6
#define KP_W 5	//1
#define KD_W 15//15	//3

#define TS 1	// Tempo de atualização em [ms]

// Timer para base de tempo de atualização
#define TIM_CLK			__TIM3_CLK_ENABLE()
#define TIM				TIM3
#define TIM_PERIOD		999
#define TIM_PRESCALER	83
#define TIMx_IRQn		TIM3_IRQn
#define TIMx_IRQHandler	TIM3_IRQHandler


#define CNT_PER_1000MM 142400	// = (1000*PPR) / (W_DIAMETER*PI)
#define CNT_PER_360DEG 37120	// = ((2*PI*W_DISTANCE)*CNT_PER_1000MM)/(2*1000)
#define W_DISTANCE	83


#define GYRO_SCALE 1
#define SENSOR_SCALE 1


#define MINIMAL_SX_STRAIGHT 5

#define SIZE_BUFFER_SECTORS		40

#define PAUSE			9
#define STOP			10
#define CAL_GYRO		66
#define TESTE			88

#define GOAL_OK			4
#define RUN_OK			5
#define WAIT			6


/* Macros --------------------------------------------------------------------*/
#define MM_TO_COUNTS(mm)	(((mm) * CNT_PER_1000MM) / 1000)
#define SPEEDX_TO_COUNTS(speed)	((CNT_PER_1000MM * (speed) * 2 * TS) / 1000000)
#define ACCX_TO_COUNTS(acc)		(SPEEDX_TO_COUNTS(acc) / (TS * 1000))
#define COUNTS_TO_MM(cnt)	(((cnt) * 1000) / CNT_PER_1000MM)

#define DEG_TO_COUNTS(deg)	(((deg) * CNT_PER_360DEG) / 360)
#define SPEEDW_TO_COUNTS(speed)	((CNT_PER_360DEG * (speed) * 2 * TS) / 360000)
#define ACCW_TO_COUNTS(acc)		(SPEEDW_TO_COUNTS(acc / 2) / TS)
#define COUNTS_TO_DEG(cnt)	(((cnt) * 360) / CNT_PER_360DEG)

#define ACCC_TO_COUNTS(acc) (float)((float)acc / 1755.6180f)

#define W_2		(float)((float)MM_TO_COUNTS(W_DISTANCE) / 2.0f)


/* Protótipos das Funções --------------------------------------------------- */
void speedControlConfig(void);

void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int32_t needToDecelerate(int32_t, int32_t, int32_t);
void resetProfile(void);

void manageRuns(void);
void calculateSpeedProfile(int32_t topSpeedX, int32_t accC);
void changeSpeedProfile(void);
void updateBufferSpeedProfile(void);
void writeLFT(void);


/* Variáveis externas --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern int32_t distanceLeft, distance_mm;
extern int32_t targetSpeedX, targetSpeedW;
extern int32_t endSpeedX, endSpeedW;
extern int32_t accX, decX, accW, decW;
extern bool useEncoderFeedback;
extern bool useGyroFeedback;
extern bool useSensorFeedback;
extern int32_t buf_temp[3 * SIZE_BUFFER_SECTORS];


#endif /* __SPEED_CONTROL_H */
