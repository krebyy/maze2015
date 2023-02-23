/**
  ******************************************************************************
  * @file    ../include/speed_profile.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    15-Julho-2015
  * @brief   Funções para controle de velocidade dos motores
  ******************************************************************************
  */

#ifndef SPEED_PROFILE_H_
#define SPEED_PROFILE_H_


/* Libraries -----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "encoders.h"
#include "motores.h"
#include "sensores.h"
#include "delays.h"
#include "buzzer.h"
#include "usart1.h"
#include "leds.h"


/* Constants -----------------------------------------------------------------*/
#define KP_X	1	// Proportional gain for the speed controller (translational)
#define KD_X	2	// Derivative gain for the speed controller (translational)
#define KP_W	1	// Proportional gain for the speed controller (rotational)
#define KD_W	6	// Derivative gain for the speed controller (rotational)

#define TRANSLATIONAL	0
#define ROTATIONAL		1

#define ENCODER_PPR		14400		// Encoder resolution per revolution [ppr]
#define	W_DIAMETER		32			// Wheel diameter [mm]
#define W_DISTANCE		83			// Wheel to wheel distance [mm]
#define CNT_PER_1000MM	143240		// = (1000*ENCODER_PPR) / (W_DIAMETER*PI)
#define CNT_PER_360DEG	37350		// = ((PI*W_DISTANCE)*CNT_PER_1000MM)/(1000)
#define CELL_SIZE	180
#define ONE_CELL_DISTANCE MM_TO_COUNTS(CELL_SIZE)
#define CORRECTION_CELL_DISTANCE MM_TO_COUNTS(100)
#define CORRECTION_TURN_DISTANCE 45	//45 (somente após a curva)
#define OFFSET_BEFORE_TURN 7
#define OFFSET_AFTER_TURN 55

#define TS	1		// Sampling period [ms]

#define SENSOR_SCALE	100
#define GYRO_RATIO		6.65// 6.65 (somente após a curva, raio 90mm) (//((float)(1000000.0 / (float)(CNT_PER_1000MM * TS)))	// mm/s to counts/s

#define MOVE_SPEED		500		// Calibration: 300 mm/s
#define TURN_SPEED		500	// 600
#define SPEED_W			360	// 400

#define MAX_BUF_SIZE	1500

/* Macros --------------------------------------------------------------------*/
#define READ_LEFT_ENCODER	getEncoderEsquerda()
#define READ_RIGHT_ENCODER	getEncoderDireita()
#define RESET_LEFT_ENCODER	resetEncoderEsquerda()
#define RESET_RIGHT_ENCODER	resetEncoderDireita()

#define MM_TO_COUNTS(mm)	((((int32_t)mm) * CNT_PER_1000MM) / 1000)
#define DEG_TO_COUNTS(deg)	((((int32_t)deg) * CNT_PER_360DEG) / 360)
#define COUNTS_TO_MM(cnt)	((((int32_t)cnt) * 1000) / CNT_PER_1000MM)
#define COUNTS_TO_DEG(cnt)	((((int32_t)cnt) * 360) / CNT_PER_360DEG)
#define DIST_TO_COUNTS(mm)	((((int32_t)mm) * CNT_PER_1000MM) / 1000)
#define SPEED_TO_COUNTS(speed)	((CNT_PER_1000MM * ((int32_t)speed) * 2 * TS) / 1000000)
#define COUNTS_TO_SPEED(cnt)	((((float)cnt) * 1000000) / (CNT_PER_1000MM * TS))
//#define RAD_S_TO_COUNTS(rad_s)	(SPEED_TO_COUNTS((rad_s) * W_DISTANCE))
#define RAD_S_TO_COUNTS(rad_s)	((float)(CNT_PER_1000MM * (rad_s) * TS * W_DISTANCE) / (2.0 * 1000000.0))


/* Function prototypes -------------------------------------------------------*/
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
float needToDecelerate(float dist, float curSpd, float endSpd);
void resetSpeedProfile(void);
void moveStraight(int32_t dist, int32_t endSpd);
void moveOneCell(int32_t endSpd);
void turnCorrection(int16_t dist, int16_t deg);
void curveTurn(int16_t deg);
void pivot180(void);
void pivot(int16_t deg);
void frontCal(void);
void doMove(int32_t move);
void doPath( int16_t moves[] , uint8_t num_moves );
void setSpeeds(int32_t spd_move, int32_t spd_turn);
void putBuffers(void);
void printBuffers(void);
void writeLFT(void);

void frente(int32_t mm);
void curva(int16_t angle);
void curvaPivot(int16_t angle);


/* External Variables --------------------------------------------------------*/
extern int32_t targetSpeedX, targetSpeedW;
extern float accX, decX, accW, decW;
extern TIM_HandleTypeDef htim3;


#endif /* SPEED_PROFILE_H_ */
