/**
  ******************************************************************************
  * @file    ../include/main.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    15-Julho-2015
  * @brief   Inclusão das bibliotecas de usuário e definição de parâmetros
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "delays.h"
#include "buzzer.h"
#include "leds.h"
#include "botao.h"
#include "motores.h"
#include "sensores.h"
#include "encoders.h"
#include "usart1.h"
#include "usb_user.h"
#include "flash.h"
//#include "controlador.h"
#include "speed_profile.h"
#include "rodinhas.h"
#include "flood_fill.h"


/* Definições ----------------------------------------------------------------*/
#define STDIO_UART	// STDIO_UART ou STDIO_USB: direciona as funções de escrita
				// e leitura (printf, scanf, putc...) para a UART ou para a USB

//#define DEBUG_PRINTS

/* Constantes ----------------------------------------------------------------*/
#define N_PARAMETERS 64


/* Tipos ---------------------------------------------------------------------*/
typedef enum
{
	SEARCH_RUN = 0,
	FAST_RUN1,
	FAST_RUN2,
	FAST_RUN3,
	FAST_RUN4,
	FAST_RUN5,
	RETURN_TO_HOME,
	CAL_GYRO,
	CAL_SENSORS,
	CAL_PID,
	CAL_180,
	PRINT_SENSORS,
	TEST_MOVE,
	WAIT,
	STOP
} SelectType_t;

/* Macros --------------------------------------------------------------------*/
/* Protótipos das Funções --------------------------------------------------- */
void systick(void);
void configPeripherals(void);
void fastRun(int32_t topSpeed, int32_t turnSpeed);
void initParameters(void);
void recordPath(void);
void calGyro(void);

/* Variáveis Externas ---------------------------------------------------------*/
extern int32_t i32_bufParams[N_PARAMETERS];


#endif /* __MAIN_H */
