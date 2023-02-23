/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/motores.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    17-Fevereiro-2015
  * @brief   Cabeçalho para o módulo motores.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __MOTORES_H
#define __MOTORES_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Constantes ----------------------------------------------------------------*/
#define N_PINS	5
#define MOTORES_CLK	__GPIOB_CLK_ENABLE(); __GPIOC_CLK_ENABLE(); __TIM4_CLK_ENABLE()
#define PWM_AF_TIM	GPIO_AF2_TIM4
#define PWM_TIM		TIM4
#define PWM_PERIOD		999
#define PWM_PRESCALER	3
#define L_PWM_PORT	GPIOB
#define L_PWM_PIN	GPIO_PIN_6
#define L_PWM_CHANNEL	TIM_CHANNEL_1
#define L_PWM_CCR	TIM4->CCR1
#define L_IN2_PORT	GPIOB
#define L_IN2_PIN	GPIO_PIN_9
#define L_IN1_PORT	GPIOB
#define L_IN1_PIN	GPIO_PIN_8

#define R_PWM_PORT	GPIOB
#define R_PWM_PIN	GPIO_PIN_7
#define R_PWM_CHANNEL	TIM_CHANNEL_2
#define R_PWM_CCR	TIM4->CCR2
#define R_IN2_PORT	GPIOC
#define R_IN2_PIN	GPIO_PIN_2
#define R_IN1_PORT	GPIOC
#define R_IN1_PIN	GPIO_PIN_3
#define STBY_PORT	GPIOC
#define STBY_PIN	GPIO_PIN_1

/* Protótipos das Funções --------------------------------------------------- */
void configMotors(void);
void setMotores(int16_t pwm_esquerda, int16_t pwm_direita);


#endif /* __MOTORES_H */
