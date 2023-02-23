/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/usart1.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Cabeçalho para o módulo usart1.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __USART1_H
#define __USART1_H


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f4xx.h"
#include "main.h"

/* Constantes ----------------------------------------------------------------*/
#define BAUD_RATE	115200
#define USART_TIMEOUT	100
#define BUFFER_SIZE	100


#define USARTx_IRQn			USART1_IRQn
#define USARTx_IRQHandler	USART1_IRQHandler

#define USARTx_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define USARTx_TX_DMA_STREAM             DMA2_Stream7

/* Protótipos das Funções --------------------------------------------------- */
void configUsart1(void);
int32_t comandosUART(void);

/* Variáveis ---------------------------------------------------------------- */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_tx;
extern char RxBuffer[BUFFER_SIZE];
extern char RxByte;
extern uint32_t rx_available;


#endif /* __USART1_H */
