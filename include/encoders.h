/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/encoders.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Cabe�alho para o m�dulo encoders.c
  ******************************************************************************
  */

/* Define para previnir a inclus�o recursiva ---------------------------------*/
#ifndef __ENCODERS_H
#define __ENCODERS_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Constantes ----------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/

/* Prot�tipos das Fun��es --------------------------------------------------- */
void configEncoders(void);
int32_t getEncoderEsquerda(void);
void resetEncoderEsquerda(void);
int32_t getEncoderDireita(void);
void resetEncoderDireita(void);


#endif /* __ENCODERS_H */
