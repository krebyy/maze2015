/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/flash.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Cabeçalho para o módulo flash.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"

/* Constantes ----------------------------------------------------------------*/
#define ADDR_FLASH_SECTOR_10	0x080C0000	// Endereço do setor utilizado - Setor 10 do STM32F405
#define	ADDR_FLASH_SECTOR_11	0x080E0000	// Endereço do setor utilizado - Setor 11 do STM32F405

/* Protótipos das Funções --------------------------------------------------- */
uint8_t writeFlash(uint32_t startAddress, int32_t *buffer, uint32_t num_words);
void readFlash(uint32_t startAddress, int32_t *buffer, uint32_t num_words);


#endif /* __FLASH_H */
