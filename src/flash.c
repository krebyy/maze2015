/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/flash.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções de acesso à memória flash para guardar dados
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "flash.h"


/**
  * @brief Função para gravar um buffer na Flash
  * @param buffer com os dados a serem gravados
  * @param num_words: tamanho do buffer
  * @return 0: erro durante a gravação; 1: gravação realizada com sucesso
  */
uint8_t writeFlash(uint32_t startAddress, int32_t *buffer, uint32_t num_words)
{
	HAL_FLASH_Unlock();

	if (startAddress == ADDR_FLASH_SECTOR_10)
	{
		FLASH_Erase_Sector(FLASH_SECTOR_10, VOLTAGE_RANGE_3);
	}
	else if (startAddress == ADDR_FLASH_SECTOR_11)
	{
		FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);
	}

	for (uint32_t i = 0; i < num_words; i++)
	{
		if (HAL_FLASH_Program(TYPEPROGRAM_WORD,(uint32_t)(startAddress+(i*4)),*buffer)!= HAL_OK)
		{
			HAL_FLASH_Lock();
			return 0;
		}
		buffer++;
	}

	HAL_FLASH_Lock();
	return 1;
}


/**
  * @brief Função para ler o buffer gravado na Flash pela writeFlash
  * @param buffer para receber os dados
  * @param num_words (tamanho do buffer)
  * @return nenhum
  */
void readFlash(uint32_t startAddress, int32_t *buffer, uint32_t num_words)
{
	for (uint32_t i = 0; i < num_words; i++)
	{
		buffer[i] = *(int32_t *)(startAddress + (i*4));
	}
}
