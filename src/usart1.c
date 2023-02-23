/**
  ******************************************************************************
  * @file    ../src/usart1.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V2.0.0 - Software micromouseBT
  * @date    23-Agosto-2015
  * @brief   Funções para configuração e inicialização da USART1
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart1.h"

/* Variáveis Externas ------------------------------------------------------- */
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_tx;
char RxBuffer[BUFFER_SIZE];
char RxByte;
uint32_t rx_available = 0;


/**
  * @brief Configuração da USART1
  * @param Nenhum
  * @return Nenhum
  */
void configUsart1(void)
{
	__GPIOA_CLK_ENABLE();	// Habilita o barramento de clock do GPIOA
	__USART1_CLK_ENABLE();	// Habilita o barramento de clock da USART1
	__DMA2_CLK_ENABLE();

	// Configura os GPIOs da USART1 como Alternate Function
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configuração do periférico USART
	huart1.Instance = USART1;
	huart1.Init.BaudRate = BAUD_RATE;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);

	HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);


	/*##-3- Configure the DMA streams ##########################################*/
	/* Configure the DMA handler for Transmission process */
	hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
	hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
	hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_tx.Init.Mode                = DMA_NORMAL;
	hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
	hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
	hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

	HAL_DMA_Init(&hdma_tx);

	/* Associate the initialized DMA handle to the the UART handle */
	__HAL_LINKDMA(&huart1, hdmatx, hdma_tx);

	/*##-4- Configure the NVIC for DMA #########################################*/
	/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart1.hdmatx);
}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    static uint32_t pos = 0;

	RxBuffer[pos] = RxByte;

	HAL_UART_Receive_IT(UartHandle, &RxByte, 1);

	pos++;
	if (RxBuffer[pos - 1] == '\n')
	{
		RxBuffer[pos] = '\0';
		rx_available = pos;
		pos = 0;
	}

}


int32_t comandosUART(void)
{
	int32_t bufferTemp[N_PARAMETERS + 1];
	int32_t checksum = 0;
	static uint32_t i = 0;

	if(rx_available > 0)
	{
		// Envia os dados do robô para o celular
		if (strcmp(RxBuffer, "GET\n") == 0)
		{
			readFlash(ADDR_FLASH_SECTOR_11, bufferTemp, N_PARAMETERS);
			checksum = 0;
			for (i = 0; i < N_PARAMETERS; i++) checksum += bufferTemp[i];
			bufferTemp[N_PARAMETERS] = checksum;

			printf("%ld", (int32_t)0xBBBBBBBB);
			HAL_UART_DMAResume(&huart1);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)bufferTemp, (N_PARAMETERS + 1) * 4);

			//printf("Dados enviados com sucesso, checksum = %d\r\n", checksum);
		}

		// Recebe os dados do celular e grava no robô
		else if (strcmp(RxBuffer, "SET\n") == 0)
		{
			i = 1;
		}
		else if (i == 1)
		{
			char* token;
			i = 0;

			memset(bufferTemp, 0, N_PARAMETERS + 1);
			token = strtok(RxBuffer,"_");
			while (token != NULL)
			{
				bufferTemp[i] = atoi(token);
				i++;
				token = strtok (NULL, "_");
			}

			checksum = 0;
			for (i = 0; i < N_PARAMETERS; i++) checksum += bufferTemp[i];

			if ((checksum == bufferTemp[N_PARAMETERS]) && (checksum != 0))
			{
				writeFlash(ADDR_FLASH_SECTOR_11, bufferTemp, N_PARAMETERS);
				initParameters();
				printf("Dados recebidos com sucesso, checksum = %ld\r\n", checksum);
				beep(100);
			}
			else
			{
				printf("Checksum INCORRETO!\r\n");
			}
		}

		// Parâmetro desconhecido
		else
		{
			printf("ERRO\r\n");
		}

		rx_available = 0;
		memset(RxBuffer, 0, BUFFER_SIZE);
	}

	//delay_ms(1);

	return -1;
}



#ifdef STDIO_UART

/**
  * @brief Redefinição da função de escrita para uso da função printf - stdio.h
  */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, ptr, len, USART_TIMEOUT);
	//HAL_UART_Transmit_DMA(&huart1, ptr, len);

	return len;
}


/**
  * @brief Redefinição da função de leitura para uso da função scanf - stdio.h
  */
int _read (int file, char *ptr, int len)
{
	HAL_UART_Receive(&huart1, ptr, len, USART_TIMEOUT);

	return len;
}

#endif
