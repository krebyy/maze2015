/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/usb_user.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    26-Abril-2015
  * @brief   Funções para configuração e inicialização da USB no modo CDC
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_user.h"


/* Variáveis Externas ------------------------------------------------------- */
extern PCD_HandleTypeDef hpcd;
USBD_HandleTypeDef USBD_Device;


/**
  * @brief Configuração da USB
  * @param Nenhum
  * @return Nenhum
  */
void configUSB(void)
{
#ifdef STDIO_USB
	USBD_Init(&USBD_Device, &VCP_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);
#endif
}


/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}


/**
  * @brief Redefinição das funções de escrita/leitura para uso das funções da stdio.h
  */
#ifdef STDIO_USB
int _fstat (int fd, struct stat *pStat)
{
	pStat->st_mode = S_IFCHR;
	return 0;
}

int _close(int fildes)
{
	return -1;
}

int _write (int fd, char *pBuffer, int size)
{
	return VCP_write(pBuffer, size);
}

int _isatty (int fd)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return -1;
}

int _read (int fd, char *pBuffer, int size)
{
	while (1)
	{
		int done = VCP_read(pBuffer, size);
		if (done)
			return done;
	}

	return 0;
}
#endif
