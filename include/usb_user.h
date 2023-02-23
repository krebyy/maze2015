/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/usb_user.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Cabeçalho para o módulo usb_user.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __USB_USER_H
#define __USB_USER_H


/* Includes ------------------------------------------------------------------*/
#include <sys/stat.h>
#include <stdio.h>
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>
#include "stm32f4xx.h"
#include "main.h"

/* Constantes ----------------------------------------------------------------*/
/* Protótipos das Funções --------------------------------------------------- */
void configUSB(void);

/* Variáveis ---------------------------------------------------------------- */


#endif /* __USB_USER_H */
