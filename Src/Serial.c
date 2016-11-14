/******************************************************************************/
/* SERIAL.C: Low Level Serial Routines                                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

/*
 * Copyright (c) 2016 Du Huanpeng <u74147@gmail.com>
 */

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart2;

/* implementation of putchar (also used by printf function to output data)    */
int sendchar (int ch)  {                 /* Write character to Serial Port    */

	HAL_StatusTypeDef error;
	static unsigned char buf[4];
	int n;

	if(ch == '\n') {
		buf[0] = '\r';
		buf[1] = '\n';
		n      = 2;
	} else {
		buf[0] = ch;
		n      = 1;
	}
	
	error = HAL_UART_Transmit(&huart2, buf, n, 64);
	//error = HAL_UART_Transmit_DMA(&huart2, buf, n);

	if(error) return -1;

  return ch;
}


int getkey (void)  {                    /* Read character from Serial Port   */
	unsigned char buf[1];
	int ch;
	
	HAL_StatusTypeDef error;
	error = HAL_UART_Receive(&huart2, buf, 1, 64);
	if(error) ch = 'E';
	else ch = buf[0];
	return ch;
}
