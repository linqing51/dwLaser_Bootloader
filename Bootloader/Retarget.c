/*----------------------------------------------------------------------------
 * Name:    Retarget.c
 * Purpose: 'Retarget' layer for target-dependent low level functions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "main.h"
#include "stdio.h"
#include "core_cm4.h"
#include "usart.h"
#include "bootloader.h"
#pragma import(__use_no_semihosting_swi)



struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

//extern UART_HandleTypeDef huart1;
//int fputc(int ch,FILE *f){
//    uint8_t temp[1]={ch};
//    HAL_UART_Transmit(&huart1, temp, 1, 2);
//	return ch;
//}


int fputc(int ch,FILE *f){
	uint8_t dat;
	dat = (ch & 0xFF);
#if (BOARD_STM32F413H_DISCO == 1) || (BOARD_STM32F413RH_DWLASER == 1)
	HAL_UART_Transmit(&huart1, &dat, 1, 0xFFFF);
#endif
#if	BOARD_STM32F407_DEV == 1
	HAL_UART_Transmit(&huart3, &dat, 1, 0xFFFF);
#endif
	
	//ITM_SendChar((uint32_t)ch);
	return ch;
}

//int fgetc(FILE *f) {
//  return (SER_GetChar());
//}


//int ferror(FILE *f) {
//  /* Your implementation of ferror */
//  return EOF;
//}


//void _ttywrch(int c) {
//  SER_PutChar(c);
//}


//void _sys_exit(int return_code) {
//label:  goto label;  /* endless loop */
//}
