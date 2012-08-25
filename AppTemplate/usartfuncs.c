/*
 * usartfuncs.c
 *
 *  Created on: 19.08.2012
 *      Author: diego
 */

#include "hwsetup.h"
#include <stdio.h>
#include <stm32f10x_usart.h>
//#include <stm32_eval.h>

USART_InitTypeDef USART_InitStructure;

void init_usart()
{
	//while(1);
	//System_Setup();
	/* USARTx configured as follow:
	- BaudRate = 115200 baud
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//
	STM_EVAL_COMInit(COM1, &USART_InitStructure);

	printf("USART started... \n");

}
