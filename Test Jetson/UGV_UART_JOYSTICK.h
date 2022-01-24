/********************************************************************************
 *               		                                                            *
 *  Project    		UGV_ARM				                                          			*
 *	File      		UGV_UART.h     	                                   						*
 * 	Author	    	Le Phuc Cong Thanh		                                        *
 *  Created on: 	Feb 26, 2021	                                                *
 *  Version 			V1.0	                                                       	*
 *  Descrption: 	Use USART3 to read Joystick                                   *
 *               		                                                            *
 ********************************************************************************/

 
#ifndef UGV_UART_H
#define UGV_UART_H

#include "stdint.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include <string.h>
/******************************************************************************
 *                                                                            *
 *  												DEFINE  	                                        *
 *  														                                              *
 ******************************************************************************/

/******************************************************************************
 *                                                                            *
 *  												TYPEDEFINE                                        *
 *  														                                              *
 ******************************************************************************/

typedef struct _uart_buffer_
	{
		 uint16_t counter;
		 bool flag;
		 uint8_t mbuff_rx[100];
		 uint8_t mbuff_tx[100];
	} mybuffer;	

/******************************************************************************
 *                                                                            *
 *  												FUNCTION	                                        *
 *  														                                              *
 ******************************************************************************/
extern int mode_manual;
extern uint16_t channels[16];
extern int16_t scalechannels[16];
extern int16_t old_scalechannels[16];
extern bool failSafe;
extern bool lostFrame;
extern mybuffer uart_buffer_u3;
extern uint8_t Send_count;			
extern int mode_manual;
	
void USART6_DMA_Rx_Config(void);
void Manual_mode (void);
	
#endif 
