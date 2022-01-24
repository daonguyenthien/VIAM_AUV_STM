/********************************************************************************
 *               		                                                            *
 *  Project    		UGV_ARM																												*
 *	File      		UGV_SBUS.h     	                                              *
 * 	Author	    	Le Phuc Cong Thanh		                                        *
 *  Created on: 	Feb 26, 2021	                                                *
 *  Version 			V1.0	                                                       	*
 *                                                                   		        *
 ********************************************************************************/
#ifndef UGV_SBUS_h
#define UGV_SBUS_h

#include "stm32f4xx.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "UGV_UART_JOYSTICK.h"

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

/******************************************************************************
 *                                                                            *
 *  												FUNCTION	                                        *
 *  														                                              *
 ******************************************************************************/

bool SBUS_read(mybuffer *_sbuff, uint16_t* channels, int16_t* scalechannels, bool* failsafe, bool* lostFrame);
void clearScaleSbus(int16_t* _scalechannels,uint16_t* _channels);

#endif 










