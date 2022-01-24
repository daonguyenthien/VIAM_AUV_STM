/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_CAMERA_H
#define __MY_CAMERA_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"	 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h" 
#include "misc.h"
#include "main.h"
	 
	 
/* Exported constants/macros -------------------------------------------------*/

/** 
* @brief   Real output data collection typedef 
*/	
	 
/* Exported types ------------------------------------------------------------*/
	 
/* Exported function prototypes ----------------------------------------------*/
void UCAM_Configuration(void);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
