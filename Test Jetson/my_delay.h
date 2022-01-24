/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_DELAY_H
#define __MY_DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"	 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

/* Exported constants/macros -------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
	 

	 
/* Exported function prototypes ----------------------------------------------*/
void UDELAY_Configuration(void);	 

void UDELAY_ms(uint32_t ms);
void UDELAY_us(uint32_t us);
	 
#ifdef __cplusplus
}
#endif

#endif 
