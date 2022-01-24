/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_BRAKE_H
#define __USER_BRAKE_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"	 
#include "stm32f10x_tim.h"
/* Exported constants/macros -------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void UBRAKE_Configuration(void);	 
void UBRAKE_Enable(void);
void UBRAKE_Disable(void);
	 
#ifdef __cplusplus
}
#endif

#endif 
