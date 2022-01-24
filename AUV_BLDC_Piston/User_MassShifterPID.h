/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_MASSSHIFTERPID_H
#define __USER_MASSSHIFTERPID_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"	 
#include "stm32f10x_tim.h"
/* Exported constants/macros -------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void UMSPID_Configuration(void);
void UMSPID_Calculate(uint16_t _setPoint); 
	 
#ifdef __cplusplus
}
#endif

#endif 
