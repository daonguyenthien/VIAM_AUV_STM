/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_ENCODER_H
#define __MY_ENCODER_H

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
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h" 
#include "misc.h"
	 
	 
/* Exported constants/macros -------------------------------------------------*/
typedef struct
{
	float Mass_Encoder;
	float	Pistol_Encoder;
}Encoder_StatusTypeDef;

/** 
* @brief   Real output data collection typedef 
*/	
	 
/* Exported types ------------------------------------------------------------*/
	 
/* Exported function prototypes ----------------------------------------------*/
void UENC_Configuration(void);
void UENC_StartEncoder(void);
void UENC_StopEncoder(void);
float UENC_GetNbrRoundMass(void);
float UENC_GetNbrRoundPistol(void);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
