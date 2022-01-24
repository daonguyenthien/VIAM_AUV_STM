/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_TRAINGAUGE_H
#define __MY_TRAINGAUGE_H

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
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h" 
#include "misc.h"

/* Exported constants/macros -------------------------------------------------*/
typedef struct
{
	float Strain_Gauge_P1;
	float Strain_Gauge_P2;
	float Strain_Gauge_P3;
	float Strain_Gauge_P4;
	float Strain_Gauge_P5;
	float Strain_Gauge_P6;
	float Strain_Gauge_P7;
	float Strain_Gauge_P8;
}Strain_Gauge_StatusTypeDef;
/* Exported types ------------------------------------------------------------*/
	 
/* Exported function prototypes ----------------------------------------------*/
void USTR_Configuration(void);
void USTR_GetStatus(Strain_Gauge_StatusTypeDef *_strain_gauge_st);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
