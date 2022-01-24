/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_BMS24V10AH_H
#define __MY_BMS24V10AH_H

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

/* Exported constants/macros -------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;
	float BatteryTotal; //mAh
	float BatteryCapacity; // mAh
	float BatteryUsed; // mAh
	float BatteryPercentage; // %
	float BatteryCurrent; // mA
	float BaterryVoltage; // V	
}UBMS10_Status_Typedef;

/* Exported function prototypes ----------------------------------------------*/
void UBMS10_Configuration(void);
void UBMS10_InitBattery(uint8_t _hours, uint8_t _minutes, uint8_t _seconds, float _battery_total);
void UBMS10_PassConfig(void);
void UBMS10_GetStatus(UBMS10_Status_Typedef *_ubms10_status);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
