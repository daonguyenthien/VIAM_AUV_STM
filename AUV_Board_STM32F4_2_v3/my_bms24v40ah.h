/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_BMS24V40AH_H
#define __MY_BMS24V40AH_H

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
#include "my_delay.h"
	 
/* Exported constants/macros -------------------------------------------------*/
//typedef struct
//{
//	uint8_t Year;
//	uint8_t Month;
//	uint8_t Day;
//	uint8_t Hours;
//	uint8_t Minutes;
//	uint8_t Seconds;
//	float SoC; //%
//	float Cell_1; // mAh
//	float Cell_2; // mAh
//	float Cell_3; // mAh
//	float Cell_4; // mAh
//	float Cell_5; // mAh
//	float Cell_6; // mAh
//	float Cell_7; // mAh	
//	float Cell_8; // mAh	
//	float IT;
//	float ET;
//	float Sign_BattCurrent;
//	float BattCurrentAbs;
//	float PV1;
//	float PV2;
//	float EXT;
//	float AD2;
//	float AD3;
//	float AD4;
//	float Heat_1;
//	float Heat_2;
//	uint32_t Error;
//}UBMS40_Status_Typedef;
typedef struct
{
//	uint8_t Hours;
//	uint8_t Minutes;
//	uint8_t Seconds;
	float BatteryTotal; //mAh
	float BatteryCapacity; // mAh
	float BatteryUsed; // mAh
	float BatteryPercentage; // %
	float BatteryCurrent; // mA
	float BaterryVoltage; // V	
}UBMS40_Status_Typedef;

/* Exported types ------------------------------------------------------------*/
	 
/* Exported function prototypes ----------------------------------------------*/
void UBMS40_Configuration(void);
void UBMS40_InitBattery(uint8_t _hours, uint8_t _minutes, uint8_t _seconds, float _battery_total);
void UBMS40_PassConfig(void);
void UBMS40_GetStatus(UBMS40_Status_Typedef *_ubms40_status);
void UBMS40_Cmd(FunctionalState NewState);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
