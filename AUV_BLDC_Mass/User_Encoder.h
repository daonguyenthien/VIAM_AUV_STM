/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_ENCODER_H
#define __USER_ENCODER_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"	 
#include "stm32f10x_tim.h"
/* Exported constants/macros -------------------------------------------------*/
extern __IO uint16_t revolution_count;

union r_Position
{
	float Value;
	
	struct
	{
		unsigned a1:8;
		unsigned a2:8;
		unsigned a3:8;
		unsigned a4:8;
	} byte;
};

extern union r_Position UENC_ActualPosition;


/* Exported types ------------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void UENC_Configure(void);	 
uint32_t UENC_GetCounterValue(void);
uint32_t UENCmm2pulse(float _distance);
void UENC_ResetCounter(void);
float UENC_GetActualPosition(void);
uint8_t UENC_GetBytesActualPosition(uint8_t byte);
float UENC_GetActualSpeed(void);
uint8_t UENC_GetBytesActualSpeed(uint8_t byte);
float UENCpulse2mm(uint32_t _pulse);

float UENC_GetSpeed(void);
	 
#ifdef __cplusplus
}
#endif

#endif 
