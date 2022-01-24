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
typedef enum
{
	CAM_POWER_ON		= 0x02,
	CAM_POWER_OFF		= 0x03,
}UCAM_Power_TypeDef;

typedef enum
{
	CAM_ZOOM_STOP		= 0x00,
	CAM_ZOOM_TELE		= 0x02,
	CAM_ZOOM_WIDE		= 0x03,
}CAM_ZOOM_TypeDef;

typedef enum
{
	CAM_BRIGHT_RESET		= 0x00,
	CAM_BRIGHT_UP			= 0x02,
	CAM_BRIGHT_DOWN		= 0x03,
}CAM_BRIGHT_TypeDef;

typedef enum
{
	CAM_FOCUS_AUTO		= 0x02,
	CAM_FOCUS_MANUAL		= 0x03,
	CAM_FOCUS_BOTH		= 0x10,
	CAM_FOCUS_FAR		= 0x20,
	CAM_FOCUS_NEAR		= 0x30,
	CAM_FOCUS_STOP		= 0x00,
}CAM_FOCUS_TypeDef;

/* Exported types ------------------------------------------------------------*/
	 
/* Exported function prototypes ----------------------------------------------*/
void UCAM_Configuration(void);
void UCAM_Focus_SetUp(uint8_t _limit);
void UCAM_Focus_Mode(CAM_FOCUS_TypeDef _mode);
void UCAM_Bright(CAM_BRIGHT_TypeDef _bright);
void UCAM_Zoom(CAM_ZOOM_TypeDef _standard);
void UCAM_Power(UCAM_Power_TypeDef _state);
void UCAM_IF_Clear(void);
void UCAM_AddressSet(uint8_t _id);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
