/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_ADC_H
#define __USER_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"	 


	 
/* Exported constants/macros -------------------------------------------------*/
#define Temp_On_Chip				0
#define iMotor							2
#define Vref								3
#define Temp_LM35						1


 /**
 * @defgroup <Group name here>
 * @{
 */	
 
 /**
 * @}
 */	 
/* Exported types ------------------------------------------------------------*/
	


	 
/* Exported function prototypes ----------------------------------------------*/
void UADC_GPIO_Configure(void);
void UADC_ADC_DMA_Configure(void);
uint8_t UADC_GetBytesConverted(uint8_t Channel, uint8_t byte);
uint16_t UADC_GetAvrValue(uint8_t Channel);
float UADC_GetValue(uint8_t Channel);
/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
