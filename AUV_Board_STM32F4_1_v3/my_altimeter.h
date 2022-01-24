/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_ALTIMETER_H
#define __MY_ALTIMETER_H

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
typedef struct
{
	float ALTI_in_feet;
	float ALTI_in_metres;
	float ALTI_in_fathoms;
}Altimeter_StatusTypeDef;

union Altimeter_Sensor
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

extern union Altimeter_Sensor UALTI_in_feet, UALTI_in_metres, UALTI_in_fathoms;

/* Exported types ------------------------------------------------------------*/
	 
/* Exported function prototypes ----------------------------------------------*/
void UALTI_Configuration(void);
void UALTI_Transmit(char *szData);
void UALTI_GetStatus(Altimeter_StatusTypeDef *_status_data);
uint32_t UALTI_GetAddressMemory(void);
uint8_t UALTI_Checksum(uint8_t *_data);
uint8_t UALTI_Convert2Int(uint8_t _data);
void UALTI_ArrangeData(uint8_t *_data_in, uint8_t *_data_out);
void UALTI_ConvertData(uint8_t *_data, float* _in_feet, float* _in_metres, float* _in_fathoms);
uint8_t UALTI_GetByteData(uint8_t _type, uint8_t _num_byte);
float UALTI_GetData(uint8_t _type);


/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
