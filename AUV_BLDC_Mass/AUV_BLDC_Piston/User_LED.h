/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_LED_H
#define __USER_LED_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stdbool.h"
/*Defines*/
/* Exported types ------------------------------------------------------------*/
typedef enum {BLUE = GPIO_Pin_4, GREEN = GPIO_Pin_3, ORANGE = GPIO_Pin_15} LED_Index;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported variables*/

void ULED_Configure(void);
void ULED_Toggle(LED_Index myLed);
void ULED_Write(LED_Index myLed, bool status);

#endif /* __LED_H */
