/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_HARDWARERESET_H
#define __MY_HARDWARERESET_H

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
#include "misc.h"
#include "stm32f4xx_wwdg.h"
	 
/* Exported constants/macros -------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
	 
/* Exported function prototypes ----------------------------------------------*/
void UHRST_Configuration(void);
void UHRST_ResetCNT(void);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
