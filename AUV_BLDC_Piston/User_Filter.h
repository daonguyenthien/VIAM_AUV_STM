/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_FILTER_H
#define __USER_FILTER_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"	 
	 
/* Exported constants/macros -------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
	 
typedef struct{
	float a;
	float b;
} FirstOrderParameter;
	 
typedef struct{
	float a;
	float b;
	float c;
	float d;
} SecondOrderParameter;
/* Exported function prototypes ----------------------------------------------*/

float UFIL_1stOrderFilter_Output(FirstOrderParameter*, float);
float UFIL_2ndOrderFilter_Output(SecondOrderParameter*, float);
	 
#ifdef __cplusplus
}
#endif

#endif 