/* Includes ------------------------------------------------------------------*/
#include "my_hardwarereset.h"

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   UHRST Pin define 
		*/
			#define 	UHRST_CLK									RCC_APB1Periph_WWDG
			#define		UHRST_CLK_Cmd		  	 			RCC_APB1PeriphClockCmd	
			#define 	UHRST_Prescaler						WWDG_Prescaler_8 //WDGTB = 3
			#define 	UHRST_WindowValue					80
			//Set Window value to 80
			//WWG counter should be refreshed only when counter is below 80 and greater than 64 otherwise a reset will be generated
			//The refresh window is: ~780 * (127-80) = 37ms < refresh window < ~780 * 64 = 49.92ms
			#define   UHRST_WatchdogCounter			127
			#define   UHRST_IRQn								WWDG_IRQn
			#define		UHRST_IRQHandler					WWDG_IRQHandler
			//WWDG clock counter = (4096.2^3)/42Mhz ~ 780us
			

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/
void UHRST_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	NVIC_InitStruct.NVIC_IRQChannel = UHRST_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	UHRST_CLK_Cmd(UHRST_CLK, ENABLE);

	WWDG_SetPrescaler(UHRST_Prescaler);		
	
	WWDG_SetWindowValue(UHRST_WindowValue);	
	
	WWDG_Enable(UHRST_WatchdogCounter);																					//Set counter value to 127 (T[6:0]) => Timeout ~ 780us * 64 (T[5:0]) = 49.92ms
	WWDG_ClearFlag();																					//Remove pro wakeup interrput flag
	WWDG_EnableIT();																					//Open the window watchdog interrupt
}

void UHRST_ResetCNT(void)
{
	WWDG_SetCounter(UHRST_WatchdogCounter);
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
/**
  * @brief  If WWDG_Flag is cleared, program will reset after Timeout. Vice verse, program operate normally
  * @param  None
  * @retval None
  */
void UHRST_IRQHandler(void)
{
	WWDG_ClearFlag();											//Remove pre wakeup interrupt flag
	UHRST_ResetCNT();
//	GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
}
