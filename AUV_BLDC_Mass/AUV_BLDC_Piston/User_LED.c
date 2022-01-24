/* Includes ------------------------------------------------------------------*/
#include "User_LED.h"

/* External functions define -------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/


/* Private const/macros ------------------------------------------------------*/


/**
 * @}
 */
 

/* Private variables ---------------------------------------------------------*/
static bool state_BLUE = false, state_GREEN = false, state_ORANGE = false;
/* Private function prototypes -----------------------------------------------*/


/* Exported function body ----------------------------------------------------*/

void ULED_Configure(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitTypeDef LED_GPIOInitStruct;
	LED_GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	LED_GPIOInitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	LED_GPIOInitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &LED_GPIOInitStruct);
	
	LED_GPIOInitStruct.GPIO_Pin = GPIO_Pin_15;
	
	GPIO_Init(GPIOA, &LED_GPIOInitStruct);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}

void ULED_Toggle(LED_Index myLed){
	if(myLed == ORANGE){
		GPIO_WriteBit(GPIOA, myLed, state_ORANGE);
		state_ORANGE = !state_ORANGE;
	}
	if(myLed == GREEN) 
	{
		GPIO_WriteBit(GPIOB, myLed, state_GREEN);
		state_GREEN = !state_GREEN;
	}
	if(myLed == BLUE)
	{
		GPIO_WriteBit(GPIOB, myLed, state_BLUE);
		state_BLUE = !state_BLUE;
	}
}

void ULED_Write(LED_Index myLed, bool status){
	if (myLed == ORANGE){
		GPIO_WriteBit(GPIOA, GPIO_Pin_15, status);
	}
	else {
		GPIO_WriteBit(GPIOB, myLed, status);
	}
}



/* Private functions body ----------------------------------------------------*/


