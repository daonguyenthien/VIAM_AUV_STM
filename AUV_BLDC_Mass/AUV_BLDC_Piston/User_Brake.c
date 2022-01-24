/* Includes ------------------------------------------------------------------*/
#include "User_Brake.h"
/* Public variables ----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private const/macros ------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void UBRAKE_Configuration(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef UENC_GPIOInitStruct;
	UENC_GPIOInitStruct.GPIO_Pin = GPIO_Pin_2;
	UENC_GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	UENC_GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &UENC_GPIOInitStruct);
	TIM_TimeBaseInitTypeDef UBRAKE_TimeBaseInitStruct;
	UBRAKE_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	UBRAKE_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	UBRAKE_TimeBaseInitStruct.TIM_Prescaler = 35;
	UBRAKE_TimeBaseInitStruct.TIM_Period    = 1000;
	UBRAKE_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &UBRAKE_TimeBaseInitStruct);
	
	TIM_OCInitTypeDef UBRAKE_OCInitStruct;
	UBRAKE_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	UBRAKE_OCInitStruct.TIM_OCPolarity	= TIM_OCPolarity_High;
	UBRAKE_OCInitStruct.TIM_OutputState	= TIM_OutputState_Enable;
	UBRAKE_OCInitStruct.TIM_OCIdleState	= TIM_OCIdleState_Reset;
	UBRAKE_OCInitStruct.TIM_Pulse 			= 0;
	TIM_OC1Init(TIM5, &UBRAKE_OCInitStruct);
	TIM_Cmd(TIM5,ENABLE);
}

void UBRAKE_Enable(void){
	TIM_SetCompare3(TIM2, 30000);
}

void UBRAKE_Disable(void){
	TIM_SetCompare3(TIM2, 0);
}
