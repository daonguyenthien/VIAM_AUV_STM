/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "delay.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  GPIO
  * @param  None
  * @retval None
  */
void LED_Init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef 									IC_GPIO_InitStruct;
	IC_GPIO_InitStruct.GPIO_Mode 			= GPIO_Mode_OUT;
	IC_GPIO_InitStruct.GPIO_OType 		= GPIO_OType_PP;
	IC_GPIO_InitStruct.GPIO_PuPd 			= GPIO_PuPd_NOPULL;
	IC_GPIO_InitStruct.GPIO_Speed			= GPIO_Speed_100MHz;
	IC_GPIO_InitStruct.GPIO_Pin 			= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &IC_GPIO_InitStruct);
}

void LED_DIM_Config(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_InitTypeDef 									IC_GPIO_InitStruct;
	IC_GPIO_InitStruct.GPIO_Mode 			= GPIO_Mode_AF;
	IC_GPIO_InitStruct.GPIO_OType 		= GPIO_OType_PP;
	IC_GPIO_InitStruct.GPIO_PuPd 			= GPIO_PuPd_NOPULL;
	IC_GPIO_InitStruct.GPIO_Speed			= GPIO_Speed_100MHz;
	IC_GPIO_InitStruct.GPIO_Pin 			= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &IC_GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	TIM_TimeBaseInitTypeDef 	TIM4_TimeBaseInitStruct;
	TIM4_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM4_TimeBaseInitStruct.TIM_Prescaler			= 83;
	TIM4_TimeBaseInitStruct.TIM_Period				= 999;
	TIM4_TimeBaseInitStruct.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM4_TimeBaseInitStruct);
	
	TIM_OCInitTypeDef					TIM4_OCInitStruct;
	TIM4_OCInitStruct.TIM_Pulse								= 0;
	TIM4_OCInitStruct.TIM_OutputState					= TIM_OutputState_Enable;
	TIM4_OCInitStruct.TIM_OCPolarity					= TIM_OCPolarity_Low;
	TIM4_OCInitStruct.TIM_OCIdleState					= TIM_OCIdleState_Reset;
	TIM4_OCInitStruct.TIM_OCMode							= TIM_OCMode_PWM2;
	
	
	
	
	
	TIM_OC1Init(TIM4, &TIM4_OCInitStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM4_OCInitStruct);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM4_OCInitStruct);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM4_OCInitStruct);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	
	TIM_Cmd(TIM4, ENABLE);
}

void LED_Toggle(LED_Index myLed){
	GPIO_ToggleBits(GPIOD,myLed);
}
void LED_SetBits(LED_Index myLed){
	GPIO_SetBits(GPIOD, myLed);
}
void LED_ResetBits(LED_Index myLed){
	GPIO_ResetBits(GPIOD, myLed);
}

void LED_DIM(void){
	for (int i = 0; i<1000; i++){
			TIM_SetCompare1(TIM4, i);
			TIM_SetCompare2(TIM4, i);
			TIM_SetCompare3(TIM4, i);
			TIM_SetCompare4(TIM4, i);
			delay_ms(1);
	}
		for (int i = 999; i>=0; i--){
			TIM_SetCompare1(TIM4, i);
			TIM_SetCompare2(TIM4, i);
			TIM_SetCompare3(TIM4, i);
			TIM_SetCompare4(TIM4, i);
			delay_ms(1);
	}
}
/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
