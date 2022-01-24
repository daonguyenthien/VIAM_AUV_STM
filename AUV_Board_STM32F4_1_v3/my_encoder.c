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
	* 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "my_encoder.h"
#include "my_led.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @defgroup Module Pin define
 * @{
 */
					/** 
					* @brief   Encoder [M]ass Shifter channel A Pin define 
					*/
					#define 	UENC_ENCM_A_PIN  									GPIO_Pin_4
					#define 	UENC_ENCM_A_PORT 									GPIOB
					#define 	UENC_ENCM_A_AF										GPIO_AF_TIM3
					#define 	UENC_ENCM_A_CLK										RCC_AHB1Periph_GPIOB
					#define 	UENC_ENCM_A_SOURCE								GPIO_PinSource4					
					#define		UENC_ENCM_A_CLK_Cmd		  					RCC_AHB1PeriphClockCmd
					
					
					/** 
					* @brief   Encoder [M]ass Shifter channel B Pin define 
					*/
					#define 	UENC_ENCM_B_PIN  									GPIO_Pin_5
					#define 	UENC_ENCM_B_PORT 									GPIOB
					#define 	UENC_ENCM_B_AF										GPIO_AF_TIM3
					#define 	UENC_ENCM_B_CLK										RCC_AHB1Periph_GPIOB
					#define 	UENC_ENCM_B_SOURCE								GPIO_PinSource5					
					#define		UENC_ENCM_B_CLK_Cmd		  					RCC_AHB1PeriphClockCmd
					
					/** 
					* @brief   Encoder [P]istol channel A Pin define 
					*/
					#define 	UENC_ENCP_A_PIN  									GPIO_Pin_12
					#define 	UENC_ENCP_A_PORT 									GPIOD
					#define 	UENC_ENCP_A_AF										GPIO_AF_TIM4
					#define 	UENC_ENCP_A_CLK										RCC_AHB1Periph_GPIOD
					#define 	UENC_ENCP_A_SOURCE								GPIO_PinSource12				
					#define		UENC_ENCP_A_CLK_Cmd		  					RCC_AHB1PeriphClockCmd
					
					/** 
					* @brief   Encoder [P]istol channel B Pin define 
					*/
					#define 	UENC_ENCP_B_PIN  									GPIO_Pin_13
					#define 	UENC_ENCP_B_PORT 									GPIOD
					#define 	UENC_ENCP_B_AF										GPIO_AF_TIM4
					#define 	UENC_ENCP_B_CLK										RCC_AHB1Periph_GPIOD
					#define 	UENC_ENCP_B_SOURCE								GPIO_PinSource13					
					#define		UENC_ENCP_B_CLK_Cmd		  					RCC_AHB1PeriphClockCmd
					
/**
 * @defgroup Encoder module define
 * @{
 */		
 
 
					/** 
					* @brief   Counter [M]ass Shifter Module define 
					*/
					#define 	UENC_ENCM_TIM  				      			TIM3
					#define 	UENC_ENCM_TIM_CLK					 	 			RCC_APB1Periph_TIM3	
					#define 	UENC_ENCM_TIM_CLK_Cmd    					RCC_APB1PeriphClockCmd
					//#define 	UENC_ENCM_TIM_IRQn    						xxx
					//#define 	UENC_ENCM_TIM_IRQHandler					xxx										
					#define 	UENC_ENCM_RESOLUTION							500.0f 
					#define 	UENC_ENCM_MODE										4.0f
					#define   UENC_ENCM_GEARBOX									14.0f
					#define 	UENC_ENCM_LSB											360.0f/(UENC_ENCM_RESOLUTION*UENC_ENCM_MODE)
					
					
					/** 
					* @brief   Counter [P]istol Module define 
					*/	
					#define 	UENC_ENCP_TIM  				      			TIM4
					#define 	UENC_ENCP_TIM_CLK					 	 			RCC_APB1Periph_TIM4	
					#define 	UENC_ENCP_TIM_CLK_Cmd    					RCC_APB1PeriphClockCmd
					//#define 	UENC_ENCP_TIM_IRQn    						xxx
					//#define 	UENC_ENCP_TIM_IRQHandler					xxx			
					#define 	UENC_ENCP_RESOLUTION							500.0f
					#define 	UENC_ENCP_MODE										4.0f
					#define   UENC_ENCP_GEARBOX									14.0f
					#define 	UENC_ENCP_LSB											360.0f/(UENC_ENCM_RESOLUTION*UENC_ENCM_MODE)
					
					/** 
					* @brief   Timer Module define 
					*/					
					#define 	UENC_TIM  				      			TIM7
					#define 	UENC_TIM_CLK					 	 			RCC_APB1Periph_TIM7	
					#define 	UENC_TIM_CLK_Cmd    					RCC_APB1PeriphClockCmd
					#define 	UENC_TIM_IRQn    							TIM7_IRQn				
					#define		UENC_TIM_PreemptionPriority		0x00
					#define		UENC_TIM_SubPriority					0x05		
					
					#define 	UENC_TIM_IRQHandler						TIM7_IRQHandler	
					

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/			//Add those global variable here.
//MotorInitTypeDef motor1;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Config Encoder
  * @param  Sampling time in miliseconds
  * @retval None
  */
void UENC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//Configure for Mass Shifter Encoder
	UENC_ENCM_A_CLK_Cmd(UENC_ENCM_A_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UENC_ENCM_A_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UENC_ENCM_A_PORT,&GPIO_InitStruct);
	
	UENC_ENCM_B_CLK_Cmd(UENC_ENCM_B_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UENC_ENCM_B_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UENC_ENCM_B_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UENC_ENCM_A_PORT, UENC_ENCM_A_SOURCE, UENC_ENCM_A_AF);
	GPIO_PinAFConfig(UENC_ENCM_B_PORT, UENC_ENCM_B_SOURCE, UENC_ENCM_B_AF);
	
	UENC_ENCM_TIM_CLK_Cmd(UENC_ENCM_TIM_CLK, ENABLE);	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);	
	TIM_TimeBaseInit(UENC_ENCM_TIM, &TIM_TimeBaseStruct);			
	TIM_EncoderInterfaceConfig(UENC_ENCM_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(UENC_ENCM_TIM, DISABLE);
	
	//Configure for Pistol Encoder
	UENC_ENCP_A_CLK_Cmd(UENC_ENCP_A_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UENC_ENCP_A_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UENC_ENCP_A_PORT,&GPIO_InitStruct);
	
	UENC_ENCP_B_CLK_Cmd(UENC_ENCP_B_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UENC_ENCP_B_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UENC_ENCP_B_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UENC_ENCP_A_PORT, UENC_ENCP_A_SOURCE, UENC_ENCP_A_AF);
	GPIO_PinAFConfig(UENC_ENCP_B_PORT, UENC_ENCP_B_SOURCE, UENC_ENCP_B_AF);
	
	UENC_ENCP_TIM_CLK_Cmd(UENC_ENCP_TIM_CLK, ENABLE);	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);	
	TIM_TimeBaseInit(UENC_ENCP_TIM, &TIM_TimeBaseStruct);			
	TIM_EncoderInterfaceConfig(UENC_ENCP_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(UENC_ENCP_TIM, DISABLE);
	
	UENC_TIM_CLK_Cmd(UENC_TIM_CLK, ENABLE);	
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_Period = 840 - 1;
	TIM_TimeBaseStruct.TIM_Prescaler = 1000 - 1;
	TIM_TimeBaseInit(UENC_TIM, &TIM_TimeBaseStruct);
	
	TIM_ITConfig(UENC_TIM, TIM_IT_Update, DISABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = UENC_TIM_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UENC_TIM_PreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UENC_TIM_SubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_Cmd(UENC_TIM, DISABLE);
}


void UENC_StartEncoder(void)
{
	TIM_Cmd(UENC_ENCM_TIM, ENABLE);
	TIM_Cmd(UENC_ENCP_TIM, ENABLE);
	TIM_ITConfig(UENC_TIM, TIM_IT_Update, ENABLE);
	TIM_Cmd(UENC_TIM, ENABLE);
}



void UENC_StopEncoder(void)
{
	TIM_Cmd(UENC_ENCM_TIM, DISABLE);
	TIM_Cmd(UENC_ENCP_TIM, DISABLE);
	TIM_SetCounter(UENC_ENCM_TIM,0);
	TIM_SetCounter(UENC_ENCP_TIM,0);
}


/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void UENC_TIM_IRQHandler(void)
{
	if(TIM_GetITStatus(UENC_TIM, TIM_IT_Update) != RESET)
	{

		TIM_ClearITPendingBit(UENC_TIM, TIM_IT_Update);
	}
}


/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
