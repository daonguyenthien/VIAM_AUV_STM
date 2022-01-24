/* Includes ------------------------------------------------------------------*/
#include "my_io.h"

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   IO Pin define 
		*/
			#define 	UIO_LEDRED_PIN  				   	GPIO_Pin_12			
			#define 	UIO_LEDRED_PORT 				  	GPIOD
			#define 	UIO_LEDRED_CLK							RCC_AHB1Periph_GPIOD			
			#define 	UIO_LEDRED_SOURCE				 		GPIO_PinSource12				
			#define		UIO_LEDRED_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd	
			
			#define 	UIO_LEDGREEN_PIN  				   	GPIO_Pin_13			
			#define 	UIO_LEDGREEN_PORT 				  	GPIOD
			#define 	UIO_LEDGREEN_CLK							RCC_AHB1Periph_GPIOD			
			#define 	UIO_LEDGREEN_SOURCE				 		GPIO_PinSource13				
			#define		UIO_LEDGREEN_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd	
		
			#define 	UIO_LEDBLUE_PIN  				   	GPIO_Pin_14			
			#define 	UIO_LEDBLUE_PORT 				  	GPIOD
			#define 	UIO_LEDBLUE_CLK							RCC_AHB1Periph_GPIOD			
			#define 	UIO_LEDBLUE_SOURCE				 	GPIO_PinSource14				
			#define		UIO_LEDBLUE_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd	
			
			#define 	UIO_LEDORANGE_PIN  				   	GPIO_Pin_15			
			#define 	UIO_LEDORANGE_PORT 				  	GPIOD
			#define 	UIO_LEDORANGE_CLK							RCC_AHB1Periph_GPIOD			
			#define 	UIO_LEDORANGE_SOURCE				 	GPIO_PinSource15				
			#define		UIO_LEDORANGE_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd	
			
			#define 	UIO_LEAK1_PIN  				   	GPIO_Pin_11			
			#define 	UIO_LEAK1_PORT 				  	GPIOE
			#define 	UIO_LEAK1_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LEAK1_SOURCE				 	GPIO_PinSource11				
			#define		UIO_LEAK1_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LEAK1_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LEAK1_EXTI_PORT				EXTI_PinSource11
			#define		UIO_LEAK1_EXTI_Line				EXTI_Line11
			#define		UIO_LEAK1_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LEAK1_IRQn						EXTI15_10_IRQn
			
			#define 	UIO_LEAK2_PIN  				   	GPIO_Pin_12			
			#define 	UIO_LEAK2_PORT 				  	GPIOE
			#define 	UIO_LEAK2_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LEAK2_SOURCE				 	GPIO_PinSource12				
			#define		UIO_LEAK2_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LEAK2_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LEAK2_EXTI_PORT				EXTI_PinSource12
			#define		UIO_LEAK2_EXTI_Line				EXTI_Line12
			#define		UIO_LEAK2_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LEAK2_IRQn						EXTI15_10_IRQn
			
			#define 	UIO_LEAK3_PIN  				   	GPIO_Pin_13			
			#define 	UIO_LEAK3_PORT 				  	GPIOE
			#define 	UIO_LEAK3_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LEAK3_SOURCE				 	GPIO_PinSource13				
			#define		UIO_LEAK3_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LEAK3_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LEAK3_EXTI_PORT				EXTI_PinSource13
			#define		UIO_LEAK3_EXTI_Line				EXTI_Line13
			#define		UIO_LEAK3_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LEAK3_IRQn						EXTI15_10_IRQn
			
			#define 	UIO_LEAK4_PIN  				   	GPIO_Pin_14			
			#define 	UIO_LEAK4_PORT 				  	GPIOE
			#define 	UIO_LEAK4_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LEAK4_SOURCE				 	GPIO_PinSource14				
			#define		UIO_LEAK4_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LEAK4_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LEAK4_EXTI_PORT				EXTI_PinSource14
			#define		UIO_LEAK4_EXTI_Line				EXTI_Line14
			#define		UIO_LEAK4_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LEAK4_IRQn						EXTI15_10_IRQn
			
			#define 	UIO_LEAK5_PIN  				   	GPIO_Pin_15			
			#define 	UIO_LEAK5_PORT 				  	GPIOE
			#define 	UIO_LEAK5_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LEAK5_SOURCE				 	GPIO_PinSource15				
			#define		UIO_LEAK5_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LEAK5_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LEAK5_EXTI_PORT				EXTI_PinSource15
			#define		UIO_LEAK5_EXTI_Line				EXTI_Line15
			#define		UIO_LEAK5_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LEAK5_IRQn						EXTI15_10_IRQn
			#define 	UIO_EXTI15_10_IRQHandler			EXTI15_10_IRQHandler
			
			// Buzzer
			#define 	UIO_BUZZER_PIN  				   	GPIO_Pin_15			
			#define 	UIO_BUZZER_PORT 				  	GPIOC
			#define 	UIO_BUZZER_CLK							RCC_AHB1Periph_GPIOC			
			#define 	UIO_BUZZER_SOURCE				 		GPIO_PinSource15				
			#define		UIO_BUZZER_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			

/* Private variables ---------------------------------------------------------*/
typedef enum
{
	LEAK_POSITION_1 = 0x01,
	LEAK_POSITION_2 = 0x02,
	LEAK_POSITION_3 = 0x04,
	LEAK_POSITION_4 = 0x08,
	LEAK_POSITION_5 = 0x10,
}Leak_PositionTypeDef;

static uint8_t _leak_position = 0x00;

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/
void UIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//LED STATUS
	UIO_LEDRED_CLK_Cmd(UIO_LEDRED_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEDRED_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEDRED_PORT,&GPIO_InitStruct);
	GPIO_ResetBits(UIO_LEDRED_PORT, UIO_LEDRED_PIN);
	
	UIO_LEDGREEN_CLK_Cmd(UIO_LEDGREEN_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEDGREEN_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEDGREEN_PORT,&GPIO_InitStruct);
	GPIO_ResetBits(UIO_LEDGREEN_PORT, UIO_LEDGREEN_PIN);
	
	UIO_LEDBLUE_CLK_Cmd(UIO_LEDBLUE_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEDBLUE_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEDBLUE_PORT,&GPIO_InitStruct);
	GPIO_ResetBits(UIO_LEDBLUE_PORT, UIO_LEDBLUE_PIN);
	
	UIO_LEDORANGE_CLK_Cmd(UIO_LEDORANGE_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEDORANGE_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEDORANGE_PORT,&GPIO_InitStruct);
	GPIO_ResetBits(UIO_LEDORANGE_PORT, UIO_LEDORANGE_PIN);
	
//	//LEAK 1
//	UIO_LEAK1_CLK_Cmd(UIO_LEAK1_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_LEAK1_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_LEAK1_PORT,&GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(UIO_LEAK1_EXTI_SOURCE,UIO_LEAK1_EXTI_PORT);
//	
//	EXTI_InitStruct.EXTI_Line = UIO_LEAK1_EXTI_Line;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK1_EXTI_Trigger;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK1_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);
//	
//	//LEAK 2
//	UIO_LEAK2_CLK_Cmd(UIO_LEAK2_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_LEAK2_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_LEAK2_PORT,&GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(UIO_LEAK2_EXTI_SOURCE,UIO_LEAK2_EXTI_PORT);
//	
//	EXTI_InitStruct.EXTI_Line = UIO_LEAK2_EXTI_Line;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK2_EXTI_Trigger;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);
//	
//	//LEAK 3
//	UIO_LEAK3_CLK_Cmd(UIO_LEAK3_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_LEAK3_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_LEAK3_PORT,&GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(UIO_LEAK3_EXTI_SOURCE,UIO_LEAK3_EXTI_PORT);
//	
//	EXTI_InitStruct.EXTI_Line = UIO_LEAK3_EXTI_Line;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK3_EXTI_Trigger;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK3_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);
//	
//	//LEAK 4
//	UIO_LEAK4_CLK_Cmd(UIO_LEAK4_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_LEAK4_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_LEAK4_PORT,&GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(UIO_LEAK4_EXTI_SOURCE,UIO_LEAK4_EXTI_PORT);
//	
//	EXTI_InitStruct.EXTI_Line = UIO_LEAK4_EXTI_Line;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK4_EXTI_Trigger;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);
//	
//	//LEAK 5
//	UIO_LEAK5_CLK_Cmd(UIO_LEAK5_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_LEAK5_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_LEAK5_PORT,&GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(UIO_LEAK5_EXTI_SOURCE,UIO_LEAK5_EXTI_PORT);
//	
//	EXTI_InitStruct.EXTI_Line = UIO_LEAK5_EXTI_Line;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK5_EXTI_Trigger;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK5_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);
	
	//Emergency Case	
	UIO_BUZZER_CLK_Cmd(UIO_BUZZER_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_BUZZER_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_BUZZER_PORT,&GPIO_InitStruct);
	
	GPIO_ResetBits(UIO_BUZZER_PORT, UIO_BUZZER_PIN);
}

void UIO_LEDRED_ON(void)
{
	GPIO_SetBits(UIO_LEDRED_PORT, UIO_LEDRED_PIN);
}

void UIO_LEDRED_OFF(void)
{
	GPIO_ResetBits(UIO_LEDRED_PORT, UIO_LEDRED_PIN);
}

void UIO_LEDGREEN_ON(void)
{
	GPIO_SetBits(UIO_LEDGREEN_PORT, UIO_LEDGREEN_PIN);
}

void UIO_LEDGREEN_OFF(void)
{
	GPIO_ResetBits(UIO_LEDGREEN_PORT, UIO_LEDGREEN_PIN);
}

void UIO_LEDBLUE_ON(void)
{
	GPIO_SetBits(UIO_LEDBLUE_PORT, UIO_LEDBLUE_PIN);
}

void UIO_LEDBLUE_OFF(void)
{
	GPIO_ResetBits(UIO_LEDBLUE_PORT, UIO_LEDBLUE_PIN);
}

void UIO_LEDORANGE_ON(void)
{
	GPIO_SetBits(UIO_LEDORANGE_PORT, UIO_LEDORANGE_PIN);
}

void UIO_LEDORANGE_OFF(void)
{
	GPIO_ResetBits(UIO_LEDORANGE_PORT, UIO_LEDORANGE_PIN);
}

void UIO_LEDORANGE_TOGGLE(void)
{
	GPIO_ToggleBits(UIO_LEDORANGE_PORT, UIO_LEDORANGE_PIN);
}

void UIO_Buzzer(FunctionalState NewState)
{
	(NewState == ENABLE)? GPIO_SetBits(UIO_BUZZER_PORT, UIO_BUZZER_PIN): GPIO_ResetBits(UIO_BUZZER_PORT, UIO_BUZZER_PIN);
}

uint8_t UIO_LeakSensorPosition(void)
{
	return _leak_position;
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void UIO_EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(UIO_LEAK1_EXTI_Line) != RESET)
	{
		_leak_position  |= LEAK_POSITION_1;
		EXTI_ClearITPendingBit(UIO_LEAK1_EXTI_Line);
	}	
	
	if(EXTI_GetITStatus(UIO_LEAK2_EXTI_Line) != RESET)
	{
		_leak_position  |= LEAK_POSITION_2;
		EXTI_ClearITPendingBit(UIO_LEAK2_EXTI_Line);
	}	
	
	if(EXTI_GetITStatus(UIO_LEAK3_EXTI_Line) != RESET)
	{
		_leak_position  |= LEAK_POSITION_3;
		EXTI_ClearITPendingBit(UIO_LEAK3_EXTI_Line);
	}	
	
	if(EXTI_GetITStatus(UIO_LEAK4_EXTI_Line) != RESET)
	{
		_leak_position  |= LEAK_POSITION_4;
		EXTI_ClearITPendingBit(UIO_LEAK4_EXTI_Line);
	}
	
	if(EXTI_GetITStatus(UIO_LEAK5_EXTI_Line) != RESET)
	{
		_leak_position  |= LEAK_POSITION_5;
		EXTI_ClearITPendingBit(UIO_LEAK5_EXTI_Line);
	}	
}
