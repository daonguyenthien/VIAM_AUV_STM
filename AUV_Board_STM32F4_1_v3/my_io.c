/* Includes ------------------------------------------------------------------*/
#include "my_io.h"
#include "my_pmcan.h"
#include "my_syscan.h"

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
			
			#define 	UIO_LIGHT_PIN  				   	GPIO_Pin_9			
			#define 	UIO_LIGHT_PORT 				  	GPIOE
			#define 	UIO_LIGHT_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LIGHT_SOURCE				 	GPIO_PinSource9				
			#define		UIO_LIGHT_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
		
			
			#define 	UIO_LS1_PIN  				   	GPIO_Pin_2			
			#define 	UIO_LS1_PORT 				  	GPIOE
			#define 	UIO_LS1_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LS1_SOURCE				 	GPIO_PinSource2				
			#define		UIO_LS1_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LS1_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LS1_EXTI_PORT				EXTI_PinSource2
			#define		UIO_LS1_EXTI_Line				EXTI_Line2
			#define		UIO_LS1_EXTI_Trigger		EXTI_Trigger_Rising
			//Interrupt
			#define		UIO_LS1_IRQn						EXTI2_IRQn
			#define 	UIO_LS1_IRQHandler			EXTI2_IRQHandler
			
			#define 	UIO_LS2_PIN  				   	GPIO_Pin_3			
			#define 	UIO_LS2_PORT 				  	GPIOE
			#define 	UIO_LS2_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LS2_SOURCE				 	GPIO_PinSource3				
			#define		UIO_LS2_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LS2_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LS2_EXTI_PORT				EXTI_PinSource3
			#define		UIO_LS2_EXTI_Line				EXTI_Line3
			#define		UIO_LS2_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LS2_IRQn						EXTI3_IRQn
			#define 	UIO_LS2_IRQHandler			EXTI3_IRQHandler
			
			#define 	UIO_LS3_PIN  				   	GPIO_Pin_4			
			#define 	UIO_LS3_PORT 				  	GPIOE
			#define 	UIO_LS3_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LS3_SOURCE				 	GPIO_PinSource4				
			#define		UIO_LS3_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LS3_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LS3_EXTI_PORT				EXTI_PinSource4
			#define		UIO_LS3_EXTI_Line				EXTI_Line4
			#define		UIO_LS3_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LS3_IRQn						EXTI4_IRQn
			#define 	UIO_LS3_IRQHandler			EXTI4_IRQHandler
			
			#define 	UIO_LS4_PIN  				   	GPIO_Pin_5			
			#define 	UIO_LS4_PORT 				  	GPIOE
			#define 	UIO_LS4_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LS4_SOURCE				 	GPIO_PinSource5				
			#define		UIO_LS4_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LS4_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LS4_EXTI_PORT				EXTI_PinSource5
			#define		UIO_LS4_EXTI_Line				EXTI_Line5
			#define		UIO_LS4_EXTI_Trigger		EXTI_Trigger_Rising	
			//Interrupt
			#define		UIO_LS4_IRQn						EXTI9_5_IRQn
			#define 	UIO_EXTI9_5_IRQHandler	EXTI9_5_IRQHandler
//			
//			#define 	UIO_LS5_PIN  				   	GPIO_Pin_6			
//			#define 	UIO_LS5_PORT 				  	GPIOE
//			#define 	UIO_LS5_CLK							RCC_AHB1Periph_GPIOE			
//			#define 	UIO_LS5_SOURCE				 	GPIO_PinSource6				
//			#define		UIO_LS5_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
//			// EXTI
//			#define 	UIO_LS5_EXTI_SOURCE			EXTI_PortSourceGPIOE
//			#define		UIO_LS5_EXTI_PORT				EXTI_PinSource6
//			#define		UIO_LS5_EXTI_Line				EXTI_Line6
//			#define		UIO_LS5_EXTI_Trigger		EXTI_Trigger_Rising	
//			//Interrupt
//			#define		UIO_LS5_IRQn						EXTI9_5_IRQn
//			
//			#define 	UIO_LS6_PIN  				   	GPIO_Pin_13			
//			#define 	UIO_LS6_PORT 				  	GPIOC
//			#define 	UIO_LS6_CLK							RCC_AHB1Periph_GPIOC			
//			#define 	UIO_LS6_SOURCE				 	GPIO_PinSource13				
//			#define		UIO_LS6_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
//			// EXTI
//			#define 	UIO_LS6_EXTI_SOURCE			EXTI_PortSourceGPIOC
//			#define		UIO_LS6_EXTI_PORT				EXTI_PinSource13
//			#define		UIO_LS6_EXTI_Line				EXTI_Line13
//			#define		UIO_LS6_EXTI_Trigger		EXTI_Trigger_Falling	
//			//Interrupt
//			#define		UIO_LS6_IRQn						EXTI15_10_IRQn

			#define 	UIO_EXTI15_10_IRQHandler			EXTI15_10_IRQHandler
			
			#define 	UIO_LEAK1_PIN  				   	GPIO_Pin_11			
			#define 	UIO_LEAK1_PORT 				  	GPIOE
			#define 	UIO_LEAK1_CLK							RCC_AHB1Periph_GPIOE			
			#define 	UIO_LEAK1_SOURCE				 	GPIO_PinSource11				
			#define		UIO_LEAK1_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			// EXTI
			#define 	UIO_LEAK1_EXTI_SOURCE			EXTI_PortSourceGPIOE
			#define		UIO_LEAK1_EXTI_PORT				EXTI_PinSource11
			#define		UIO_LEAK1_EXTI_Line				EXTI_Line11
			#define		UIO_LEAK1_EXTI_Trigger		EXTI_Trigger_Falling	
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
			#define		UIO_LEAK2_EXTI_Trigger		EXTI_Trigger_Falling	
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
			#define		UIO_LEAK3_EXTI_Trigger		EXTI_Trigger_Falling	
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
			#define		UIO_LEAK4_EXTI_Trigger		EXTI_Trigger_Falling	
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
			#define		UIO_LEAK5_EXTI_Trigger		EXTI_Trigger_Falling	
			//Interrupt
			#define		UIO_LEAK5_IRQn						EXTI15_10_IRQn
			
			// Emergency Case
			#define 	UIO_EMER24V40AH_PIN  				   	GPIO_Pin_0			
			#define 	UIO_EMER24V40AH_PORT 				  	GPIOC
			#define 	UIO_EMER24V40AH_CLK							RCC_AHB1Periph_GPIOC			
			#define 	UIO_EMER24V40AH_SOURCE				 	GPIO_PinSource0				
			#define		UIO_EMER24V40AH_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			
			#define 	UIO_EMER24V10AH_PIN  				   	GPIO_Pin_1			
			#define 	UIO_EMER24V10AH_PORT 				  	GPIOC
			#define 	UIO_EMER24V10AH_CLK							RCC_AHB1Periph_GPIOC			
			#define 	UIO_EMER24V10AH_SOURCE				 	GPIO_PinSource1				
			#define		UIO_EMER24V10AH_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd	
			
			

/* Private variables ---------------------------------------------------------*/
//typedef enum
//{
//	LEAK_POSITION_1 = 0x01,
//	LEAK_POSITION_2 = 0x02,
//	LEAK_POSITION_3 = 0x04,
//	LEAK_POSITION_4 = 0x08,
//	LEAK_POSITION_5 = 0x10,
//}Leak_PositionTypeDef;

//uint8_t _leak_position = 0x00;

uint8_t LEAK_POSITION_1 = 0;
uint8_t LEAK_POSITION_2 = 0;
uint8_t LEAK_POSITION_3 = 0;
uint8_t LEAK_POSITION_4 = 0;
uint8_t LEAK_POSITION_5 = 0;

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
	
	//LIGHT
	UIO_LIGHT_CLK_Cmd(UIO_LIGHT_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LIGHT_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LIGHT_PORT,&GPIO_InitStruct);
	GPIO_ResetBits(UIO_LIGHT_PORT, UIO_LIGHT_PIN);
	
	UIO_Light_Cmd(DISABLE);
	
	//LIMIT SWITCH 1
	UIO_LS1_CLK_Cmd(UIO_LS1_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LS1_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LS1_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LS1_EXTI_SOURCE,UIO_LS1_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LS1_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LS1_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LS1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_Init(&NVIC_InitStruct);
	
	//LIMIT SWITCH 2
	UIO_LS2_CLK_Cmd(UIO_LS2_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LS2_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LS2_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LS2_EXTI_SOURCE,UIO_LS2_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LS2_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LS2_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LS2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x04;
	NVIC_Init(&NVIC_InitStruct);
	
	//LIMIT SWITCH 3
	UIO_LS3_CLK_Cmd(UIO_LS3_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LS3_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LS3_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LS3_EXTI_SOURCE,UIO_LS3_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LS3_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LS3_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LS3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x05;
	NVIC_Init(&NVIC_InitStruct);
	
	//LIMIT SWITCH 4
	UIO_LS4_CLK_Cmd(UIO_LS4_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LS4_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LS4_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LS4_EXTI_SOURCE,UIO_LS4_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LS4_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LS4_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LS4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x06;
	NVIC_Init(&NVIC_InitStruct);
	
//	//LIMIT SWITCH 5
//	UIO_LS5_CLK_Cmd(UIO_LS5_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_LS5_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_LS5_PORT,&GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(UIO_LS5_EXTI_SOURCE,UIO_LS5_EXTI_PORT);
//	
//	EXTI_InitStruct.EXTI_Line = UIO_LS5_EXTI_Line;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = UIO_LS5_EXTI_Trigger;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UIO_LS5_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);
//	
//	//LIMIT SWITCH 6
//	UIO_LS6_CLK_Cmd(UIO_LS6_CLK,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_LS6_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_LS6_PORT,&GPIO_InitStruct);
//	
//	SYSCFG_EXTILineConfig(UIO_LS6_EXTI_SOURCE,UIO_LS6_EXTI_PORT);
//	
//	EXTI_InitStruct.EXTI_Line = UIO_LS6_EXTI_Line;
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStruct.EXTI_Trigger = UIO_LS6_EXTI_Trigger;
//	EXTI_Init(&EXTI_InitStruct);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UIO_LS6_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);
	
	//LEAK 1
	UIO_LEAK1_CLK_Cmd(UIO_LEAK1_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEAK1_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEAK1_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LEAK1_EXTI_SOURCE,UIO_LEAK1_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LEAK1_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK1_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
	
	//LEAK 2
	UIO_LEAK2_CLK_Cmd(UIO_LEAK2_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEAK2_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEAK2_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LEAK2_EXTI_SOURCE,UIO_LEAK2_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LEAK2_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK2_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Init(&NVIC_InitStruct);
	
	//LEAK 3
	UIO_LEAK3_CLK_Cmd(UIO_LEAK3_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEAK3_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEAK3_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LEAK3_EXTI_SOURCE,UIO_LEAK3_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LEAK3_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK3_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStruct);
	
	//LEAK 4
	UIO_LEAK4_CLK_Cmd(UIO_LEAK4_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEAK4_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEAK4_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LEAK4_EXTI_SOURCE,UIO_LEAK4_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LEAK4_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK4_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_Init(&NVIC_InitStruct);
	
	//LEAK 5
	UIO_LEAK5_CLK_Cmd(UIO_LEAK5_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UIO_LEAK5_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UIO_LEAK5_PORT,&GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(UIO_LEAK5_EXTI_SOURCE,UIO_LEAK5_EXTI_PORT);
	
	EXTI_InitStruct.EXTI_Line = UIO_LEAK5_EXTI_Line;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = UIO_LEAK5_EXTI_Trigger;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = UIO_LEAK5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x04;
	NVIC_Init(&NVIC_InitStruct);
	
//	//Emergency Case	
//	UIO_EMER24V40AH_CLK_Cmd(UIO_EMER24V40AH_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_EMER24V40AH_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_EMER24V40AH_PORT,&GPIO_InitStruct);
//	
//	UIO_EMER24V10AH_CLK_Cmd(UIO_EMER24V10AH_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UIO_EMER24V10AH_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UIO_EMER24V10AH_PORT,&GPIO_InitStruct);
}

//uint8_t UIO_LeakSensorPosition(void)
//{
//	return _leak_position;
//}

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

void UIO_Emergency24V40Ah(FunctionalState NewState)
{
	(NewState == ENABLE)? GPIO_SetBits(UIO_EMER24V40AH_PORT, UIO_EMER24V40AH_PIN): GPIO_ResetBits(UIO_EMER24V40AH_PORT, UIO_EMER24V40AH_PIN);
}

void UIO_Emergency24V10Ah(FunctionalState NewState)
{
	(NewState == ENABLE)? GPIO_SetBits(UIO_EMER24V10AH_PORT, UIO_EMER24V10AH_PIN): GPIO_ResetBits(UIO_EMER24V10AH_PORT, UIO_EMER24V10AH_PIN);
}

uint8_t UIO_CheckPistolAtHead(void)
{
	return (GPIO_ReadInputDataBit(UIO_LS1_PORT, UIO_LS1_PIN) == Bit_SET) ? 0x01 : 0x00;
}

uint8_t UIO_CheckPistolAtTail(void)
{
	return (GPIO_ReadInputDataBit(UIO_LS2_PORT, UIO_LS2_PIN) == Bit_SET) ? 0x01 : 0x00;
}

uint8_t UIO_CheckMassAtHead(void)
{
	return (GPIO_ReadInputDataBit(UIO_LS3_PORT, UIO_LS3_PIN) == Bit_SET) ? 0x01 : 0x00;
}

uint8_t UIO_CheckMassAtTail(void)
{
	return (GPIO_ReadInputDataBit(UIO_LS4_PORT, UIO_LS4_PIN) == Bit_SET) ? 0x01 : 0x00;
}

void UIO_Light_Cmd(FunctionalState NewState)
{
		(NewState == ENABLE)? GPIO_SetBits(UIO_LIGHT_PORT, UIO_LIGHT_PIN): GPIO_ResetBits(UIO_LIGHT_PORT, UIO_LIGHT_PIN);
}
/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/

//PE2: Piston_T
void UIO_LS1_IRQHandler(void)
{
	if(EXTI_GetITStatus(UIO_LS1_EXTI_Line) != RESET)
	{
		if(Flag.Done_setup)
		{
			UPMCAN_Pistol_SetOLoop_Duty(0,MOTOR_CCW);
			for(uint8_t i = 0; i < 3; i++)
			{
				UDELAY_us(1);
				UPMCAN_Pistol_SetOLoop_Duty(30,MOTOR_CW);
			}
			Flag.Pis_T = true;
			UIO_LEDORANGE_TOGGLE();
		}
	}
	EXTI_ClearITPendingBit(UIO_LS1_EXTI_Line);
}

//PE3: Piston_H
void UIO_LS2_IRQHandler(void)
{
	if(EXTI_GetITStatus(UIO_LS2_EXTI_Line) != RESET)
	{
		if(Flag.Done_setup)
		{
			UPMCAN_Pistol_SetOLoop_Duty(0,MOTOR_CW);
			for(uint8_t i = 0; i < 3; i++)
			{
				UDELAY_us(1);
				UPMCAN_Pistol_SetOLoop_Duty(30,MOTOR_CCW);
			}
			Flag.Pis_H = true;
			UIO_LEDORANGE_TOGGLE();
		}
	}
	EXTI_ClearITPendingBit(UIO_LS2_EXTI_Line);
}

//PE4: Mass_T
void UIO_LS3_IRQHandler(void)
{
	if(EXTI_GetITStatus(UIO_LS3_EXTI_Line) != RESET)
	{
		if(Flag.Done_setup)
		{
			UPMCAN_Mass_SetOLoop_Duty(0,MOTOR_CCW);
			for(uint8_t i = 0; i < 3; i++)
			{
				UDELAY_us(1);
				UPMCAN_Mass_SetOLoop_Duty(30,MOTOR_CW);
			}
			Flag.Mass_T = true;
			UIO_LEDORANGE_TOGGLE();
		}
	}
	EXTI_ClearITPendingBit(UIO_LS3_EXTI_Line);
}

//PE5: Mass_H
void UIO_EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(UIO_LS4_EXTI_Line) != RESET)
	{
		if(Flag.Done_setup)
		{
			UPMCAN_Mass_SetOLoop_Duty(0,MOTOR_CW);
			for(uint8_t i = 0; i < 3; i++)
			{
				UDELAY_us(1);
				UPMCAN_Mass_SetOLoop_Duty(30,MOTOR_CCW);
			}
			Flag.Mass_H = true;
			UIO_LEDORANGE_TOGGLE();
		}
	}
	EXTI_ClearITPendingBit(UIO_LS4_EXTI_Line);
}

void UIO_EXTI15_10_IRQHandler(void)
{	
	if(EXTI_GetITStatus(UIO_LEAK1_EXTI_Line) != RESET)
	{
		LEAK_POSITION_1 = 1;
		EXTI_ClearITPendingBit(UIO_LEAK1_EXTI_Line);
	}
		
	if(EXTI_GetITStatus(UIO_LEAK2_EXTI_Line) != RESET)
	{
		LEAK_POSITION_2 = 1;
		EXTI_ClearITPendingBit(UIO_LEAK2_EXTI_Line);
	}
		
	if(EXTI_GetITStatus(UIO_LEAK3_EXTI_Line) != RESET)
	{
		LEAK_POSITION_3 = 1;
		EXTI_ClearITPendingBit(UIO_LEAK3_EXTI_Line);
	}	
	
	if(EXTI_GetITStatus(UIO_LEAK4_EXTI_Line) != RESET)
	{
		LEAK_POSITION_4 = 1;
		EXTI_ClearITPendingBit(UIO_LEAK4_EXTI_Line);
	}	
	
	if(EXTI_GetITStatus(UIO_LEAK5_EXTI_Line) != RESET)
	{
		LEAK_POSITION_5 = 1;
		EXTI_ClearITPendingBit(UIO_LEAK5_EXTI_Line);
	}	
}
