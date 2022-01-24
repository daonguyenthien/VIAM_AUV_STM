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
#include "my_leddriver.h"
#include "my_delay.h"
/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   MCP41010 Pin define 
		*/
		//CS
		#define ULED_NSS_PIN_CLK_INIT				 	 RCC_AHB1PeriphClockCmd
		#define ULED_NSS_PIN                   GPIO_Pin_5
		#define ULED_NSS_GPIO_PORT             GPIOA
		#define ULED_NSS_GPIO_CLK            	 RCC_AHB1Periph_GPIOA

		#define ULED_SCK_PIN_CLK_INIT				 	 RCC_AHB1PeriphClockCmd
		#define ULED_SCK_PIN                   GPIO_Pin_4
		#define ULED_SCK_GPIO_PORT             GPIOA
		#define ULED_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
		
		#define ULED_MOSI_PIN_CLK_INIT				 RCC_AHB1PeriphClockCmd
		#define ULED_MOSI_PIN                  GPIO_Pin_7
		#define ULED_MOSI_GPIO_PORT            GPIOA
		#define ULED_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
//  #define ULED_MOSI_SOURCE               GPIO_PinSource7
//  #define ULED_MOSI_AF                   GPIO_AF_SPI1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint16_t ULEDDRIVER_MakeFrame(COMMAND _cmd, uint8_t _regData);
uint16_t sendData;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure SPI for MCP41010
  * @param  None
  * @retval None
  */
void ULEDDRIVER_Config(void){
	//SPI_Config();
	GPIO_InitTypeDef LC_GPIO_InitStruct;
	ULED_NSS_PIN_CLK_INIT(ULED_NSS_GPIO_CLK, ENABLE);
	//Clock GPIO
	ULED_SCK_PIN_CLK_INIT(ULED_SCK_GPIO_CLK, ENABLE);
	//MISO pin GPIO
	
	//GPIO configuration
	GPIO_DeInit(ULED_SCK_GPIO_PORT);

	
	LC_GPIO_InitStruct.GPIO_Mode		= GPIO_Mode_OUT;
	LC_GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
	LC_GPIO_InitStruct.GPIO_PuPd		= GPIO_PuPd_DOWN;
	LC_GPIO_InitStruct.GPIO_Speed		= GPIO_Speed_50MHz;
	LC_GPIO_InitStruct.GPIO_Pin			= ULED_SCK_PIN;
	
	
	GPIO_Init(ULED_SCK_GPIO_PORT, &LC_GPIO_InitStruct);
	
	LC_GPIO_InitStruct.GPIO_Mode		= GPIO_Mode_OUT;
	LC_GPIO_InitStruct.GPIO_PuPd		= GPIO_PuPd_NOPULL;
	LC_GPIO_InitStruct.GPIO_Pin			= ULED_MOSI_PIN;
	GPIO_Init(ULED_MOSI_GPIO_PORT, &LC_GPIO_InitStruct);
	
	LC_GPIO_InitStruct.GPIO_PuPd		= GPIO_PuPd_UP;
	LC_GPIO_InitStruct.GPIO_Mode		= GPIO_Mode_OUT;
	LC_GPIO_InitStruct.GPIO_Pin			= ULED_NSS_PIN;
	
	GPIO_Init(ULED_NSS_GPIO_PORT, &LC_GPIO_InitStruct);
	
}

void ULEDDRIVER_Write(uint8_t _data){
	sendData = ULEDDRIVER_MakeFrame(WRITEDATA, _data);
	GPIO_ResetBits(ULED_NSS_GPIO_PORT, ULED_NSS_PIN);
	for (int i = 15; i>=0; i--){
		if (sendData & (1<<i)) 
		{
			GPIO_SetBits(ULED_MOSI_GPIO_PORT, ULED_MOSI_PIN);
			
		}
		else {
			GPIO_ResetBits(ULED_MOSI_GPIO_PORT, ULED_MOSI_PIN);
		}
		GPIO_SetBits(ULED_SCK_GPIO_PORT, ULED_SCK_PIN);
		UDELAY_us(1);
		GPIO_ResetBits(ULED_SCK_GPIO_PORT, ULED_SCK_PIN);
		UDELAY_us(1);
	}
	GPIO_SetBits(ULED_NSS_GPIO_PORT, ULED_NSS_PIN);
}

static uint16_t ULEDDRIVER_MakeFrame(COMMAND _cmd, uint8_t _regData){
	uint16_t temp =	(_cmd<<8) + _regData;    
	return temp;
}

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
