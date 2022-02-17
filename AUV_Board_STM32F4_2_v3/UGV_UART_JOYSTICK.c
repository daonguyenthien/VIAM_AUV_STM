#include "stm32f4xx.h"
#include <string.h>
#include "UGV_UART_JOYSTICK.h"
#include "UGV_SBUS.h"
//#include "UGV_PWM.h"


/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   JOYSTICK Pin define 
		*/
		
		/* Definition for USARTx resources ******************************************/
		#define		USARTx_JOYSTICK_TX_CLK_INIT		  								 	RCC_AHB1PeriphClockCmd
		#define 	USARTx_JOYSTICK_TX_GPIO_CLK												RCC_AHB1Periph_GPIOC
		#define 	USARTx_JOYSTICK_TX_GPIO_PORT 					 						GPIOC
		#define		USARTx_JOYSTICK_TX_PIN														GPIO_Pin_6
		#define 	USARTx_JOYSTICK_TX_SOURCE											 		GPIO_PinSource6
		
		#define		USARTx_JOYSTICK_RX_CLK_INIT		  	 								RCC_AHB1PeriphClockCmd
		#define 	USARTx_JOYSTICK_RX_GPIO_CLK												RCC_AHB1Periph_GPIOC
		#define 	USARTx_JOYSTICK_RX_GPIO_PORT 				  						GPIOC
		#define		USARTx_JOYSTICK_RX_PIN														GPIO_Pin_7
		#define 	USARTx_JOYSTICK_RX_SOURCE					 								GPIO_PinSource7
		
		#define		USARTx_JOYSTICK																		USART6
		#define 	USARTx_JOYSTICK_CLK																RCC_APB2Periph_USART6
		#define		USARTx_JOYSTICK_CLK_INIT		 											RCC_APB2PeriphClockCmd
		#define 	USARTx_JOYSTICK_BAUDRATE													100000
		#define		USARTx_JOYSTICK_AF																GPIO_AF_USART6
		#define 	USARTx_JOYSTICK_IRQn															USART6_IRQn
		#define 	USARTx_JOYSTICK_IRQPreemptionPriority							0x00
		#define 	USARTx_JOYSTICK_IRQSubPriority										0x02
		
		#define		USARTx_JOYSTICK_DMA																DMA2
		#define 	USARTx_JOYSTICK_DMAx_CLK                  				RCC_AHB1Periph_DMA2
		#define   USARTx_JOYSTICK_DMA_Channel												DMA_Channel_5
		#define   USARTx_JOYSTICK_DMA_Stream												DMA2_Stream1
		#define		USARTx_JOYSTICK_DMA_CLK_Cmd		  									RCC_AHB1PeriphClockCmd
		#define		USARTx_JOYSTICK_DMA_Stream_IRQn										DMA2_Stream1_IRQn
		#define 	USARTx_JOYSTICK_DMA_IRQPreemptionPriority					0x00
		#define 	USARTx_JOYSTICK_DMA_IRQSubPriority								0x01
		
		#define		USARTx_JOYSTICK_DMA_Stream_IRQHandler							DMA2_Stream1_IRQHandler
		#define		USARTx_JOYSTICK_IRQHandler												USART6_IRQHandler
		
		#define 	USARTx_JOYSTICK_BUFFER_LENGTH 										25

/**
 * @}
 */
mybuffer uart_buffer_u3;
bool failSafe;
bool lostFrame;
uint16_t channels[16];
int16_t scalechannels[16];
int16_t old_scalechannels[16];
int mode_manual;
int end_mode;
double speed1=0,speed2=0;

void USART6_DMA_Rx_Config(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef   DMA_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStruct;
	
	/* Enable GPIO clock */
	USARTx_JOYSTICK_TX_CLK_INIT(USARTx_JOYSTICK_TX_GPIO_CLK, ENABLE);
	/* Enable UART clock */
	USARTx_JOYSTICK_CLK_INIT(USARTx_JOYSTICK_CLK, ENABLE);
	/* Enable DMA clock */
	USARTx_JOYSTICK_DMA_CLK_Cmd(USARTx_JOYSTICK_DMAx_CLK, ENABLE);
	
	/* GPIO Configuration for UART Rx */
	USARTx_JOYSTICK_RX_CLK_INIT(USARTx_JOYSTICK_RX_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin   = USARTx_JOYSTICK_RX_PIN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
  GPIO_Init(USARTx_JOYSTICK_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* GPIO Configuration for UART Tx */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin   = USARTx_JOYSTICK_TX_PIN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
  GPIO_Init(USARTx_JOYSTICK_TX_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(USARTx_JOYSTICK_TX_GPIO_PORT, USARTx_JOYSTICK_TX_SOURCE, USARTx_JOYSTICK_AF);
  GPIO_PinAFConfig(USARTx_JOYSTICK_RX_GPIO_PORT, USARTx_JOYSTICK_RX_SOURCE, USARTx_JOYSTICK_AF); 

  /* USARTx configured as follow:
		- BaudRate = 100000 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = USARTx_JOYSTICK_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx_JOYSTICK, &USART_InitStructure);
  
	/* Enable USART */
  USART_Cmd(USARTx_JOYSTICK, ENABLE);
	
	/* Enable UART DMA */
	USART_DMACmd(USARTx_JOYSTICK, USART_DMAReq_Rx, ENABLE);
	/* DMA1 Stream2 Channel4 for USART3 Rx configuration */	
  DMA_InitStructure.DMA_Channel = USARTx_JOYSTICK_DMA_Channel;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uart_buffer_u3.mbuff_rx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = USARTx_JOYSTICK_BUFFER_LENGTH;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;//
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  
	DMA_Init(USARTx_JOYSTICK_DMA_Stream, &DMA_InitStructure);
  DMA_Cmd(USARTx_JOYSTICK_DMA_Stream, ENABLE);

//	NVIC_InitStruct.NVIC_IRQChannel = USARTx_JOYSTICK_DMA_Stream_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = USARTx_JOYSTICK_DMA_IRQPreemptionPriority;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = USARTx_JOYSTICK_DMA_IRQSubPriority;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	DMA_ITConfig(USARTx_JOYSTICK_DMA_Stream, DMA_IT_TC, ENABLE);
}

void USARTx_JOYSTICK_DMA_Stream_IRQHandler(void)
{
	if(DMA_GetITStatus(USARTx_JOYSTICK_DMA_Stream, DMA_IT_TCIF1) != RESET)
	{
		SBUS_read(&uart_buffer_u3,&channels[0],&scalechannels[0],&failSafe,&lostFrame);
	}
	DMA_ClearITPendingBit(USARTx_JOYSTICK_DMA_Stream,DMA_IT_DMEIF1|DMA_IT_FEIF1|DMA_IT_HTIF1|DMA_IT_TCIF1|DMA_IT_TEIF1);
}

