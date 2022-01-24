/* Includes ------------------------------------------------------------------*/
#include "my_dvl.h"

#define UDVL_BUFFER_LENGTH			38

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static uint8_t UDVL_DMABuffer[UDVL_BUFFER_LENGTH], UDVL_DataBuffer[UDVL_BUFFER_LENGTH];

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   DVL Pin define 
		*/
			#define 	UDVL_TX_PIN  				   	GPIO_Pin_9			
			#define 	UDVL_TX_PORT 				  	GPIOA
			#define 	UDVL_TX_CLK							RCC_AHB1Periph_GPIOA			
			#define 	UDVL_TX_SOURCE				 		GPIO_PinSource9				
			#define		UDVL_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UDVL_RX_PIN  				   	GPIO_Pin_10
			#define 	UDVL_RX_PORT 				  	GPIOA
			#define 	UDVL_RX_CLK							RCC_AHB1Periph_GPIOA
			#define 	UDVL_RX_SOURCE				 		GPIO_PinSource10
			#define		UDVL_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			
			#define		UDVL_USART								USART1
			#define 	UDVL_USART_CLK						RCC_APB2Periph_USART1
			#define		UDVL_USART_CLK_Cmd		  	RCC_APB2PeriphClockCmd
			#define 	UDVL_USART_BAUDRATE			115200
			#define		UDVL_USART_AF						GPIO_AF_USART1
			#define 	UDVL_USART_IRQn					USART1_IRQn
			
			#define   UDVL_DMA_Channel					DMA_Channel_4
			#define   UDVL_DMA_Stream					DMA2_Stream5
			#define		UDVL_DMA_CLK_Cmd		  		RCC_AHB1PeriphClockCmd
			#define 	UDVL_DMA_CLK							RCC_AHB1Periph_DMA2
			#define		UDVL_DMA_Stream_IRQn			DMA2_Stream5_IRQn
			
			#define		UDVL_DMA_Stream_IRQHandler			DMA2_Stream5_IRQHandler
			#define		UDVL_USART_IRQHandler					USART1_IRQHandler
			

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/
void UDVL_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	UDVL_TX_CLK_Cmd(UDVL_TX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UDVL_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UDVL_TX_PORT,&GPIO_InitStruct);
	
	UDVL_RX_CLK_Cmd(UDVL_RX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UDVL_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UDVL_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UDVL_TX_PORT,UDVL_TX_SOURCE,UDVL_USART_AF);
	GPIO_PinAFConfig(UDVL_RX_PORT,UDVL_RX_SOURCE,UDVL_USART_AF);
	
	UDVL_USART_CLK_Cmd(UDVL_USART_CLK,ENABLE);
	USART_InitStruct.USART_BaudRate = UDVL_USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(UDVL_USART,&USART_InitStruct);
	
	USART_DMACmd(UDVL_USART,USART_DMAReq_Rx,ENABLE);
	
	UDVL_DMA_CLK_Cmd(UDVL_DMA_CLK,ENABLE);	
	DMA_InitStruct.DMA_Channel = UDVL_DMA_Channel;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&(UDVL_DMABuffer));
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(UDVL_USART->DR));
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = UDVL_BUFFER_LENGTH;																	
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(UDVL_DMA_Stream, &DMA_InitStruct);

	DMA_Cmd(UDVL_DMA_Stream, ENABLE);	

	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(UDVL_DMA_Stream, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UDVL_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	USART_Cmd(UDVL_USART,ENABLE);

}

uint8_t UDVL_Checksum(uint8_t *_data)
{
	uint8_t _start = 0, _end = 0;
	for(int i = 0; i < UDVL_BUFFER_LENGTH; i++)
	{
		if(_data[i] == '$') _start = i;
		if(_data[i] == '*') _end = i;
	}
	
	uint8_t _checksum = _data[_start + 1];
	for(int index = _start + 2; index < _end; index++)
	{
		_checksum ^= _data[index];
	}
	return _checksum;
}

void UDVL_ArrangeData(uint8_t *_data_in, uint8_t *_data_out)
{
	uint8_t _start = 0;
	for(int i = 0; i < UDVL_BUFFER_LENGTH; i++)
	{
		if(_data_in[i] == '$') _start = i; 
	}
	for(int i = 0; i < UDVL_BUFFER_LENGTH; i++)
	{
		if((_start + i) < UDVL_BUFFER_LENGTH)
		{
			_data_out[i] = _data_in[_start + i];
		}
		else 
		{
			_data_out[i] = _data_in[i - (UDVL_BUFFER_LENGTH - _start)];
		}
	}	
}

void UDVL_Transmit(char *szData)
{
	char c = *szData;
	while(c)
	{
		while(USART_GetFlagStatus(UDVL_USART,USART_FLAG_TXE) != SET);
		USART_SendData(UDVL_USART,c);
		c = *(++szData);
	}
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void UDVL_DMA_Stream_IRQHandler(void)
{	
	if(DMA_GetITStatus(UDVL_DMA_Stream, DMA_IT_TCIF5) != RESET)
	{	
		UDVL_ArrangeData(UDVL_DMABuffer,UDVL_DataBuffer);
		
		DMA_ClearITPendingBit(UDVL_DMA_Stream,DMA_IT_DMEIF5|DMA_IT_FEIF5|DMA_IT_HTIF5|DMA_IT_TCIF5|DMA_IT_TEIF5);
	}
}

