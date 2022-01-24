/* Includes ------------------------------------------------------------------*/
#include "my_altimeter.h"
#include "my_io.h"

#define UALTI_BUFFER_LENGTH			38
#define UALTI_IN_FEET						0
#define UALTI_IN_METRES					1
#define UALTI_IN_FATHOMS				2
/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static uint8_t UALTI_DMABuffer[UALTI_BUFFER_LENGTH], UALTI_DataBuffer[UALTI_BUFFER_LENGTH];

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   XSENS Pin define 
		*/
			#define 	UALTI_TX_PIN  				   	GPIO_Pin_9			
			#define 	UALTI_TX_PORT 				  	GPIOA
			#define 	UALTI_TX_CLK							RCC_AHB1Periph_GPIOA			
			#define 	UALTI_TX_SOURCE				 		GPIO_PinSource9				
			#define		UALTI_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UALTI_RX_PIN  				   	GPIO_Pin_10
			#define 	UALTI_RX_PORT 				  	GPIOA
			#define 	UALTI_RX_CLK							RCC_AHB1Periph_GPIOA
			#define 	UALTI_RX_SOURCE				 		GPIO_PinSource10
			#define		UALTI_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			
			#define		UALTI_USART								USART1
			#define 	UALTI_USART_CLK						RCC_APB2Periph_USART1
			#define		UALTI_USART_CLK_Cmd		  	RCC_APB2PeriphClockCmd
			#define 	UALTI_USART_BAUDRATE			115200
			#define		UALTI_USART_AF						GPIO_AF_USART1
			#define 	UALTI_USART_IRQn					USART1_IRQn
			
			#define   UALTI_DMA_Channel					DMA_Channel_4
			#define   UALTI_DMA_Stream					DMA2_Stream5
			#define		UALTI_DMA_CLK_Cmd		  		RCC_AHB1PeriphClockCmd
			#define 	UALTI_DMA_CLK							RCC_AHB1Periph_DMA2
			#define		UALTI_DMA_Stream_IRQn			DMA2_Stream5_IRQn
			
			#define		UALTI_DMA_Stream_IRQHandler			DMA2_Stream5_IRQHandler
			#define		UALTI_USART_IRQHandler					USART1_IRQHandler
			

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/
void UALTI_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	UALTI_TX_CLK_Cmd(UALTI_TX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UALTI_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UALTI_TX_PORT,&GPIO_InitStruct);
	
	UALTI_RX_CLK_Cmd(UALTI_RX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UALTI_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UALTI_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UALTI_TX_PORT,UALTI_TX_SOURCE,UALTI_USART_AF);
	GPIO_PinAFConfig(UALTI_RX_PORT,UALTI_RX_SOURCE,UALTI_USART_AF);
	
	UALTI_USART_CLK_Cmd(UALTI_USART_CLK,ENABLE);
	USART_InitStruct.USART_BaudRate = UALTI_USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(UALTI_USART,&USART_InitStruct);
	
	USART_DMACmd(UALTI_USART,USART_DMAReq_Rx,ENABLE);
	
	UALTI_DMA_CLK_Cmd(UALTI_DMA_CLK,ENABLE);	
	DMA_InitStruct.DMA_Channel = UALTI_DMA_Channel;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&(UALTI_DMABuffer));
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(UALTI_USART->DR));
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = UALTI_BUFFER_LENGTH;																	
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

	DMA_Init(UALTI_DMA_Stream, &DMA_InitStruct);

	DMA_Cmd(UALTI_DMA_Stream, ENABLE);	

	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(UALTI_DMA_Stream, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UALTI_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	USART_Cmd(UALTI_USART,ENABLE);

}

union Altimeter_Sensor UALTI_in_feet, UALTI_in_metres, UALTI_in_fathoms;

uint32_t UALTI_GetAddressMemory(void)
{
	return (uint32_t)(&(UALTI_DMABuffer));
}

uint8_t UALTI_Checksum(uint8_t *_data)
{
	uint8_t _start = 0, _end = 0;
	for(int i = 0; i < UALTI_BUFFER_LENGTH; i++)
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

uint8_t UALTI_Convert2Int(uint8_t _data)
{
	uint8_t _int = 0;
		switch(_data)
		{
			case 'A':
			{
				_int = 10;
				break;
			}
			case 'B':
			{
				_int = 11;
				break;
			}
			case 'C':
			{
				_int = 12;
				break;
			}
			case 'D':
			{
				_int = 13;
				break;
			}
			case 'E':
			{
				_int = 14;
				break;
			}
			case 'F':
			{
				_int = 15;
				break;
			}
			default:
			{
				_int = _data - 0x30;
				break;
			}
		}
		return _int;
}

void UALTI_ArrangeData(uint8_t *_data_in, uint8_t *_data_out)
{
	uint8_t _start = 0;
	for(int i = 0; i < UALTI_BUFFER_LENGTH; i++)
	{
		if(_data_in[i] == '$') _start = i; 
	}
	for(int i = 0; i < UALTI_BUFFER_LENGTH; i++)
	{
		if((_start + i) < UALTI_BUFFER_LENGTH)
		{
			_data_out[i] = _data_in[_start + i];
		}
		else 
		{
			_data_out[i] = _data_in[i - (UALTI_BUFFER_LENGTH - _start)];
		}
	}	
}

void UALTI_ConvertData(uint8_t *_data, float* _in_feet, float* _in_metres, float* _in_fathoms)
{
	uint8_t _data_in_feet[5], _data_in_metres[5], _data_in_fathoms[5];
	_data_in_feet[0] = _data[7] - 0x30;
	_data_in_feet[1] = _data[8] - 0x30;
	_data_in_feet[2] = _data[9] - 0x30;
	_data_in_feet[3] = _data[11] - 0x30;
	_data_in_feet[4] = _data[12] - 0x30;
	*_in_feet =(float)(_data_in_feet[0]*100.0 + _data_in_feet[1]*10.0 + _data_in_feet[2] + _data_in_feet[3]/10.0 + _data_in_feet[4]/100.0);
	
	_data_in_metres[0] = _data[16] - 0x30;
	_data_in_metres[1] = _data[17] - 0x30;
	_data_in_metres[2] = _data[18] - 0x30;
	_data_in_metres[3] = _data[20] - 0x30;
	_data_in_metres[4] = _data[21] - 0x30;
	*_in_metres =(float)(_data_in_metres[0]*100.0 + _data_in_metres[1]*10.0 + _data_in_metres[2] + _data_in_metres[3]/10.0 + _data_in_metres[4]/100.0);
	
	_data_in_fathoms[0] = _data[25] - 0x30;
	_data_in_fathoms[1] = _data[26] - 0x30;
	_data_in_fathoms[2] = _data[27] - 0x30;
	_data_in_fathoms[3] = _data[29] - 0x30;
	_data_in_fathoms[4] = _data[30] - 0x30;
	*_in_fathoms =(float)(_data_in_fathoms[0]*100.0 + _data_in_fathoms[1]*10.0 + _data_in_fathoms[2] + _data_in_fathoms[3]/10.0 + _data_in_fathoms[4]/100.0);	
}

uint8_t UALTI_GetByteData(uint8_t _type, uint8_t _num_byte)
{
	uint8_t _data = 0;
	switch(_type)
	{
		case UALTI_IN_FEET:
		{
			if(_num_byte == 0) _data = UALTI_in_feet.byte.a1;
			if(_num_byte == 1) _data = UALTI_in_feet.byte.a2;
			if(_num_byte == 2) _data = UALTI_in_feet.byte.a3;
			if(_num_byte == 3) _data = UALTI_in_feet.byte.a4;
			break;
		}
		case UALTI_IN_METRES:
		{
			if(_num_byte == 0) _data = UALTI_in_metres.byte.a1;
			if(_num_byte == 1) _data = UALTI_in_metres.byte.a2;
			if(_num_byte == 2) _data = UALTI_in_metres.byte.a3;
			if(_num_byte == 3) _data = UALTI_in_metres.byte.a4;
			break;
		}
		case UALTI_IN_FATHOMS:
		{
			if(_num_byte == 0) _data = UALTI_in_fathoms.byte.a1;
			if(_num_byte == 1) _data = UALTI_in_fathoms.byte.a2;
			if(_num_byte == 2) _data = UALTI_in_fathoms.byte.a3;
			if(_num_byte == 3) _data = UALTI_in_fathoms.byte.a4;
			break;
		}
	}
	return _data;
}

float UALTI_GetData(uint8_t _type)
{
	float _value = 0.0;
	switch(_type)
	{
		case UALTI_IN_FEET:
		{
			_value = UALTI_in_feet.Value;
			break;
		}
		case UALTI_IN_METRES:
		{
			_value = UALTI_in_metres.Value;
			break;
		}
		case UALTI_IN_FATHOMS:
		{
			_value = UALTI_in_fathoms.Value;
			break;
		}
	}
	return _value;
}

void UALTI_Transmit(char *szData)
{
	char c = *szData;
	while(c)
	{
		while(USART_GetFlagStatus(UALTI_USART,USART_FLAG_TXE) != SET);
		USART_SendData(UALTI_USART,c);
		c = *(++szData);
	}
}

void UALTI_GetStatus(Altimeter_StatusTypeDef *_status_data)
{
	_status_data->ALTI_in_fathoms = UALTI_in_fathoms.Value;
	_status_data->ALTI_in_metres = UALTI_in_metres.Value;
	_status_data->ALTI_in_feet = UALTI_in_feet.Value;
}
/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/

void UALTI_DMA_Stream_IRQHandler(void)
{
	if(DMA_GetITStatus(UALTI_DMA_Stream, DMA_IT_TCIF5) != RESET)
	{	
		UALTI_ArrangeData(UALTI_DMABuffer,UALTI_DataBuffer);
		if(UALTI_Checksum(UALTI_DataBuffer) == (UALTI_Convert2Int(UALTI_DataBuffer[34])*16 + UALTI_Convert2Int(UALTI_DataBuffer[35])))
		{
			UALTI_ConvertData(UALTI_DataBuffer,&UALTI_in_feet.Value,&UALTI_in_metres.Value,&UALTI_in_fathoms.Value);
		}
		DMA_ClearITPendingBit(UALTI_DMA_Stream,DMA_IT_DMEIF5|DMA_IT_FEIF5|DMA_IT_HTIF5|DMA_IT_TCIF5|DMA_IT_TEIF5);
	}
}

