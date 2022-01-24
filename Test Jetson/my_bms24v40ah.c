/* Includes ------------------------------------------------------------------*/
#include "my_bms24v40ah.h"
#include <math.h>

#define UBMS40_BUFFER_LENGTH			36
/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static uint8_t UBMS40_DMABuffer[UBMS40_BUFFER_LENGTH] = {0};
static uint8_t UBMS40_DataBuffer[UBMS40_BUFFER_LENGTH] = {0};

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   BMS40 Pin define 
		*/
			#define 	UBMS40_TX_PIN  				   	GPIO_Pin_12			
			#define 	UBMS40_TX_PORT 				  	GPIOC
			#define 	UBMS40_TX_CLK							RCC_AHB1Periph_GPIOC			
			#define 	UBMS40_TX_SOURCE				 	GPIO_PinSource12				
			#define		UBMS40_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UBMS40_RX_PIN  				   	GPIO_Pin_2
			#define 	UBMS40_RX_PORT 				  	GPIOD
			#define 	UBMS40_RX_CLK							RCC_AHB1Periph_GPIOD
			#define 	UBMS40_RX_SOURCE				 	GPIO_PinSource2
			#define		UBMS40_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			
			#define		UBMS40_USART							UART5
			#define 	UBMS40_USART_CLK					RCC_APB1Periph_UART5
			#define		UBMS40_USART_CLK_Cmd		  RCC_APB1PeriphClockCmd
			#define 	UBMS40_USART_BAUDRATE			115200
			#define		UBMS40_USART_AF						GPIO_AF_UART5
			#define 	UBMS40_USART_IRQn					UART5_IRQn
			
			#define   UBMS40_DMA_Channel				DMA_Channel_4
			#define   UBMS40_DMA_Stream					DMA1_Stream0
			#define		UBMS40_DMA_CLK_Cmd		  	RCC_AHB1PeriphClockCmd
			#define 	UBMS40_DMA_CLK						RCC_AHB1Periph_DMA1
			#define		UBMS40_DMA_Stream_IRQn		DMA1_Stream0_IRQn
			
			#define		UBMS40_DMA_Stream_IRQHandler		DMA1_Stream0_IRQHandler
			#define		UBMS40_USART_IRQHandler					UART5_IRQHandler
			

/* Private variables ---------------------------------------------------------*/
static char my_bms40_start[] 	= "$AUV2000START\r\n";
static char my_bms40_stop[] 	= "$AUV2000STOP\r\n";	

UBMS40_Status_Typedef UBMS40_Status;
/* Private function prototypes -----------------------------------------------*/
static void UBMS40_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out)
{
	union
	{
		float _value;
		uint8_t _byte[4];
	}_part;
	
	_part._value = _data_in;
	
	_data_out[0] = _part._byte[0];
	_data_out[1] = _part._byte[1];
	_data_out[2] = _part._byte[2];
	_data_out[3] = _part._byte[3];
}

static float UBMS40_Convert_Bytes_to_Float(uint8_t* _data_in)
{
	union
	{
		float _value;
		uint8_t _byte[4];
	}_part;
	
	_part._byte[0] = _data_in[0];
	_part._byte[1] = _data_in[1];
	_part._byte[2] = _data_in[2];
	_part._byte[3] = _data_in[3];
	
	return _part._value; 
}

uint8_t UBMS40_Checksum(uint8_t *_data)
{
	uint8_t value = 0;

	//Calculate CheckSum (Byte)
	for (int i = 0; i < UBMS40_BUFFER_LENGTH - 4; i++)
	{
		value += _data[i];
	}
	value = ~value;
	value++;
	return (uint8_t)value;
}
/* Exported function body ----------------------------------------------------*/
void UBMS40_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	UBMS40_TX_CLK_Cmd(UBMS40_TX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UBMS40_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UBMS40_TX_PORT,&GPIO_InitStruct);
	
	UBMS40_RX_CLK_Cmd(UBMS40_RX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UBMS40_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UBMS40_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UBMS40_TX_PORT,UBMS40_TX_SOURCE,UBMS40_USART_AF);
	GPIO_PinAFConfig(UBMS40_RX_PORT,UBMS40_RX_SOURCE,UBMS40_USART_AF);
	
	UBMS40_USART_CLK_Cmd(UBMS40_USART_CLK,ENABLE);
	USART_InitStruct.USART_BaudRate = UBMS40_USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(UBMS40_USART,&USART_InitStruct);
	
	USART_DMACmd(UBMS40_USART,USART_DMAReq_Rx,ENABLE);
	
	UBMS40_DMA_CLK_Cmd(UBMS40_DMA_CLK,ENABLE);	
	DMA_InitStruct.DMA_Channel = UBMS40_DMA_Channel;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&(UBMS40_DMABuffer));
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(UBMS40_USART->DR));
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = UBMS40_BUFFER_LENGTH;																	
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

	DMA_Init(UBMS40_DMA_Stream, &DMA_InitStruct);

	DMA_Cmd(UBMS40_DMA_Stream, DISABLE);	

	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(UBMS40_DMA_Stream, DMA_IT_TC, DISABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UBMS40_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	USART_Cmd(UBMS40_USART,ENABLE);

}

// p - location of the data in the string.
// s - size of the data (example year is 1 cell voltage is 2 and PV1 current is 3)
// d - the name of the variable in this case sbms.
float dcmp(int p, int s, uint8_t* d)
{
		double xx;
		int i;
		xx = 0;
		for (int z = 0; z < s; z++)
		{
				i = p + s - 1 -z;
				xx = xx + ((d[i] - 35) * pow(91, z));
		}
		return xx;
}

void UBMS40_ArrangeData(uint8_t *_data_in, uint8_t *_data_out)
{
	uint8_t _start = 0;
	for(int i = 0; i < UBMS40_BUFFER_LENGTH; i++)
	{
		if((_data_in[i] == '$')&&(_data_in[i+1] == 'A')&&(_data_in[i+2] == 'U')&&(_data_in[i+3] == 'V')
			&&(_data_in[i+4] == '2')&&(_data_in[i+5] == '0')&&(_data_in[i+6] == '0')&&(_data_in[i+7] == '0')) 
		{
			_start = i; 
			break;
		}
	}
	for(int i = 0; i < UBMS40_BUFFER_LENGTH; i++)
	{
		if((_start + i) < UBMS40_BUFFER_LENGTH)
		{
			_data_out[i] = _data_in[_start + i];
		}
		else 
		{
			_data_out[i] = _data_in[i - (UBMS40_BUFFER_LENGTH - _start)];
		}
	}	
}

void UBMS40_Transmit(char *szData)
{
	char c = szData[0];
	while(c)
	{
		while(USART_GetFlagStatus(UBMS40_USART,USART_FLAG_TXE) != SET);
		USART_SendData(UBMS40_USART,c);
		c = *(++szData);
	}
}

void UBMS40_Cmd(FunctionalState NewState)
{
	if(NewState == ENABLE)
	{
		UBMS40_Transmit(my_bms40_start);
		/* DMA Transfer complete interrupt configure */
		DMA_ITConfig(UBMS40_DMA_Stream, DMA_IT_TC, ENABLE);
		DMA_Cmd(UBMS40_DMA_Stream, ENABLE);	
	}
	else
	{
		UBMS40_Transmit(my_bms40_stop);
		DMA_Cmd(UBMS40_DMA_Stream, DISABLE);	

		/* DMA Transfer complete interrupt configure */
		DMA_ITConfig(UBMS40_DMA_Stream, DMA_IT_TC, DISABLE);
	}
}

void UBMS40_GetStatus(UBMS40_Status_Typedef *_ubms40_status)
{
//	UBMS40_Status.Hours = _ubms40_status->Hours;
//	UBMS40_Status.Minutes = _ubms40_status->Minutes;
//	UBMS40_Status.Seconds = _ubms40_status->Seconds;
	_ubms40_status->BatteryTotal = UBMS40_Status.BatteryTotal ;
	_ubms40_status->BatteryCapacity = UBMS40_Status.BatteryCapacity;
	_ubms40_status->BatteryUsed = UBMS40_Status.BatteryUsed;
	_ubms40_status->BatteryPercentage = UBMS40_Status.BatteryPercentage;
	_ubms40_status->BatteryCurrent = UBMS40_Status.BatteryCurrent;
	_ubms40_status->BaterryVoltage = UBMS40_Status.BaterryVoltage;
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void UBMS40_DMA_Stream_IRQHandler(void)
{	
	if(DMA_GetITStatus(UBMS40_DMA_Stream, DMA_IT_TCIF2) != RESET)
	{	
		UBMS40_ArrangeData(UBMS40_DMABuffer, UBMS40_DataBuffer);
//		for(int i = 0; i < UBMS40_BUFFER_LENGTH; i++)
//		{
//			UBMS40_DataBuffer[i] = UBMS40_DMABuffer[i];
//		}
		if((UBMS40_DataBuffer[0] == '$') && (UBMS40_DataBuffer[1] == 'A') && (UBMS40_DataBuffer[2] == 'U') && (UBMS40_DataBuffer[3] == 'V') 
		&& (UBMS40_DataBuffer[4] == '2') && (UBMS40_DataBuffer[5] == '0') && (UBMS40_DataBuffer[6] == '0') && (UBMS40_DataBuffer[7] == '0') 
		&& (UBMS40_DataBuffer[32] == UBMS40_Checksum(UBMS40_DataBuffer)))
		{
//			UBMS40_Status.Hours = UBMS40_DataBuffer[8];
//			UBMS40_Status.Minutes = UBMS40_DataBuffer[9];
//			UBMS40_Status.Seconds = UBMS40_DataBuffer[10];
//			
//			UBMS40_Status.BatteryTotal = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 11);
//			UBMS40_Status.BatteryCapacity = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 15);
//			UBMS40_Status.BatteryUsed = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 19);
//			UBMS40_Status.BatteryPercentage = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 23);
//			UBMS40_Status.BatteryCurrent = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 27);
//			UBMS40_Status.BaterryVoltage = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 31);

			UBMS40_Status.BatteryTotal = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 8);
			UBMS40_Status.BatteryCapacity = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 12);
			UBMS40_Status.BatteryUsed = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 16);
			UBMS40_Status.BatteryPercentage = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 20);
			UBMS40_Status.BatteryCurrent = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 24);
			UBMS40_Status.BaterryVoltage = UBMS40_Convert_Bytes_to_Float(UBMS40_DataBuffer + 28);
		}
		DMA_ClearITPendingBit(UBMS40_DMA_Stream,DMA_IT_DMEIF2|DMA_IT_FEIF2|DMA_IT_HTIF2|DMA_IT_TCIF2|DMA_IT_TEIF2);
	}
}	

