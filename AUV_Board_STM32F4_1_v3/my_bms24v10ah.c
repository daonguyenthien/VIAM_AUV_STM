/* Includes ------------------------------------------------------------------*/
#include "my_bms24v10ah.h"

#define UBMS10_MAX_BUFFER_LENGTH			38
#define UBMS10_MAX_TRANSMITDATA_LENGTH			38
/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static uint8_t UBMS10_DMABuffer[UBMS10_MAX_BUFFER_LENGTH];
static uint8_t UBMS10_TransmitDataBuffer[UBMS10_MAX_TRANSMITDATA_LENGTH] = {'$', 'A', 'U', 'V', '2', '0', '0', '0',
																																				0, 0, 0,	// hour, minute, second
																																				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // BatteryTotal, BatteryCapacity, BatteryUsed, 
																																				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // BatteryPercentage, BatteryCurrent, BaterryVoltage
																																				0, '\r', '\n'};
UBMS10_Status_Typedef UBMS10_Status;

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   BMS24V40Ah Pin define 
		*/
			#define 	UBMS10_TX_PIN  				   	GPIO_Pin_12			
			#define 	UBMS10_TX_PORT 				  	GPIOC
			#define 	UBMS10_TX_CLK							RCC_AHB1Periph_GPIOC			
			#define 	UBMS10_TX_SOURCE				 	GPIO_PinSource12				
			#define		UBMS10_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UBMS10_RX_PIN  				   	GPIO_Pin_2
			#define 	UBMS10_RX_PORT 				  	GPIOD
			#define 	UBMS10_RX_CLK							RCC_AHB1Periph_GPIOD
			#define 	UBMS10_RX_SOURCE				 	GPIO_PinSource2
			#define		UBMS10_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			
			#define		UBMS10_USART							UART5
			#define 	UBMS10_USART_CLK					RCC_APB1Periph_UART5
			#define		UBMS10_USART_CLK_Cmd		  RCC_APB1PeriphClockCmd
			#define 	UBMS10_USART_BAUDRATE			115200
			#define		UBMS10_USART_AF						GPIO_AF_UART5
			#define 	UBMS10_USART_IRQn					UART5_IRQn
			
			#define   UBMS10_DMA_Channel				DMA_Channel_4
			#define   UBMS10_DMA_Stream					DMA1_Stream0
			#define		UBMS10_DMA_CLK_Cmd		  	RCC_AHB1PeriphClockCmd
			#define 	UBMS10_DMA_CLK						RCC_AHB1Periph_DMA1
			#define		UBMS10_DMA_Stream_IRQn		DMA1_Stream0_IRQn
			
			#define		UBMS10_DMA_PreemptionPriority		0x00
			#define		UBMS10_DMA_SubPriority					0x03
			
			#define		UBMS10_DMA_Stream_IRQHandler		DMA1_Stream0_IRQHandler
			#define		UBMS10_USART_IRQHandler					UART5_IRQHandler
			

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void UBMS10_Transmit(uint8_t *szData)
{
	char c = *szData;
	while(c)
	{
		while(USART_GetFlagStatus(UBMS10_USART,USART_FLAG_TXE) != SET);
		USART_SendData(UBMS10_USART,c);
		c = *(++szData);
	}
}

static void UBMS10_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out)
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

static float UBMS10_Convert_Bytes_to_Float(uint8_t* _data_in)
{
	union
	{
		float _value;
		uint8_t _byte[4];
	}_part;
	
	_data_in[0] = _part._byte[0];
	_data_in[1] = _part._byte[1];
	_data_in[2] = _part._byte[2];
	_data_in[3] = _part._byte[3];
	
	return _part._value; 
}

uint8_t UBMS10_Checksum(uint8_t *_data)
{
	uint8_t value = 0;

	//Calculate CheckSum (Byte)
	for (int i = 0; i < UBMS10_MAX_TRANSMITDATA_LENGTH; i++)
	{
		value += _data[i];
	}
	value = ~value;
	value++;
	return (uint8_t)value;
}

/* Exported function body ----------------------------------------------------*/
void UBMS10_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	UBMS10_TX_CLK_Cmd(UBMS10_TX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UBMS10_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UBMS10_TX_PORT,&GPIO_InitStruct);
	
	UBMS10_RX_CLK_Cmd(UBMS10_RX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UBMS10_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UBMS10_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UBMS10_TX_PORT,UBMS10_TX_SOURCE,UBMS10_USART_AF);
	GPIO_PinAFConfig(UBMS10_RX_PORT,UBMS10_RX_SOURCE,UBMS10_USART_AF);
	
	UBMS10_USART_CLK_Cmd(UBMS10_USART_CLK,ENABLE);
	USART_InitStruct.USART_BaudRate = UBMS10_USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(UBMS10_USART,&USART_InitStruct);
	
	USART_DMACmd(UBMS10_USART,USART_DMAReq_Rx,ENABLE);
	
	UBMS10_DMA_CLK_Cmd(UBMS10_DMA_CLK,ENABLE);	
	DMA_InitStruct.DMA_Channel = UBMS10_DMA_Channel;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&(UBMS10_DMABuffer));
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(UBMS10_USART->DR));
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = UBMS10_MAX_BUFFER_LENGTH;																	
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

	DMA_Init(UBMS10_DMA_Stream, &DMA_InitStruct);

	DMA_Cmd(UBMS10_DMA_Stream, ENABLE);	

	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(UBMS10_DMA_Stream, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UBMS10_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UBMS10_DMA_PreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UBMS10_DMA_SubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	USART_Cmd(UBMS10_USART,ENABLE);

}

void UBMS10_InitBattery(uint8_t _hours, uint8_t _minutes, uint8_t _seconds, float _battery_total)
{
	UBMS10_TransmitDataBuffer[8] = 'I';
	UBMS10_TransmitDataBuffer[9] = 'N';
	UBMS10_TransmitDataBuffer[10] = 'I';
	UBMS10_TransmitDataBuffer[11] = 'T';
	
	UBMS10_TransmitDataBuffer[12] = _hours;
	UBMS10_TransmitDataBuffer[13] = _minutes;
	UBMS10_TransmitDataBuffer[14] = _seconds;
	
	UBMS10_Convert_Float_to_Bytes(_battery_total, UBMS10_TransmitDataBuffer + 15);
	
	UBMS10_TransmitDataBuffer[35] = UBMS10_Checksum(UBMS10_TransmitDataBuffer);
	UBMS10_Transmit(UBMS10_TransmitDataBuffer);
}

void UBMS10_PassConfig(void)
{
	UBMS10_TransmitDataBuffer[8] = 'P';
	UBMS10_TransmitDataBuffer[9] = 'A';
	UBMS10_TransmitDataBuffer[10] = 'S';
	UBMS10_TransmitDataBuffer[11] = 'S';
	
	UBMS10_TransmitDataBuffer[35] = UBMS10_Checksum(UBMS10_TransmitDataBuffer);
	
	UBMS10_Transmit(UBMS10_TransmitDataBuffer);
}

void UBMS10_GetStatus(UBMS10_Status_Typedef *_ubms10_status)
{
	UBMS10_Status.Hours = _ubms10_status->Hours;
	UBMS10_Status.Minutes = _ubms10_status->Minutes;
	UBMS10_Status.Seconds = _ubms10_status->Seconds;
	UBMS10_Status.BatteryTotal = _ubms10_status->BatteryTotal;
	UBMS10_Status.BatteryCapacity = _ubms10_status->BatteryCapacity;
	UBMS10_Status.BatteryUsed = _ubms10_status->BatteryUsed;
	UBMS10_Status.BatteryPercentage = _ubms10_status->BatteryPercentage;
	UBMS10_Status.BatteryCurrent = _ubms10_status->BatteryCurrent;
	UBMS10_Status.BaterryVoltage = _ubms10_status->BaterryVoltage;
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void UBMS10_DMA_Stream_IRQHandler(void)
{	
	if(DMA_GetITStatus(UBMS10_DMA_Stream, DMA_IT_TCIF0) != RESET)
	{	
		if((UBMS10_DMABuffer[0] == '$') && (UBMS10_DMABuffer[1] == 'A') && (UBMS10_DMABuffer[2] == 'U') && (UBMS10_DMABuffer[3] == 'V') 
		&& (UBMS10_DMABuffer[4] == '2') && (UBMS10_DMABuffer[5] == '0') && (UBMS10_DMABuffer[6] == '0') && (UBMS10_DMABuffer[7] == '0') 
		&& (UBMS10_DMABuffer[35] == UBMS10_Checksum(UBMS10_DMABuffer)))
		{
			UBMS10_Status.Hours = UBMS10_DMABuffer[8];
			UBMS10_Status.Minutes = UBMS10_DMABuffer[9];
			UBMS10_Status.Seconds = UBMS10_DMABuffer[10];
			
			UBMS10_Status.BatteryTotal = UBMS10_Convert_Bytes_to_Float(UBMS10_DMABuffer + 11);
			UBMS10_Status.BatteryCapacity = UBMS10_Convert_Bytes_to_Float(UBMS10_DMABuffer + 15);
			UBMS10_Status.BatteryUsed = UBMS10_Convert_Bytes_to_Float(UBMS10_DMABuffer + 19);
			UBMS10_Status.BatteryPercentage = UBMS10_Convert_Bytes_to_Float(UBMS10_DMABuffer + 23);
			UBMS10_Status.BatteryCurrent = UBMS10_Convert_Bytes_to_Float(UBMS10_DMABuffer + 27);
			UBMS10_Status.BaterryVoltage = UBMS10_Convert_Bytes_to_Float(UBMS10_DMABuffer + 31);
		}
		DMA_ClearITPendingBit(UBMS10_DMA_Stream,DMA_IT_DMEIF0|DMA_IT_FEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0|DMA_IT_TEIF0);
	}
}

