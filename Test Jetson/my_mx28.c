/* Includes ------------------------------------------------------------------*/
#include "my_mx28.h"	//https://emanual.robotis.com/docs/en/dxl/mx/mx-28/

#define UMX28_BUFFER_LENGTH			14
/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static uint8_t UMX28_DMABuffer[UMX28_BUFFER_LENGTH], UMX28_DataBuffer[UMX28_BUFFER_LENGTH];

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   MX_28 Pin define 
		*/
			#define 	UMX28_TX_PIN  				   	GPIO_Pin_2			
			#define 	UMX28_TX_PORT 				  	GPIOA
			#define 	UMX28_TX_CLK							RCC_AHB1Periph_GPIOA			
			#define 	UMX28_TX_SOURCE				 		GPIO_PinSource2				
			#define		UMX28_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UMX28_RX_PIN  				   	GPIO_Pin_3
			#define 	UMX28_RX_PORT 				  	GPIOA
			#define 	UMX28_RX_CLK							RCC_AHB1Periph_GPIOA
			#define 	UMX28_RX_SOURCE				 		GPIO_PinSource3
			#define		UMX28_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			
			#define 	UMX28_TR_PIN  				   	GPIO_Pin_4			
			#define 	UMX28_TR_PORT 				  	GPIOA
			#define 	UMX28_TR_CLK							RCC_AHB1Periph_GPIOA			
			#define 	UMX28_TR_SOURCE				 		GPIO_PinSource2				
			#define		UMX28_TR_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd	
			
			#define		UMX28_USART								USART2
			#define 	UMX28_USART_CLK						RCC_APB1Periph_USART2
			#define		UMX28_USART_CLK_Cmd		  	RCC_APB1PeriphClockCmd
			#define 	UMX28_USART_BAUDRATE			57600
			#define		UMX28_USART_AF						GPIO_AF_USART2
			#define 	UMX28_USART_IRQn					USART2_IRQn
			#define		UMX28_USART_IRQPreemptionPriority	0x01
			#define		UMX28_USART_IRQSubPriority				0x01			
			
			#define   UMX28_DMA_Channel					DMA_Channel_4
			#define   UMX28_DMA_Stream					DMA1_Stream5
			#define		UMX28_DMA_CLK_Cmd		  		RCC_AHB1PeriphClockCmd
			#define 	UMX28_DMA_CLK							RCC_AHB1Periph_DMA1
			#define		UMX28_DMA_Stream_IRQn			DMA1_Stream5_IRQn
			#define		UMX28_DMA_IRQPreemptionPriority	0x00
			#define		UMX28_DMA_IRQSubPriority				0x04			
			
			#define		UMX28_DMA_Stream_IRQHandler			DMA1_Stream5_IRQHandler
			#define		UMX28_USART_IRQHandler					USART2_IRQHandler
			
									/** 
			* @brief   Timer Module define 
			*/
			#define 	UMX28_TIM  				      				TIM7
			#define 	UMX28_TIM_CLK					 	 				RCC_APB1Periph_TIM7	
			#define 	UMX28_TIM_CLK_Cmd    						RCC_APB1PeriphClockCmd
			#define 	UMX28_TIM_IRQn    							TIM7_IRQn				
			#define		UMX28_TIM_PreemptionPriority		0x00
			#define		UMX28_TIM_SubPriority						0x05		
			
			#define 	UMX28_TIM_IRQHandler						TIM7_IRQHandler	
/* Private variables ---------------------------------------------------------*/
MX28_Status_Typedef MX28_Status;

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/
void UMX28_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
	
//	UMX28_TR_CLK_Cmd(UMX28_TR_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = UMX28_TR_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(UMX28_TR_PORT,&GPIO_InitStruct);
//	GPIO_ResetBits(UMX28_TR_PORT, UMX28_TR_PIN);
	
	UMX28_TX_CLK_Cmd(UMX28_TX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UMX28_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UMX28_TX_PORT,&GPIO_InitStruct);
	
	UMX28_RX_CLK_Cmd(UMX28_RX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UMX28_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UMX28_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UMX28_TX_PORT,UMX28_TX_SOURCE,UMX28_USART_AF);
	GPIO_PinAFConfig(UMX28_RX_PORT,UMX28_RX_SOURCE,UMX28_USART_AF);
	
	UMX28_USART_CLK_Cmd(UMX28_USART_CLK,ENABLE);
	USART_InitStruct.USART_BaudRate = UMX28_USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(UMX28_USART,&USART_InitStruct);
	
//	USART_ITConfig(UMX28_USART, USART_IT_RXNE, ENABLE);
	
	USART_DMACmd(UMX28_USART,USART_DMAReq_Rx, ENABLE);
	
	UMX28_DMA_CLK_Cmd(UMX28_DMA_CLK,ENABLE);	
	DMA_InitStruct.DMA_Channel = UMX28_DMA_Channel;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&(UMX28_DMABuffer));
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(UMX28_USART->DR));
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = UMX28_BUFFER_LENGTH;																	
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

	DMA_Init(UMX28_DMA_Stream, &DMA_InitStruct);

	DMA_Cmd(UMX28_DMA_Stream, ENABLE);	

	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(UMX28_DMA_Stream, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UMX28_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UMX28_DMA_IRQPreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UMX28_DMA_IRQSubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
//	NVIC_InitStruct.NVIC_IRQChannel = UMX28_USART_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UMX28_USART_IRQPreemptionPriority;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UMX28_USART_IRQSubPriority;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
	
	USART_Cmd(UMX28_USART,ENABLE);
	
	UMX28_TIM_CLK_Cmd(UMX28_TIM_CLK, ENABLE);	
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_Period = 840 - 1;
	TIM_TimeBaseStruct.TIM_Prescaler = 1000 - 1;
	TIM_TimeBaseInit(UMX28_TIM, &TIM_TimeBaseStruct);
	
	TIM_ITConfig(UMX28_TIM, TIM_IT_Update, DISABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = UMX28_TIM_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UMX28_TIM_PreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UMX28_TIM_SubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_Cmd(UMX28_TIM, DISABLE);
}

void UMX28_AnalycsData(uint8_t *_data_in, uint8_t *_data_out)
{
	uint8_t _start = 0xFF;
	if((_data_in[11] == 0xFF) && (_data_in[12] == 0xFF) && (_data_in[13] == 0x01) && (_data_in[0] == 0x0A))
	{
		_start = 11;
	}
	else if((_data_in[12] == 0xFF) && (_data_in[13] == 0xFF) && (_data_in[0] == 0x01) && (_data_in[1] == 0x0A))
	{
		_start = 12;
	}			
	else if((_data_in[13] == 0xFF) && (_data_in[0] == 0xFF) && (_data_in[1] == 0x01) && (_data_in[2] == 0x0A))
	{
		_start = 13;
	}			
	else
	{
		for(uint8_t i = 1; i < UMX28_BUFFER_LENGTH; i++)
		{
			if((i < (UMX28_BUFFER_LENGTH - 3)) && (_data_in[i] == 0xFF) && (_data_in[i+1] == 0xFF) && (_data_in[i+2] == 0x01) && (_data_in[i+3] == 0x0A))
			{
				_start = i;
				break;
			}
		}
	}
	
	if(_start != 0xFF)
	{
		for(uint8_t i = 0; i < UMX28_BUFFER_LENGTH; i++)
		{
			if((_start + i) < UMX28_BUFFER_LENGTH)
			{
				_data_out[i] = _data_in[_start + i];
			}
			else 
			{
				_data_out[i] = _data_in[i - (UMX28_BUFFER_LENGTH - _start)];
			}
		}
	}
}

uint16_t UMX28_Convert_Byte2Uint(uint8_t *_data_in)
{
		union
		{
			uint8_t _part[2];
			uint16_t _value;
		}_data;
		
		_data._part[0] = _data_in[0];
		_data._part[1] = _data_in[1];
		
		return _data._value;
}

void UMX28_Transmit(uint8_t *szData, uint8_t _length)
{
//	GPIO_SetBits(UMX28_TR_PORT, UMX28_TR_PIN);
	for(uint8_t i = 0; i < _length; i++)
	{
		while(USART_GetFlagStatus(UMX28_USART,USART_FLAG_TXE) != SET);
		USART_SendData(UMX28_USART, szData[i]);
	}
//	UDELAY_us(200);
//	GPIO_ResetBits(UMX28_TR_PORT, UMX28_TR_PIN);
}

uint8_t UMX28_CheckSum(uint8_t *_data, uint16_t _length)
{
	uint16_t _tmp = 0;
	for (int i = 2; i < (_length - 1); i++)
	{
		_tmp += _data[i];
	}
	_tmp = ~(_tmp&0xFF);
	return (uint8_t)_tmp;
}

void UMX28_readStatusData(uint8_t _id)
{
	uint8_t _transmitData[8];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 0x04;
	_transmitData[4] = INT_READ;
	_transmitData[5] = PRESENT_POSITION_L;
	_transmitData[6] = 8;
	_transmitData[7] = UMX28_CheckSum(_transmitData,8);
	
	UMX28_Transmit(_transmitData, 8);
}

void UMX28_readVersionData(uint8_t _id)
{
	uint8_t _transmitData[8];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 0x04;
	_transmitData[4] = INT_READ;
	_transmitData[5] = MODEL_NUMBER_L;
	_transmitData[6] = 3;
	_transmitData[7] = UMX28_CheckSum(_transmitData,8);
	
	UMX28_Transmit(_transmitData, 8);
}

// 0-4095 ~ 0-360 (degree)
void UMX28_setGoalPosition(uint8_t _id, uint16_t _data)
{
	uint8_t _transmitData[9];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 3 + 2;
	_transmitData[4] = INT_WRITE;
	_transmitData[5] = GOAL_POSITION_L;
	_transmitData[6] = (uint8_t)(_data & 0x00FF);
	_transmitData[7] =(uint8_t)((_data & 0xFF00) >> 8);
	_transmitData[8] = UMX28_CheckSum(_transmitData, 9);
	
	UMX28_Transmit(_transmitData, 9);
}

/*0~2047 (0x000~0X7FF)
If a value is in the rage of 0~1023 then the motor rotates to the CCW direction.
If a value is in the rage of 1024~2047 then the motor rotates to the CW direction.
The 10th bit becomes the direction bit to control the direction; 0 and 1024 are equal.
The value unit is about 0.11rpm.
For example, if it is set to 300 then the motor is moving to the CCW direction at a rate of about 34.33rpm.*/
void UMX28_setMovingSpeed(uint8_t _id, uint16_t _data)
{
	uint8_t _transmitData[9];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 3 + 2;
	_transmitData[4] = INT_WRITE;
	_transmitData[5] = MOVING_SPEED_L;
	_transmitData[6] = (uint8_t)(_data & 0x00FF);
	_transmitData[7] =(uint8_t)((_data & 0xFF00) >> 8);
	_transmitData[8] = UMX28_CheckSum(_transmitData, 9);
	
	UMX28_Transmit(_transmitData, 9);
}

void UMX28_setKp(uint8_t _id, uint8_t _data)
{
	uint8_t _transmitData[8];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 3 + 1;
	_transmitData[4] = INT_WRITE;
	_transmitData[5] = P_GAIN;
	_transmitData[6] = _data;
	_transmitData[7] = UMX28_CheckSum(_transmitData, 8);
	
	UMX28_Transmit(_transmitData, 8);
}

void UMX28_setKi(uint8_t _id, uint8_t _data)
{
	uint8_t _transmitData[8];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 3 + 1;
	_transmitData[4] = INT_WRITE;
	_transmitData[5] = I_GAIN;
	_transmitData[6] = _data;
	_transmitData[7] = UMX28_CheckSum(_transmitData, 8);
	
	UMX28_Transmit(_transmitData, 8);
}

void UMX28_setKd(uint8_t _id, uint8_t _data)
{
	uint8_t _transmitData[8];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 3 + 1;
	_transmitData[4] = INT_WRITE;
	_transmitData[5] = D_GAIN;
	_transmitData[6] = _data;
	_transmitData[7] = UMX28_CheckSum(_transmitData, 8);
	
	UMX28_Transmit(_transmitData, 8);
}

void UMX28_getPIDGain(uint8_t _id)
{
	uint8_t _transmitData[6];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 0x02;
	_transmitData[4] = INT_PING;
	_transmitData[5] = UMX28_CheckSum(_transmitData, 6);
	
	UMX28_Transmit(_transmitData, 6);
}

void UMX28_setBaudrate(uint8_t _id, MX28_Baudrate_Typedef _baudrate)
{
	uint8_t _transmitData[8];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 3 + 1;
	_transmitData[4] = INT_WRITE;
	_transmitData[5] = BAUDRATE;
	_transmitData[6] = _baudrate;
	_transmitData[7] = UMX28_CheckSum(_transmitData, 8);
	
	UMX28_Transmit(_transmitData, 8);
}

void UMX28_pingServo(uint8_t _id)
{
	uint8_t _transmitData[6];
	
	_transmitData[0] = 0xFF;
	_transmitData[1] = 0xFF;
	_transmitData[2] = _id;
	_transmitData[3] = 2;
	_transmitData[4] = INT_PING;
	_transmitData[5] = UMX28_CheckSum(_transmitData, 6);
	
	UMX28_Transmit(_transmitData, 6);
}

float UMX28_getStallTorque(float _voltage)
{
	float _tmp_torque = 0;
	if (_voltage < 12.0f)
	{
		_tmp_torque = 2.3f;
	}
	else if (_voltage < 14.8f)
	{
		_tmp_torque = 2.5f;
	}
	else
	{
		_tmp_torque = 3.1f;
	}
	return _tmp_torque;
}

void UMX28_Init(void)
{
	UMX28_pingServo(254);		
	UDELAY_us(500);
	UMX28_setBaudrate(254, BPS_57600);
	UDELAY_us(500);
	// 2500 -> 0o
	UMX28_setGoalPosition(254, 2500);	
	UDELAY_us(500);	
	
	TIM_ITConfig(UMX28_TIM, TIM_IT_Update, ENABLE);
	TIM_Cmd(UMX28_TIM, ENABLE);
}

void UMX28_getStatus(MX28_Status_Typedef *_mx28_status)
{
	_mx28_status->Load = MX28_Status.Load;
	_mx28_status->Position = MX28_Status.Position;
	_mx28_status->Speed = MX28_Status.Speed;
	_mx28_status->Temperature = MX28_Status.Temperature;
	_mx28_status->Voltage = MX28_Status.Voltage;
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void UMX28_DMA_Stream_IRQHandler(void)
{	
	if(DMA_GetITStatus(UMX28_DMA_Stream, DMA_IT_TCIF5) != RESET)
	{	
		UMX28_AnalycsData(UMX28_DMABuffer, UMX28_DataBuffer);
		
		if(UMX28_DataBuffer[13] == UMX28_CheckSum(UMX28_DataBuffer, 14))
		{
			MX28_Status.Position = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 5);
			
			if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) <= 1023.0f)
			{				
				MX28_Status.Speed = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) * 0.11f;
			}
			else if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) >= 1024.0f)
			{
				float _temp = 0.0f;
				_temp = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) - 1024.0f;
				MX28_Status.Speed = _temp * -0.11f;
			}
			
			MX28_Status.Temperature = (float)UMX28_DataBuffer[12];
			
			MX28_Status.Voltage = (float)UMX28_DataBuffer[11]/10.0f;
			
			if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) <= 1023.0f)
			{
				MX28_Status.Load = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) * UMX28_getStallTorque(MX28_Status.Voltage)/1023.0f;
			}
			else if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) >= 1024.0f)
			{
				float _temp = 0.0f;
				_temp = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) - 1024.0f;
				MX28_Status.Load = (float)_temp * -UMX28_getStallTorque(MX28_Status.Voltage) / 1023.0f;
			}
		}
		DMA_ClearITPendingBit(UMX28_DMA_Stream,DMA_IT_DMEIF5|DMA_IT_FEIF5|DMA_IT_HTIF5|DMA_IT_TCIF5|DMA_IT_TEIF5);
	}
}

void UMX28_TIM_IRQHandler(void)
{
	if(TIM_GetITStatus(UMX28_TIM, TIM_IT_Update) != RESET)
	{
		UMX28_readStatusData(1);
		TIM_ClearITPendingBit(UMX28_TIM, TIM_IT_Update);
	}
}

//void UMX28_USART_IRQHandler(void)
//{
//	if(USART_GetITStatus(UMX28_USART,USART_IT_RXNE) != RESET)
//	{
//		UMX28_DataBuffer[count_RXData] = USART_ReceiveData(UMX28_USART);
//		count_RXData++;
//		if(count_RXData > UMX28_BUFFER_LENGTH) count_RXData = 0;
//		
////			if((UMX28_DataBuffer[0] == 0xFF) && (UMX28_DataBuffer[1] == 0xFF) && (UMX28_DataBuffer[2] == 0x01))
////	{
////		_mx28_status->Position = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 5);
////		
////		if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) <= 1023.0f)
////		{				
////			_mx28_status->Speed = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) * 0.11f;
////		}
////		else if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) >= 1024.0f)
////		{
////			float _temp = 0.0f;
////			_temp = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 7) - 1024.0f;
////			_mx28_status->Speed = _temp * -0.11f;
////		}
////		
////		_mx28_status->Temperature = (float)UMX28_DataBuffer[12];
////		
////		_mx28_status->Voltage = (float)UMX28_DataBuffer[11]/10.0f;
////		
////		if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) <= 1023.0f)
////		{
////			_mx28_status->Load = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) * UMX28_getStallTorque(_mx28_status->Voltage)/1023.0f;
////		}
////		else if ((float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) >= 1024.0f)
////		{
////			float _temp = 0.0f;
////			_temp = (float)UMX28_Convert_Byte2Uint(UMX28_DataBuffer + 9) - 1024.0f;
////			_mx28_status->Load = (float)_temp * -UMX28_getStallTorque(_mx28_status->Voltage) / 1023.0f;
////		}
////	}
//	}
//	
//	if(USART_GetITStatus(UMX28_USART,USART_IT_IDLE) != RESET)
//	{
//		
//		USART_ClearITPendingBit(UMX28_USART,USART_IT_IDLE);
//	}
//}
