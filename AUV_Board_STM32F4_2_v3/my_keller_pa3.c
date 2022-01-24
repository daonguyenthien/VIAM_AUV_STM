/*
 * my_keller_pa3.c
 *
 *  Created on: Apr 11, 2018
 *      Author: Chau Thanh Hai
 */
#include "my_keller_pa3.h"

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   UKELLER Pin define 
		*/
			#define 	UKELLER_TX_PIN  				   	GPIO_Pin_10			
			#define 	UKELLER_TX_PORT 				  	GPIOB
			#define 	UKELLER_TX_CLK							RCC_AHB1Periph_GPIOB			
			#define 	UKELLER_TX_SOURCE				 		GPIO_PinSource10				
			#define		UKELLER_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UKELLER_RX_PIN  				   	GPIO_Pin_11
			#define 	UKELLER_RX_PORT 				  	GPIOB
			#define 	UKELLER_RX_CLK							RCC_AHB1Periph_GPIOB
			#define 	UKELLER_RX_SOURCE				 		GPIO_PinSource11
			#define		UKELLER_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			
			#define		UKELLER_USART								USART3
			#define 	UKELLER_USART_CLK						RCC_APB1Periph_USART3
			#define		UKELLER_USART_CLK_Cmd		 		RCC_APB1PeriphClockCmd
			#define 	UKELLER_USART_BAUDRATE			9600
			#define		UKELLER_USART_AF						GPIO_AF_USART3
			#define 	UKELLER_USART_IRQn					USART3_IRQn
			#define 	UKELLER_USART_IRQPreemptionPriority		0x00
			#define 	UKELLER_USART_IRQSubPriority					0x02
			
			#define   UKELLER_DMA_Channel					DMA_Channel_4
			#define   UKELLER_DMA_Stream					DMA1_Stream1
			#define		UKELLER_DMA_CLK_Cmd		  		RCC_AHB1PeriphClockCmd
			#define 	UKELLER_DMA_CLK							RCC_AHB1Periph_DMA1
			#define		UKELLER_DMA_Stream_IRQn			DMA1_Stream1_IRQn
			#define 	UKELLER_DMA_IRQPreemptionPriority		0x00
			#define 	UKELLER_DMA_IRQSubPriority					0x01
			
			#define		UKELLER_DMA_Stream_IRQHandler			DMA1_Stream1_IRQHandler
			#define		UKELLER_USART_IRQHandler					USART3_IRQHandler

#define UKELLER_BUFFER_LENGTH 9

//static uint8_t Data_Count = 0;
static uint8_t SC_Buffer[9];
//static uint8_t UKELLER_RXBuffer[UKELLER_BUFFER_LENGTH];
static uint8_t UKELLER_DMABuffer[UKELLER_BUFFER_LENGTH];
static uint8_t Mode = 0;
UKellerPA3_Status_Typedef UKellerPA3_Status;

void UKELLER_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	UKELLER_TX_CLK_Cmd(UKELLER_TX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UKELLER_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UKELLER_TX_PORT,&GPIO_InitStruct);
	
	UKELLER_RX_CLK_Cmd(UKELLER_RX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UKELLER_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UKELLER_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UKELLER_TX_PORT,UKELLER_TX_SOURCE,UKELLER_USART_AF);
	GPIO_PinAFConfig(UKELLER_RX_PORT,UKELLER_RX_SOURCE,UKELLER_USART_AF);
	
	UKELLER_USART_CLK_Cmd(UKELLER_USART_CLK,ENABLE);
	USART_InitStruct.USART_BaudRate = UKELLER_USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(UKELLER_USART,&USART_InitStruct);
	
	UKELLER_DMA_CLK_Cmd(UKELLER_DMA_CLK,ENABLE);	
	DMA_InitStruct.DMA_Channel = UKELLER_DMA_Channel;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&(UKELLER_DMABuffer));
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(UKELLER_USART->DR));
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = UKELLER_BUFFER_LENGTH;																	
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

	DMA_Init(UKELLER_DMA_Stream, &DMA_InitStruct);
	
	USART_DMACmd(UKELLER_USART,USART_DMAReq_Rx, DISABLE);
	DMA_Cmd(UKELLER_DMA_Stream, DISABLE);	
	
	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(UKELLER_DMA_Stream, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UKELLER_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UKELLER_DMA_IRQPreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UKELLER_DMA_IRQSubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
//	USART_ITConfig(UKELLER_USART, USART_IT_RXNE, DISABLE);
	
	USART_Cmd(UKELLER_USART,ENABLE);

}

static float UKELLER_Convert_Bytes_to_Float(uint8_t* _data_in)
{
	union
	{
		float _value;
		uint8_t _byte[4];
	}_part;
	
	_part._byte[0] = _data_in[3];
	_part._byte[1] = _data_in[2];
	_part._byte[2] = _data_in[1];
	_part._byte[3] = _data_in[0];
	
	return _part._value; 
}

//////////////////////////////////////////////////////////////////////////
// CRC-16 calculation in C
//
// Calculation of CRC-16 checksum over an amount of bytes in the serial buffer.
// The calculation is done without the 2byte from crc16 (receive-mode).
// SC_Buffer[]: Byte-Buffer for the serial interface. Type: unsigned char (8bit)
// SC_Amount : Amount of Bytes which should be transmitted or are received (without CRC16)
//
//////////////////////////////////////////////////////////////////////////
void CalcCRC16(uint8_t _buffer[], uint8_t _length, unsigned char* CRC_H, unsigned char* CRC_L)
{
	// locals
	unsigned int Crc;
	unsigned char n, m, x;
	// initialisation
	Crc= 0xFFFF;
	m= _length;
	x= 0;
	// loop over all bits
	while(m>0)
	{
		Crc^= _buffer[x];
		for(n=0; n<8; n++)
	{
	if(Crc&1)
	{
		Crc>>= 1;
		Crc^= 0xA001;
	}
	else
		Crc>>= 1;
	}
	m--;
	x++;
	}
	// result
	*CRC_H= (Crc>>8)&0xFF;
	*CRC_L= Crc&0xFF;
}// end CalcCRC16

void UKELLER_MODBUS_Read(uint8_t DevAddr, uint16_t StAddr, uint16_t NoReg)
{
	SC_Buffer[0] = DevAddr;
	SC_Buffer[1] = MODBUS_READ;
	SC_Buffer[2] = (uint8_t)((StAddr >> 8)&0xFF);
	SC_Buffer[3] = (uint8_t)(StAddr&0xFF);
	SC_Buffer[4] = (uint8_t)((NoReg >> 8)&0xFF);
	SC_Buffer[5] = (uint8_t)(NoReg&0xFF);
	CalcCRC16(SC_Buffer, 6, &SC_Buffer[6], &SC_Buffer[7]);
	
	for(int i = 0; i < 8; i++)
	{
		UKELLER_TxData(SC_Buffer[i]);
	}
	while(USART_GetFlagStatus(UKELLER_USART,USART_FLAG_TXE) != SET); 
}

void UKELLER_MODBUS_WriteSingle(uint8_t DevAddr, uint16_t StAddr, uint16_t Data)
{
	SC_Buffer[0] = DevAddr;
	SC_Buffer[1] = MODBUS_WRITE;
	SC_Buffer[2] = (uint8_t)((StAddr >> 8)&0xFF);
	SC_Buffer[3] = (uint8_t)(StAddr&0xFF);
	SC_Buffer[4] = (uint8_t)((Data >> 8)&0xFF);
	SC_Buffer[5] = (uint8_t)(Data&0xFF);
	CalcCRC16(SC_Buffer, 6, &SC_Buffer[6], &SC_Buffer[7]);
	for(int i = 0; i < 8; i++)
	{
		UKELLER_TxData(SC_Buffer[i]);
	}
	while(USART_GetFlagStatus(UKELLER_USART,USART_FLAG_TXE) != SET); 
}

void UKELLER_KELLER_ReadP1(uint8_t DevAddr)
{
	SC_Buffer[0] = DevAddr;
	SC_Buffer[1] = KELLER_READ;
	SC_Buffer[2] = 0x01;
	CalcCRC16(SC_Buffer, 3, &SC_Buffer[3], &SC_Buffer[4]);
	for(int i = 0; i < 5; i++)
	{
		UKELLER_TxData(SC_Buffer[i]);
	}
	while(USART_GetFlagStatus(UKELLER_USART,USART_FLAG_TXE) != SET); 	
}

void UKELLER_KELLER_ReadTOB1(uint8_t DevAddr)
{
	SC_Buffer[0] = DevAddr;
	SC_Buffer[1] = KELLER_READ;
	SC_Buffer[2] = 0x04;
	CalcCRC16(SC_Buffer, 3, &SC_Buffer[3], &SC_Buffer[4]);
	for(int i = 0; i < 5; i++)
	{
		UKELLER_TxData(SC_Buffer[i]);
	}
	while(USART_GetFlagStatus(UKELLER_USART,USART_FLAG_TXE) != SET);
}

void UKELLER_KELLER_Init(uint8_t DevAddr)
{
	uint8_t _tmp[10];
	bool _check_sensor = false;
	unsigned char _CRC_H, _CRC_L;
	
	SC_Buffer[0] = DevAddr;
	SC_Buffer[1] = KELLER_INIT;
	SC_Buffer[2] = 0x04;
	SC_Buffer[3] = 0x43;
	for(int i = 0; i < 4; i++)
	{
		UKELLER_TxData(SC_Buffer[i]);
	}
	while(USART_GetFlagStatus(UKELLER_USART,USART_FLAG_TXE) != SET); 
	
	while(_check_sensor == false)
	{
		for(uint8_t i = 0; i < 10; i++)
		{
			while(USART_GetFlagStatus(UKELLER_USART,USART_FLAG_RXNE) != SET);
			_tmp[i] = USART_ReceiveData(UKELLER_USART);
		}
		CalcCRC16(_tmp, 8, &_CRC_H, &_CRC_L);
		if((_tmp[2] == 5) && (_tmp[8] == _CRC_H) && (_tmp[9] == _CRC_L))
		{
			_check_sensor = true;
		}
	}
	
  /* Start DMA transfer */
	USART_DMACmd(UKELLER_USART,USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(UKELLER_DMA_Stream, ENABLE);	
	
	UDELAY_ms(10);
	
	UKELLER_KELLER_ReadP1(250);
//	USART_ITConfig(UKELLER_USART, USART_IT_RXNE, ENABLE);
}

void UKELLER_TxData(uint8_t data)
{
	while(USART_GetFlagStatus(UKELLER_USART,USART_FLAG_TXE) != SET);
	USART_SendData(UKELLER_USART,data);
}

void UKELLER_GetStatus(UKellerPA3_Status_Typedef *_ukeller_status)
{
	_ukeller_status->Pressure = UKellerPA3_Status.Pressure;
	_ukeller_status->Temperature = UKellerPA3_Status.Temperature;
}

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
//void UKELLER_USART_IRQHandler(void)
//{
//	unsigned char _CRC_H, _CRC_L;
//	if(USART_GetITStatus(UKELLER_USART, USART_IT_RXNE) != RESET)
//	{
//		UKELLER_RXBuffer[Data_Count] = USART_ReceiveData(UKELLER_USART);
//		Data_Count++;
//		if(Data_Count >= UKELLER_BUFFER_LENGTH) Data_Count = 0;
//		CalcCRC16(UKELLER_RXBuffer, 7, &_CRC_H, &_CRC_L);
//		if((UKELLER_RXBuffer[7] == _CRC_H)&&(UKELLER_RXBuffer[8] == _CRC_L))
//		{
//			UKELLER_Temperature = UKELLER_Convert_Bytes_to_Float(UKELLER_RXBuffer + 2);
//		}		
//	}
//}

void UKELLER_DMA_Stream_IRQHandler(void)
{
	unsigned char _CRC_H, _CRC_L;
	if (DMA_GetITStatus(UKELLER_DMA_Stream, DMA_IT_TCIF1) != RESET)
	{
			CalcCRC16(UKELLER_DMABuffer, 7, &_CRC_H, &_CRC_L);
			if((UKELLER_DMABuffer[7] == _CRC_H)&&(UKELLER_DMABuffer[8] == _CRC_L))
			{
				if(Mode == 0)
				{
					UKellerPA3_Status.Pressure = UKELLER_Convert_Bytes_to_Float(UKELLER_DMABuffer + 2);
				}
				else
				{
					UKellerPA3_Status.Temperature = UKELLER_Convert_Bytes_to_Float(UKELLER_DMABuffer + 2);
				}
			}

			if(Mode == 0)	
			{
				UKELLER_KELLER_ReadTOB1(250);				
				Mode++;
			}
			else
			{
				UKELLER_KELLER_ReadP1(250);
				Mode = 0;
			}						
			DMA_ClearITPendingBit(UKELLER_DMA_Stream, DMA_IT_TCIF1);
	}
}
