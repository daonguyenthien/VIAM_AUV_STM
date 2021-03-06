/*
 * User_CAN.c
 *
 *  Created on: Apr 9, 2018
 *      Author: Chau Thanh Hai
 */
#include "my_can.h"
#include "UGV_UART_JOYSTICK.h"
#include "UGV_SBUS.h"
#include "my_delay.h"
/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   CAN Pin define 
		*/
			#define 	UCAN_TX_PIN  				   	GPIO_Pin_9			
			#define 	UCAN_TX_PORT 				  	GPIOB
			#define 	UCAN_TX_CLK							RCC_AHB1Periph_GPIOB			
			#define 	UCAN_TX_SOURCE				 	GPIO_PinSource9				
			#define		UCAN_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UCAN_RX_PIN  				   	GPIO_Pin_8
			#define 	UCAN_RX_PORT 				  	GPIOB
			#define 	UCAN_RX_CLK							RCC_AHB1Periph_GPIOB
			#define 	UCAN_RX_SOURCE				 	GPIO_PinSource8
			#define		UCAN_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		

			#define		UCAN_CAN								CAN1
			#define 	UCAN_CAN_CLK						RCC_APB1Periph_CAN1
			#define		UCAN_CAN_CLK_Cmd		  	RCC_APB1PeriphClockCmd
			#define		UCAN_CAN_AF							GPIO_AF_CAN1
			#define 	UCAN_CAN_IRQn						CAN1_RX0_IRQn
			
			#define		UCAN_CAN_IRQHandler			CAN1_RX0_IRQHandler
			
			/** 
			* @brief   Timer Module define 
			*/
			#define 	UCAN_TIM  				      				TIM5
			#define 	UCAN_TIM_CLK					 	 				RCC_APB1Periph_TIM5	
			#define 	UCAN_TIM_CLK_Cmd    						RCC_APB1PeriphClockCmd
			#define 	UCAN_TIM_IRQn    								TIM5_IRQn				
			#define		UCAN_TIM_PreemptionPriority			0x01
			#define		UCAN_TIM_SubPriority						0x05		
			
			#define 	UCAN_TIM_IRQHandler							TIM5_IRQHandler	
			
			#define 	UCAN_TIM_2  				      			TIM3
			#define 	UCAN_TIM_CLK_2				 	 				RCC_APB1Periph_TIM3
			#define 	UCAN_TIM_CLK_Cmd_2   						RCC_APB1PeriphClockCmd
			#define 	UCAN_TIM_IRQn_2    							TIM3_IRQn				
			#define		UCAN_TIM_PreemptionPriority_2		0x00
			#define		UCAN_TIM_SubPriority_2					0x01		
			
			#define 	UCAN_TIM_IRQHandler_2						TIM3_IRQHandler
			
#define MODULE_CAN_MCP251x 

#ifdef MODULE_CAN_MCP251x
	static const uint8_t num_frames_revc4req = 2;
#else
	static const uint8_t num_frames_revc4req = 1;
#endif

/**
 * @}
 */
static bool _need_respond_data = false;
static bool _system_ready = false;
static bool _need_system_ready = false;
uint8_t _can_respond_count = 0;

static int _IDCANBUS_ARM_1 = 0x121;
static int _IDCANBUS_ARM_2 = 0x122;

static int _IDCANBUS_THRUSTER = 0x123;
static int _IDCANBUS_PISTOL = 0x124;

static int _IDCANBUS_MASS_SHIFTER = 0x125;
static int _IDCANBUS_EPC = 0x126;

uint8_t CAN_RxMessage[8];
uint8_t CAN_TxMessage[8];

uint8_t leak_position = 0;
bool check = false;

static union CAN_UpdateData
{
	float Value;
	
	struct
	{
		unsigned a1:8;
		unsigned a2:8;
		unsigned a3:8;
		unsigned a4:8;
	} byte;
} UPWM_DutyCycle,Thruster,Rudder,Mass,Pistol,Altimeter;

CanRxMsg RxMessage;

void UCAN_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	CAN_InitTypeDef        CAN_InitStruct;
	CAN_FilterInitTypeDef  CAN_FilterInitStruct;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStruct;
	
	UCAN_TX_CLK_Cmd(UCAN_TX_CLK,ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UCAN_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UCAN_TX_PORT,&GPIO_InitStruct);

	UCAN_RX_CLK_Cmd(UCAN_RX_CLK,ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UCAN_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UCAN_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UCAN_TX_PORT,UCAN_TX_SOURCE,UCAN_CAN_AF);
	GPIO_PinAFConfig(UCAN_RX_PORT,UCAN_RX_SOURCE,UCAN_CAN_AF);
	
	UCAN_CAN_CLK_Cmd(UCAN_CAN_CLK,ENABLE);

	/* CAN register init */
	CAN_DeInit(UCAN_CAN);

/* CAN cell init */
	CAN_InitStruct.CAN_TTCM = DISABLE;
	CAN_InitStruct.CAN_ABOM = DISABLE;
	CAN_InitStruct.CAN_AWUM = DISABLE;
	CAN_InitStruct.CAN_NART = ENABLE;
	CAN_InitStruct.CAN_RFLM = DISABLE;
	CAN_InitStruct.CAN_TXFP = ENABLE;
	CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;

	/* CAN Baudrate = 1 MBps (CAN clocked at 42 MHz) */
	// Baudrate = 1/(tq + tBS1 + tBS2)
	//http://www.bittiming.can-wiki.info/
	CAN_InitStruct.CAN_BS1 = CAN_BS1_12tq;
	CAN_InitStruct.CAN_BS2 = CAN_BS2_8tq;
	CAN_InitStruct.CAN_Prescaler = 2;
	CAN_Init(UCAN_CAN, &CAN_InitStruct);

	/* CAN filter init */
	CAN_FilterInitStruct.CAN_FilterNumber = 0;
	CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_16bit;
	CAN_FilterInitStruct.CAN_FilterIdHigh = 0x122 << 5;
	CAN_FilterInitStruct.CAN_FilterIdLow = 0x122 << 5;
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x122 << 5;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x122 << 5;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);

//	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(UCAN_CAN, CAN_IT_FMP0, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
	
//	TIM_ITConfig(UCAN_TIM, TIM_IT_Update, DISABLE);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UCAN_TIM_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UCAN_TIM_PreemptionPriority;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UCAN_TIM_SubPriority;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	
//	TIM_Cmd(UCAN_TIM, DISABLE);

	UCAN_TIM_CLK_Cmd_2(UCAN_TIM_CLK_2,ENABLE);
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 72*100-1;
	TIM_InitStructure.TIM_Period = 100;
	TIM_TimeBaseInit(UCAN_TIM_2, &TIM_InitStructure);
	TIM_Cmd(UCAN_TIM_2, ENABLE);
//	TIM_ITConfig(UCAN_TIM_2, TIM_IT_Update, ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = UCAN_TIM_IRQn_2;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UCAN_TIM_PreemptionPriority_2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UCAN_TIM_SubPriority_2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void UCAN_Transmit(int _IDstd,int _length, uint8_t _data[])
{
	CanTxMsg _TxMessage;
  uint8_t mailbox;
	uint8_t status;
  _TxMessage.StdId = _IDstd;
	_TxMessage.ExtId = 0x00;
  _TxMessage.RTR = CAN_RTR_DATA;
  _TxMessage.IDE = CAN_ID_STD;
  _TxMessage.DLC = _length;
	for(int j = 0;j < _length; j++)
	{
		_TxMessage.Data[j] = _data[j];
	}

  mailbox = CAN_Transmit(UCAN_CAN,&_TxMessage);

	//wait until CAN transmission is OK
	int32_t i = 0;
  while((status != CANTXOK) && (i != 0xFFFF))
  {
    status = CAN_TransmitStatus(UCAN_CAN,mailbox);
    i++;
  }
//	UDELAY_us(100);
}

float UCAN_Convert_Bytes_to_Float(uint8_t* _data_in)
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



void UCAN_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out)
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

uint16_t UCAN_Convert_Byte2Uint16(uint8_t *_data_in)
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

void UCAN_GetDataReceive(uint8_t* _data)
{
	_data = CAN_RxMessage;
}

uint8_t UCAN_Checksum(uint8_t *_data)
{
	uint8_t value = 0;

	//Calculate CheckSum (Byte)
	for (int i = 0; i < 7; i++)
	{
		value += _data[i];
	}
	value = ~value;
	value++;
	return (uint8_t)value;
}

//void UCAN_SystemReady(void)
//{
//	CAN_TxMessage[0] = 'A';
//	CAN_TxMessage[1] = 'R';
//	CAN_TxMessage[2] = 'M';
//	CAN_TxMessage[3] = '2';
//	CAN_TxMessage[4] = 'R';
//	CAN_TxMessage[5] = 'D';
//	CAN_TxMessage[6] = 'Y';
//	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
//	
//	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
//}

//void UCAN_SystemNotReady(void)
//{
//	CAN_TxMessage[0] = 'A';
//	CAN_TxMessage[1] = 'R';
//	CAN_TxMessage[2] = 'M';
//	CAN_TxMessage[3] = '2';
//	CAN_TxMessage[4] = 'N';
//	CAN_TxMessage[5] = 'O';
//	CAN_TxMessage[6] = 'T';
//	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
//	
//	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
//}
void Close_Thruster(void)
{
	CAN_TxMessage[0] = 'C';
	CAN_TxMessage[1] = 'A';
	CAN_TxMessage[2] = 'N';
	CAN_TxMessage[3] = 'C';
	CAN_TxMessage[4] = 0;
	CAN_TxMessage[5] = 0;
	CAN_TxMessage[6] = 0;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	
	UCAN_Transmit(_IDCANBUS_THRUSTER, 8, CAN_TxMessage);
}

void Run_Thruster(float speed_percent)
{
	UPWM_DutyCycle.Value = speed_percent;
	CAN_TxMessage[0] = 'O';
	CAN_TxMessage[1] = 'L';
	if(speed_percent >= 0)
	{
		CAN_TxMessage[2] = 'R';	//rotate Right
	}
	else
	{
		CAN_TxMessage[2] = 'L';	//rotate Left
	}
	CAN_TxMessage[3] = UPWM_DutyCycle.byte.a4;
	CAN_TxMessage[4] = UPWM_DutyCycle.byte.a3;
	CAN_TxMessage[5] = UPWM_DutyCycle.byte.a2;
	CAN_TxMessage[6] = UPWM_DutyCycle.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(_IDCANBUS_THRUSTER, 8, CAN_TxMessage);
}

void Open_Piston(void)
{
	CAN_TxMessage[0] = 'C';
	CAN_TxMessage[1] = 'A';
	CAN_TxMessage[2] = 'N';
	CAN_TxMessage[3] = 'O';
	CAN_TxMessage[4] = 'P';
	CAN_TxMessage[5] = 0;
	CAN_TxMessage[6] = 0;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	
	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
}

void Run_Piston(float speed_percent)
{
	UPWM_DutyCycle.Value = speed_percent;
	CAN_TxMessage[0] = 'O';
	CAN_TxMessage[1] = 'P';
	if(speed_percent >= 0)
	{
		CAN_TxMessage[2] = 'R';	//rotate Right
	}
	else
	{
		CAN_TxMessage[2] = 'L';	//rotate Left
	}
	CAN_TxMessage[3] = UPWM_DutyCycle.byte.a4;
	CAN_TxMessage[4] = UPWM_DutyCycle.byte.a3;
	CAN_TxMessage[5] = UPWM_DutyCycle.byte.a2;
	CAN_TxMessage[6] = UPWM_DutyCycle.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
}

void Open_Shifter(void)
{
	CAN_TxMessage[0] = 'C';
	CAN_TxMessage[1] = 'A';
	CAN_TxMessage[2] = 'N';
	CAN_TxMessage[3] = 'O';
	CAN_TxMessage[4] = 'S';
	CAN_TxMessage[5] = 0;
	CAN_TxMessage[6] = 0;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	
	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
}

void Run_Shifter(float speed_percent)
{
	UPWM_DutyCycle.Value = speed_percent;
	CAN_TxMessage[0] = 'O';
	CAN_TxMessage[1] = 'S';
	if(speed_percent >= 0)
	{
		CAN_TxMessage[2] = 'R';	//rotate Right
	}
	else
	{
		CAN_TxMessage[2] = 'L';	//rotate Left
	}
	CAN_TxMessage[3] = UPWM_DutyCycle.byte.a4;
	CAN_TxMessage[4] = UPWM_DutyCycle.byte.a3;
	CAN_TxMessage[5] = UPWM_DutyCycle.byte.a2;
	CAN_TxMessage[6] = UPWM_DutyCycle.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
}

//---------------------------------------------

//void Open_Piston(void)
//{
//	CAN_TxMessage[0] = 'C';
//	CAN_TxMessage[1] = 'A';
//	CAN_TxMessage[2] = 'N';
//	CAN_TxMessage[3] = 'O';
//	CAN_TxMessage[4] = 0;
//	CAN_TxMessage[5] = 0;
//	CAN_TxMessage[6] = 0;
//	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
//	
//	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
//}

//void Run_Piston(float speed_percent)
//{
//	UPWM_DutyCycle.Value = speed_percent;
//	CAN_TxMessage[0] = 'O';
//	CAN_TxMessage[1] = 'L';
//	if(speed_percent >= 0)
//	{
//		CAN_TxMessage[2] = 'R';	//rotate Right
//	}
//	else
//	{
//		CAN_TxMessage[2] = 'L';	//rotate Left
//	}
//	CAN_TxMessage[3] = UPWM_DutyCycle.byte.a4;
//	CAN_TxMessage[4] = UPWM_DutyCycle.byte.a3;
//	CAN_TxMessage[5] = UPWM_DutyCycle.byte.a2;
//	CAN_TxMessage[6] = UPWM_DutyCycle.byte.a1;
//	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
//	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
//}

//void Open_Shifter(void)
//{
//	CAN_TxMessage[0] = 'C';
//	CAN_TxMessage[1] = 'A';
//	CAN_TxMessage[2] = 'N';
//	CAN_TxMessage[3] = 'O';
//	CAN_TxMessage[4] = 0;
//	CAN_TxMessage[5] = 0;
//	CAN_TxMessage[6] = 0;
//	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
//	
//	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
//}

//void Run_Shifter(float speed_percent)
//{
//	UPWM_DutyCycle.Value = speed_percent;
//	CAN_TxMessage[0] = 'O';
//	CAN_TxMessage[1] = 'L';
//	if(speed_percent >= 0)
//	{
//		CAN_TxMessage[2] = 'R';	//rotate Right
//	}
//	else
//	{
//		CAN_TxMessage[2] = 'L';	//rotate Left
//	}
//	CAN_TxMessage[3] = UPWM_DutyCycle.byte.a4;
//	CAN_TxMessage[4] = UPWM_DutyCycle.byte.a3;
//	CAN_TxMessage[5] = UPWM_DutyCycle.byte.a2;
//	CAN_TxMessage[6] = UPWM_DutyCycle.byte.a1;
//	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
//	UCAN_Transmit(_IDCANBUS_ARM_1, 8, CAN_TxMessage);
//}

//======================================================
bool UCAN_IsSystemReady(void)
{
	return _system_ready;
}

void UCAN_PassSystemReady(FunctionalState NewState)
{
	_system_ready = (NewState == ENABLE)?true:false;
}

void UCAN_Respond_ALLData(CAN_DataTypeDef *_can_data)
{
//	//--------LEAK SENSORS--------//
//	CAN_TxMessage[0] = ARM2_LEAK_SENSOR;
//	CAN_TxMessage[1] = STATUS_DATA;
//	CAN_TxMessage[2] = LEAK_POSITION;
//	UCAN_Convert_Float_to_Bytes(_can_data->Leak_Status, &CAN_TxMessage[3]);
//	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
//	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
//	
	//--------MX28 RUDDER--------//
	CAN_TxMessage[0] = ARM2_RUDDER;
	CAN_TxMessage[1] = STATUS_DATA;
	CAN_TxMessage[2] = MX28_PRESENT_POSITION;
	UCAN_Convert_Float_to_Bytes(_can_data->MX28_Status.Position, &CAN_TxMessage[3]);
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
	UDELAY_us(200);
	
	CAN_TxMessage[0] = ARM2_RUDDER;
	CAN_TxMessage[1] = STATUS_DATA;
	CAN_TxMessage[2] = MX28_PRESENT_SPEED;
	UCAN_Convert_Float_to_Bytes(_can_data->MX28_Status.Speed, &CAN_TxMessage[3]);
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
	UDELAY_us(200);
	
	CAN_TxMessage[0] = ARM2_RUDDER;
	CAN_TxMessage[1] = STATUS_DATA;
	CAN_TxMessage[2] = MX28_PRESENT_LOAD;
	UCAN_Convert_Float_to_Bytes(_can_data->MX28_Status.Load, &CAN_TxMessage[3]);
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
	UDELAY_us(200);
	
	CAN_TxMessage[0] = ARM2_RUDDER;
	CAN_TxMessage[1] = STATUS_DATA;
	CAN_TxMessage[2] = MX28_PRESENT_VOL;
	UCAN_Convert_Float_to_Bytes(_can_data->MX28_Status.Voltage, &CAN_TxMessage[3]);
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
	UDELAY_us(200);
	
	CAN_TxMessage[0] = ARM2_RUDDER;
	CAN_TxMessage[1] = STATUS_DATA;
	CAN_TxMessage[2] = MX28_PRESENT_TEMP;
	UCAN_Convert_Float_to_Bytes(_can_data->MX28_Status.Temperature, &CAN_TxMessage[3]);
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
  UDELAY_us(200);
	
		//--------KELLER PA3--------//
	CAN_TxMessage[0] = ARM2_PRESSURE;
	CAN_TxMessage[1] = STATUS_DATA;
	CAN_TxMessage[2] = DEPTH_DATA;
	UCAN_Convert_Float_to_Bytes(_can_data->KellerPA3_Status.Pressure, &CAN_TxMessage[3]);
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
	UDELAY_us(200);
	
	CAN_TxMessage[0] = ARM2_PRESSURE;
	CAN_TxMessage[1] = STATUS_DATA;
	CAN_TxMessage[2] = TEMP_DATA;
	UCAN_Convert_Float_to_Bytes(_can_data->KellerPA3_Status.Temperature, &CAN_TxMessage[3]);
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
	UDELAY_us(200);
	
	//--------ACK--------//
	CAN_TxMessage[0] = 'R';
	CAN_TxMessage[1] = 'E';
	CAN_TxMessage[2] = 'Q';
	CAN_TxMessage[3] = 'A';
	CAN_TxMessage[4] = 'C';
	CAN_TxMessage[5] = 'K';
	CAN_TxMessage[6] = 0x00;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(_IDCANBUS_ARM_2,8,CAN_TxMessage);
}

bool UCAN_NeedRespondData(void)
{
	if(_can_respond_count >= num_frames_revc4req)
	{
		_need_respond_data = true;
		_can_respond_count = 0;
	}
	return _need_respond_data;
}

void UCAN_AllowRespondData(FunctionalState NewState)
{
	_need_respond_data = (NewState == ENABLE)?true:false;
}

void UCAN_HeartBeat(void)
{
	CAN_TxMessage[0] = 'A';
	CAN_TxMessage[1] = 'R';
	CAN_TxMessage[2] = 'M';
	CAN_TxMessage[3] = '2';
	CAN_TxMessage[4] = 'R';
	CAN_TxMessage[5] = 'D';
	CAN_TxMessage[6] = 'Y';
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	
	UCAN_Transmit(_IDCANBUS_ARM_2, 8, CAN_TxMessage);
}

void UCAN_StartSendHeartBeat(FunctionalState NewState)
{
	if(NewState == ENABLE)
	{
		TIM_ITConfig(UCAN_TIM, TIM_IT_Update, ENABLE);
		TIM_Cmd(UCAN_TIM, ENABLE);
	}
	else
	{
		TIM_ITConfig(UCAN_TIM, TIM_IT_Update, DISABLE);
		TIM_Cmd(UCAN_TIM, DISABLE);
	}
}

bool UCAN_IsNeedCheckSystem(void)
{
	return _need_system_ready;
}

void UCAN_AllowRespondCheckSystem(FunctionalState NewState)
{
	_need_system_ready = (NewState == ENABLE)?true:false;
}
//test Jetson
//===================================
void UCAN_test_init()
{
	Thruster.Value = 0;
	Rudder.Value = 1800;
	Mass.Value = 0;
	Pistol.Value = 0;
	Altimeter.Value = 0;
	leak_position = 0;
}

void UCAN_test_end()
{
	CAN_TxMessage[0] = 'E'; //0x45
	CAN_TxMessage[1] = 'J'; //0x4A
	CAN_TxMessage[2] = 'S';	//0x53
	CAN_TxMessage[3] = 0;
	CAN_TxMessage[4] = 0;
	CAN_TxMessage[5] = 0;
	CAN_TxMessage[6] = 0;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage); //0x1E
	UCAN_Transmit(0x125, 8, CAN_TxMessage);
}

void UCAN_test_thruster()
{
	if(Thruster.Value == 100)
	{
		Thruster.Value = 0;
	}
	CAN_TxMessage[0] = 'R';
	CAN_TxMessage[1] = 'P';
	CAN_TxMessage[2] = 'V';	
	CAN_TxMessage[3] = Thruster.byte.a4;
	CAN_TxMessage[4] = Thruster.byte.a3;
	CAN_TxMessage[5] = Thruster.byte.a2;
	CAN_TxMessage[6] = Thruster.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(0x123, 8, CAN_TxMessage);
	Thruster.Value += 10;
}

void UCAN_test_rudder()
{
	if(Rudder.Value == 2600)
	{
		Rudder.Value = 1800;
	}
	CAN_TxMessage[0] = 'P';
	CAN_TxMessage[1] = 'R';
	CAN_TxMessage[2] = 0;	
	CAN_TxMessage[3] = Rudder.byte.a4;
	CAN_TxMessage[4] = Rudder.byte.a3;
	CAN_TxMessage[5] = Rudder.byte.a2;
	CAN_TxMessage[6] = Rudder.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(0x122, 8, CAN_TxMessage);
	Rudder.Value += 100;
}

void UCAN_test_mass()
{
	if(Mass.Value == 50)
	{
		Mass.Value = 0;
	}
	CAN_TxMessage[0] = 'P';
	CAN_TxMessage[1] = 'M';
	CAN_TxMessage[2] = 0;	
	CAN_TxMessage[3] = Mass.byte.a4;
	CAN_TxMessage[4] = Mass.byte.a3;
	CAN_TxMessage[5] = Mass.byte.a2;
	CAN_TxMessage[6] = Mass.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(0x121, 8, CAN_TxMessage);
	Mass.Value += 10;
}

void UCAN_test_pistol()
{
	if(Pistol.Value == 90)
	{
		Pistol.Value = 0;
	}
	CAN_TxMessage[0] = 'P';
	CAN_TxMessage[1] = 'P';
	CAN_TxMessage[2] = 0;	
	CAN_TxMessage[3] = Pistol.byte.a4;
	CAN_TxMessage[4] = Pistol.byte.a3;
	CAN_TxMessage[5] = Pistol.byte.a2;
	CAN_TxMessage[6] = Pistol.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(0x121, 8, CAN_TxMessage);
	Pistol.Value += 10;
}

void UCAN_test_altimeter()
{
	if(Altimeter.Value == 2)
	{
		Altimeter.Value = 0;
	}
	CAN_TxMessage[0] = 'R';
	CAN_TxMessage[1] = 'A';
	CAN_TxMessage[2] = 'M';	
	CAN_TxMessage[3] = Altimeter.byte.a4;
	CAN_TxMessage[4] = Altimeter.byte.a3;
	CAN_TxMessage[5] = Altimeter.byte.a2;
	CAN_TxMessage[6] = Altimeter.byte.a1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(0x121, 8, CAN_TxMessage);
	Altimeter.Value += 0.5;
}

void UCAN_test_leak()
{
	if(leak_position == 7)
	{
		leak_position = 0;
	}
	CAN_TxMessage[0] = 0;
	CAN_TxMessage[1] = 0;
	CAN_TxMessage[2] = 0;	
	CAN_TxMessage[3] = 0;
	CAN_TxMessage[4] = 0;
	CAN_TxMessage[5] = 0;
	CAN_TxMessage[6] = 0;
	CAN_TxMessage[leak_position] = 1;
	CAN_TxMessage[7] = UCAN_Checksum(CAN_TxMessage);
	UCAN_Transmit(0x121, 8, CAN_TxMessage);
	UCAN_Transmit(0x122, 8, CAN_TxMessage);
	leak_position ++;
}

/***********************************************************************************************************/
/*-------------------------------------- Peripherals Interrupt Handlers -----------------------------------*/
/***********************************************************************************************************/
void UCAN_CAN_IRQHandler(void)
{
	CAN_Receive(UCAN_CAN,CAN_FIFO0,&RxMessage);
	if((RxMessage.StdId == _IDCANBUS_ARM_2)&&(RxMessage.IDE == CAN_ID_STD)&&(RxMessage.DLC == 8)&&(UCAN_Checksum(RxMessage.Data) == RxMessage.Data[7]))
	{
				if((RxMessage.Data[0] == 'E') && (RxMessage.Data[1] == 'J') 
				&& (RxMessage.Data[2] == 'S') && (RxMessage.Data[3] == 0)
				&& (RxMessage.Data[4] == 0) 	&& (RxMessage.Data[5] == 0)
				&& (RxMessage.Data[6] == 0))
		{
			check = true;
			UIO_LEDORANGE_TOGGLE();
		}
	}
//	if((RxMessage.StdId == _IDCANBUS_ARM_2)&&(RxMessage.IDE == CAN_ID_STD)&&(RxMessage.DLC == 8)&&(UCAN_Checksum(RxMessage.Data) == RxMessage.Data[7]))
//	{
//		
//		if((RxMessage.Data[0] == 'A') && (RxMessage.Data[1] == 'R') 
//				&& (RxMessage.Data[2] == 'M') && (RxMessage.Data[3] == '2')
//				&& (RxMessage.Data[4] == 'C') && (RxMessage.Data[5] == 'H')
//				&& (RxMessage.Data[6] == 'E'))
//		{
//			_need_system_ready = true;
//		}
//		
//		switch(RxMessage.Data[1])
//		{
//			case READ_DATA:
//			{
//				if((RxMessage.Data[0] == ARM2_ALL_DATA) && (RxMessage.Data[6] == 0x0A))
//				{
//					_can_respond_count++;
//				}
//				break;
//			}
//			case WRITE_DATA:
//			{
//				switch(RxMessage.Data[0])
//				{					
////					case ARM2_POWER_INT:
////					{
////						if(RxMessage.Data[2] == INT_24V40AH)
////						{
////							UIO_Emergency24V40Ah(ENABLE);
////						}
////						break;
////					}
//					case ARM2_RUDDER:
//					{
//						if(RxMessage.Data[2] == MX28_GOAL_POSITION)
//						{
//							uint16_t tmp = (uint16_t)UCAN_Convert_Byte2Uint16(RxMessage.Data + 3);
//							UMX28_setGoalPosition(1, tmp);
//						}
//						if(RxMessage.Data[2] == MX28_MOVING_SPEED)
//						{
//							UMX28_setMovingSpeed(1, (uint16_t)UCAN_Convert_Byte2Uint16(RxMessage.Data + 3));
//						}
//						if(RxMessage.Data[2] == MX28_KP)
//						{
//							UMX28_setKp(1, RxMessage.Data[3]);
//						}
//						if(RxMessage.Data[2] == MX28_KI)
//						{
//							UMX28_setKi(1, RxMessage.Data[3]);
//						}
//						if(RxMessage.Data[2] == MX28_KD)
//						{
//							UMX28_setKd(1, RxMessage.Data[3]);
//						}
//						break;
//					}
//					default:
//						break;
//				}
//			}				
//			default:
//				break;
//		}
//	}
}	

void UCAN_TIM_IRQHandler(void)
{
	if(TIM_GetITStatus(UCAN_TIM, TIM_IT_Update) != RESET)
	{
		UCAN_HeartBeat();
		TIM_ClearITPendingBit(UCAN_TIM, TIM_IT_Update);
	}
}

void UCAN_TIM_IRQHandler_2(void)
{
		if (TIM_GetITStatus(UCAN_TIM_2, TIM_IT_Update) != RESET)
    {
			SBUS_read(&uart_buffer_u3,&channels[0],&scalechannels[0],&failSafe,&lostFrame);
			UIO_LEDORANGE_TOGGLE();
			if(scalechannels[4])
			{
				UIO_LEDORANGE_TOGGLE();
				if(scalechannels[2] != old_scalechannels[2])
				{
					if(scalechannels[2] > 10 || scalechannels[2] < -10)
					{
						Run_Thruster((float)scalechannels[2]);
					}
					else
					{
						Run_Thruster(0);
					}
					old_scalechannels[2] = scalechannels[2];
				}
				if(scalechannels[0] != old_scalechannels[0])
				{
					if(scalechannels[0] > 10 || scalechannels[0] < -10)
					{
						Run_Piston((float)scalechannels[0]);
					}
					else
					{
						Run_Piston(0);
					}
					old_scalechannels[0] = scalechannels[0];
				}
				if(scalechannels[1] != old_scalechannels[1])
				{
					if(scalechannels[1] > 10 || scalechannels[1] < -10)
					{
						Run_Shifter((float)scalechannels[1]);
					}
					else
					{
						Run_Shifter(0);
					}
					old_scalechannels[1] = scalechannels[1];
				}
				UMX28_setGoalPosition(254, (float)scalechannels[3]);
			}
//			else
//			{
//				//UCAN_StopMotor();
//				Run_Thruster(0);
//				Run_Piston(0);
//				Run_Shifter(0);
//				UMX28_setGoalPosition(254, 1800);
//			}
			TIM_ClearITPendingBit(UCAN_TIM_2, TIM_IT_Update);
		}
}

