/*
 * User_CAN.c
 *
 *  Created on: Apr 9, 2018
 *      Author: Chau Thanh Hai
 */
#include "my_pmcan.h"
#include "my_syscan.h"
#include "my_io.h"

		/** 
		* @brief  Pistol - Mass Shifter CAN Pin define 
		*/


			#define 	UPMCAN_TX_PIN  				   	GPIO_Pin_13			
			#define 	UPMCAN_TX_PORT 				  	GPIOB
			#define 	UPMCAN_TX_CLK							RCC_AHB1Periph_GPIOB			
			#define 	UPMCAN_TX_SOURCE				 	GPIO_PinSource13				
			#define		UPMCAN_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UPMCAN_RX_PIN  				   	GPIO_Pin_12
			#define 	UPMCAN_RX_PORT 				  	GPIOB
			#define 	UPMCAN_RX_CLK							RCC_AHB1Periph_GPIOB
			#define 	UPMCAN_RX_SOURCE				 	GPIO_PinSource12
			#define		UPMCAN_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		

			#define		UPMCAN_CAN								CAN2
			#define 	UPMCAN_CAN_CLK						RCC_APB1Periph_CAN2
			#define		UPMCAN_CAN_CLK_Cmd		  	RCC_APB1PeriphClockCmd
			#define		UPMCAN_CAN_AF							GPIO_AF_CAN2
			#define 	UPMCAN_CAN_IRQn						CAN2_RX1_IRQn
			
			#define		UPMCAN_CAN_IRQHandler			CAN2_RX1_IRQHandler
			
			#define 	UPMCAN_TIM  				      			TIM4
			#define 	UPMCAN_TIM_CLK				 	 				RCC_APB1Periph_TIM4
			#define 	UPMCAN_TIM_CLK_Cmd  						RCC_APB1PeriphClockCmd
			#define 	UPMCAN_TIM_IRQn   							TIM4_IRQn				
			#define		UPMCAN_TIM_PreemptionPriority		0x00
			#define		UPMCAN_TIM_SubPriority					0x01		
			
			#define 	UPMCAN_TIM_IRQHandler						TIM4_IRQHandler



/**
 * @}
 */
static int _IDCANBUS_ARM_1 = 0x121;
static int _IDCANBUS_ARM_2 = 0x122;
static int _IDCANBUS_THRUSTER = 0x123;
static int _IDCANBUS_PISTOL = 0x124;
static int _IDCANBUS_MASS_SHIFTER = 0x125;
static int _IDCANBUS_EPC = 0x126;

uint8_t UPMCAN_RxMsgArr[8];
uint8_t UPMCAN_TxMsgArr[8];
uint8_t _pmcan_count = 0;

float Pistol_Speed = 30;
float Mass_Speed = 30;
union Position
{
	float Value;
	
	struct
	{
		unsigned a1:8;
		unsigned a2:8;
		unsigned a3:8;
		unsigned a4:8;
	} byte;
}set_position;

CanRxMsg UPMCAN_RxMessage;

uint8_t Mass_max_position = 58;
uint8_t Mass_min_position = 0;
uint8_t Mass_mid_position = 29;
float Pistol_Actual_Position = 0;
float Mass_Actual_Position = 0;

//-------Internal Function-------//
void UPMCAN_Transmit(int _IDstd,int _length, uint8_t _data[]);

void UPMCAN_Configure(void)
{
	GPIO_InitTypeDef 				GPIO_InitStruct;
	CAN_InitTypeDef        	CAN_InitStruct;
	CAN_FilterInitTypeDef  	CAN_FilterInitStruct;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef				NVIC_InitStruct;
	
	UPMCAN_TX_CLK_Cmd(UPMCAN_TX_CLK,ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UPMCAN_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UPMCAN_TX_PORT,&GPIO_InitStruct);

	UPMCAN_RX_CLK_Cmd(UPMCAN_RX_CLK,ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UPMCAN_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UPMCAN_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UPMCAN_TX_PORT,UPMCAN_TX_SOURCE,UPMCAN_CAN_AF);
	GPIO_PinAFConfig(UPMCAN_RX_PORT,UPMCAN_RX_SOURCE,UPMCAN_CAN_AF);
	
	UPMCAN_CAN_CLK_Cmd(UPMCAN_CAN_CLK,ENABLE);

	/* CAN register init */
	CAN_DeInit(UPMCAN_CAN);

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
	CAN_Init(UPMCAN_CAN, &CAN_InitStruct);

	/* CAN filter init */
	CAN_FilterInitStruct.CAN_FilterNumber = 15;
	CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_16bit;
	CAN_FilterInitStruct.CAN_FilterIdHigh = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterIdLow = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;
	CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
//	CAN_SlaveStartBank(14);
	CAN_FilterInit(&CAN_FilterInitStruct);

//	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(UPMCAN_CAN, CAN_IT_FMP1, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UPMCAN_CAN_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
	
//	/* Timer_Interrupt for trasmitting can */
	UPMCAN_TIM_CLK_Cmd(UPMCAN_TIM_CLK,ENABLE);
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 72*100-1;
	TIM_InitStructure.TIM_Period = 100;
	TIM_TimeBaseInit(UPMCAN_TIM, &TIM_InitStructure);
	TIM_Cmd(UPMCAN_TIM, ENABLE);
//	TIM_ITConfig(UPMCAN_TIM, TIM_IT_Update, ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = UPMCAN_TIM_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UPMCAN_TIM_PreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UPMCAN_TIM_SubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void UPMCAN_Transmit(int _IDstd,int _length, uint8_t _data[])
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

  mailbox = CAN_Transmit(UPMCAN_CAN,&_TxMessage);

	//wait until CAN transmission is OK
	int32_t i = 0;
  while((status != CANTXOK) && (i != 0xFFFF))
  {
    status = CAN_TransmitStatus(UPMCAN_CAN,mailbox);
    i++;
  }
	
//	UDELAY_us(100);
	
}

uint8_t UPMCAN_Checksum(uint8_t *_data)
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

void UPMCAN_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out)
{
	union
	{
		float _value;
		uint8_t _byte[4];
	}_part;
	
	_part._value = _data_in;
	
	_data_out[3] = _part._byte[0];
	_data_out[2] = _part._byte[1];
	_data_out[1] = _part._byte[2];
	_data_out[0] = _part._byte[3];
}

void UPMCAN_Convert_Bytes_to_Float(uint8_t* _data_in, float *_data_out)
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
	
	*_data_out = _part._value;
}

//--------Mass Shifter--------//
void UPMCAN_Mass_Start(UPMCAN_ENABLE_DRIVER_TypeDef _state)
{
	UPMCAN_TxMsgArr[0] = 'C';
	UPMCAN_TxMsgArr[1] = 'A';
	UPMCAN_TxMsgArr[2] = 'N';

	UPMCAN_TxMsgArr[3] = _state;
	UPMCAN_TxMsgArr[4] = 0x00;
	UPMCAN_TxMsgArr[5] = 0x00;
	UPMCAN_TxMsgArr[6] = 0x00;
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetOLoop_Duty(float _duty, UPMCAN_Direction_of_Motor_TypeDef _direction)
{
	UPMCAN_TxMsgArr[0] = 'O';
	UPMCAN_TxMsgArr[1] = 'L';
	UPMCAN_TxMsgArr[2] = _direction;
	UPMCAN_Convert_Float_to_Bytes(_duty, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetPID_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction)
{
	UPMCAN_TxMsgArr[0] = 'O';
	UPMCAN_TxMsgArr[1] = 'P';
	UPMCAN_TxMsgArr[2] = _direction;
	UPMCAN_Convert_Float_to_Bytes(_speed, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetPID_Kp(float _kp)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'K';
	UPMCAN_TxMsgArr[2] = 'P';
	UPMCAN_Convert_Float_to_Bytes(_kp, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetPID_Ki(float _ki)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'K';
	UPMCAN_TxMsgArr[2] = 'I';
	UPMCAN_Convert_Float_to_Bytes(_ki, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetPID_Kd(float _kd)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'K';
	UPMCAN_TxMsgArr[2] = 'D';
	UPMCAN_Convert_Float_to_Bytes(_kd, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetFuzzy_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction)
{
	UPMCAN_TxMsgArr[0] = 'O';
	UPMCAN_TxMsgArr[1] = 'F';
	UPMCAN_TxMsgArr[2] = _direction;
	UPMCAN_Convert_Float_to_Bytes(_speed, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetFuzzy_Ge(float _ge)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'G';
	UPMCAN_TxMsgArr[2] = 'E';
	UPMCAN_Convert_Float_to_Bytes(_ge, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetFuzzy_Gde(float _gde)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'D';
	UPMCAN_TxMsgArr[2] = 'E';
	UPMCAN_Convert_Float_to_Bytes(_gde, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_SetFuzzy_Gdu(float _gdu)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'D';
	UPMCAN_TxMsgArr[2] = 'U';
	UPMCAN_Convert_Float_to_Bytes(_gdu, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Mass_GetStatus(Motor_StatusTypeDef _motor_status)
{
	UPMCAN_TxMsgArr[0] = 'R';
	UPMCAN_TxMsgArr[1] = 'E';
	UPMCAN_TxMsgArr[2] = 'Q';
	UPMCAN_TxMsgArr[3] = 'A';
	UPMCAN_TxMsgArr[4] = 'L';
	UPMCAN_TxMsgArr[5] = 'L';
	UPMCAN_TxMsgArr[6] = 0x0A;
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
}

//void UPMCAN_Mass_Position(float pos)
//{
//	set_position.Value = pos;
//	if(set_position.Value >= Mass_Actual_Position)
//	{
//		Flag.Mass_Run_CW = true;
//	}
//	if(set_position.Value < Mass_Actual_Position)
//	{
//		Flag.Mass_Run_CCW = true;
//	}
//	UPMCAN_TxMsgArr[0] = 'O';
//	UPMCAN_TxMsgArr[1] = 'L';
//	UPMCAN_TxMsgArr[2] = 'S';
//	UPMCAN_TxMsgArr[3] = set_position.byte.a4;
//	UPMCAN_TxMsgArr[4] = set_position.byte.a3;
//	UPMCAN_TxMsgArr[5] = set_position.byte.a2;
//	UPMCAN_TxMsgArr[6] = set_position.byte.a1;
//	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
//	
//	UPMCAN_Transmit(_IDCANBUS_MASS_SHIFTER, 8, UPMCAN_TxMsgArr);
//}

//--------Pistol--------//
void UPMCAN_Pistol_Start(UPMCAN_ENABLE_DRIVER_TypeDef _state)
{
	UPMCAN_TxMsgArr[0] = 'C';
	UPMCAN_TxMsgArr[1] = 'A';
	UPMCAN_TxMsgArr[2] = 'N';

	UPMCAN_TxMsgArr[3] = _state;
	UPMCAN_TxMsgArr[4] = 0x00;
	UPMCAN_TxMsgArr[5] = 0x00;
	UPMCAN_TxMsgArr[6] = 0x00;
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetOLoop_Duty(float _duty, UPMCAN_Direction_of_Motor_TypeDef _direction)
{
	UPMCAN_TxMsgArr[0] = 'O';
	UPMCAN_TxMsgArr[1] = 'L';
	UPMCAN_TxMsgArr[2] = _direction;
	UPMCAN_Convert_Float_to_Bytes(_duty, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetPID_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction)
{
	UPMCAN_TxMsgArr[0] = 'C';
	UPMCAN_TxMsgArr[1] = 'P';
	UPMCAN_TxMsgArr[2] = _direction;
	UPMCAN_Convert_Float_to_Bytes(_speed, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetPID_DesPosition(float _position){
	UPMCAN_TxMsgArr[0] = 'C';
	UPMCAN_TxMsgArr[1] = 'P';
	UPMCAN_TxMsgArr[2] = 'P';
	UPMCAN_Convert_Float_to_Bytes(_position, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetPID_Kp(float _kp)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'K';
	UPMCAN_TxMsgArr[2] = 'P';
	UPMCAN_Convert_Float_to_Bytes(_kp, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetPID_Ki(float _ki)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'K';
	UPMCAN_TxMsgArr[2] = 'I';
	UPMCAN_Convert_Float_to_Bytes(_ki, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetPID_Kd(float _kd)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'K';
	UPMCAN_TxMsgArr[2] = 'D';
	UPMCAN_Convert_Float_to_Bytes(_kd, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetFuzzy_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction)
{
	UPMCAN_TxMsgArr[0] = 'C';
	UPMCAN_TxMsgArr[1] = 'F';
	UPMCAN_TxMsgArr[2] = _direction;
	UPMCAN_Convert_Float_to_Bytes(_speed, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetFuzzy_DesPosition(float _position){
	UPMCAN_TxMsgArr[0] = 'C';
	UPMCAN_TxMsgArr[1] = 'F';
	UPMCAN_TxMsgArr[2] = 'P';
	UPMCAN_Convert_Float_to_Bytes(_position, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetFuzzy_Ge(float _ge)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'G';
	UPMCAN_TxMsgArr[2] = 'E';
	UPMCAN_Convert_Float_to_Bytes(_ge, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetFuzzy_Gde(float _gde)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'D';
	UPMCAN_TxMsgArr[2] = 'E';
	UPMCAN_Convert_Float_to_Bytes(_gde, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_SetFuzzy_Gdu(float _gdu)
{
	UPMCAN_TxMsgArr[0] = 'G';
	UPMCAN_TxMsgArr[1] = 'D';
	UPMCAN_TxMsgArr[2] = 'U';
	UPMCAN_Convert_Float_to_Bytes(_gdu, &UPMCAN_TxMsgArr[3]);
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Pistol_GetStatus(Motor_StatusTypeDef _motor_status)
{
	UPMCAN_TxMsgArr[0] = 'R';
	UPMCAN_TxMsgArr[1] = 'E';
	UPMCAN_TxMsgArr[2] = 'Q';
	UPMCAN_TxMsgArr[3] = 'A';
	UPMCAN_TxMsgArr[4] = 'L';
	UPMCAN_TxMsgArr[5] = 'L';
	UPMCAN_TxMsgArr[6] = 0x0A;
	UPMCAN_TxMsgArr[7] = UPMCAN_Checksum(UPMCAN_TxMsgArr);
	
	UPMCAN_Transmit(_IDCANBUS_PISTOL, 8, UPMCAN_TxMsgArr);
}

void UPMCAN_Initialize_Position(void)
{
	//Move Mass Shifter
	if(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4))
	{
		UPMCAN_Mass_SetOLoop_Duty(30,MOTOR_CCW);
		while(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4));
		for(uint8_t i = 0; i < 3; i++)
		{
			UPMCAN_Mass_SetOLoop_Duty(0,MOTOR_CCW);
		}
		UDELAY_ms(10);
	}
	
	//Move Pistol
	UPMCAN_Pistol_SetOLoop_Duty(30,MOTOR_CW);
	while(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3));
	for(uint8_t i = 0; i<3; i++)
	{
		UPMCAN_Pistol_SetOLoop_Duty(0,MOTOR_CW);
	}
	UDELAY_ms(10);
	UPMCAN_Pistol_SetOLoop_Duty(30,MOTOR_CCW);
	while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3));
	for(uint8_t i = 0; i<3; i++)
	{
		UPMCAN_Pistol_SetOLoop_Duty(0,MOTOR_CCW);
	}
	
	//Mass Shifter to mid
	UPMCAN_Mass_SetOLoop_Duty(30,MOTOR_CW);
	while(Mass_Actual_Position < Mass_mid_position);
	UPMCAN_Mass_SetOLoop_Duty(0,MOTOR_CW);
	UDELAY_ms(10);
	
	USYSCAN_SystemReady();
	USYSCAN_OpenThruster();
	Flag.Done_setup = true;
	TIM_ITConfig(UPMCAN_TIM, TIM_IT_Update, ENABLE);
}

void UPMCAN_RunMotor(void)
{
//	if(Flag.Piston_open)
//	{
//		UPMCAN_Pistol_Start(ENABLE_DRIVER);
//		Flag.Piston_open = false;
//	}
//	if(Flag.Shifter_open)
//	{
//		UPMCAN_Mass_Start(ENABLE_DRIVER);
//		Flag.Shifter_open = false;
//	}
	
// Mass Shifter	
	if(Flag.Mass_Position)
	{
		if(UPWM_Position_Mass >= Mass_Actual_Position + 1 && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5) == 0 && !Flag.Mass_H && !Flag.Mass_T)
		{
			UPMCAN_Mass_SetOLoop_Duty(Mass_Speed,MOTOR_CW);
			Flag.Mass_Run_CW = true;
		}
		else if(UPWM_Position_Mass <= Mass_Actual_Position - 1 && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4) == 0 && !Flag.Mass_H && !Flag.Mass_T)
		{
			UPMCAN_Mass_SetOLoop_Duty(Mass_Speed,MOTOR_CCW);
			Flag.Mass_Run_CCW = true;
		}
		Flag.Mass_Position = false;
	}
	if(Flag.Mass_Run_CW && (Mass_Actual_Position >= UPWM_Position_Mass && !Flag.Mass_H && !Flag.Mass_T))
	{
		UPMCAN_Mass_SetOLoop_Duty(0,MOTOR_CW);
		Flag.Mass_Run_CW = false;
	}
	if(Flag.Mass_Run_CCW && (Mass_Actual_Position <= UPWM_Position_Mass && !Flag.Mass_H && !Flag.Mass_T))
	{
		UPMCAN_Mass_SetOLoop_Duty(0,MOTOR_CCW);
		Flag.Mass_Run_CCW = false;
	}
	if((Flag.Mass_T && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4) == 0) || (Flag.Mass_H && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5) == 0))
	{
		for(uint8_t i = 0; i < 3; i++)
		{
			UPMCAN_Mass_SetOLoop_Duty(0,MOTOR_CW);
		}
		Flag.Mass_T = false;
		Flag.Mass_H = false;
		Flag.Mass_Run_CW = false;
		Flag.Mass_Run_CCW = false;
	}
	
// Pistol
	if((Flag.Pis_T && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) == 0) || (Flag.Pis_H && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) == 0))
	{
		for(uint8_t i = 0; i < 3; i++)
		{
			UPMCAN_Pistol_SetOLoop_Duty(0,MOTOR_CW);
		}
		Flag.Pis_T = false;
		Flag.Pis_H = false;
		Flag.Pistol_Run_CW = false;
		Flag.Pistol_Run_CCW = false;
	}
// Pistol Jetson Control
	if(Flag.Pistol_Position)
	{
		if(UPWM_Position_Pistol >= Pistol_Actual_Position + 1 && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) == 0 && !Flag.Pis_H && !Flag.Pis_T)
		{
			UPMCAN_Pistol_SetOLoop_Duty(Pistol_Speed,MOTOR_CCW);
			Flag.Pistol_Run_CCW = true;
		}
		else if(UPWM_Position_Pistol <= Pistol_Actual_Position - 1 && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) == 0 && !Flag.Pis_H && !Flag.Pis_T)
		{
			UPMCAN_Pistol_SetOLoop_Duty(Pistol_Speed,MOTOR_CW);
			Flag.Pistol_Run_CW = true;
		}
		Flag.Pistol_Position = false;
	}
	if(Flag.Pistol_Run_CW && (Pistol_Actual_Position <= UPWM_Position_Pistol && !Flag.Pis_H && !Flag.Pis_T))
	{
		UPMCAN_Pistol_SetOLoop_Duty(0,MOTOR_CW);
		Flag.Pistol_Run_CW = false;
	}
	if(Flag.Pistol_Run_CCW && (Pistol_Actual_Position >= UPWM_Position_Pistol && !Flag.Pis_H && !Flag.Pis_T))
	{
		UPMCAN_Pistol_SetOLoop_Duty(0,MOTOR_CCW);
		Flag.Pistol_Run_CCW = false;
	}
// Pistol Joystikck Control
	if(Flag.Pistol_Run_CCW_Joystick && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) == 0 && !Flag.Pis_H && !Flag.Pis_T)
	{
		UPMCAN_Pistol_SetOLoop_Duty(UPWM_DutyCycle_Pistol,MOTOR_CCW);
		Flag.Pistol_Run_CCW_Joystick = false;
	}
	if(Flag.Pistol_Run_CW_Joystick && GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) == 0 && !Flag.Pis_H && !Flag.Pis_T)
	{
		UPMCAN_Pistol_SetOLoop_Duty(UPWM_DutyCycle_Pistol,MOTOR_CW);
		Flag.Pistol_Run_CW_Joystick = false;
	}
	
// Send position data
//	if(Flag.Send_Data && (Flag.End_Frame_ARM2 || Flag.End_Frame_Jetson))
//	{
//		USYSCAN_Send_Data();
//		Flag.End_Frame_ARM2 = false;
//		Flag.End_Frame_Jetson = false;
//	}
	if(Flag.Send_Data && Flag.End_Frame_Jetson)
	{
		USYSCAN_Send_Data();
		Flag.End_Frame_ARM2 = false;
		Flag.End_Frame_Jetson = false;
	}
}
//=======================================

/***********************************************************************************************************/
/*-------------------------------------- Peripherals Interrupt Handlers -----------------------------------*/
/***********************************************************************************************************/
void UPMCAN_CAN_IRQHandler(void)
{
	CAN_Receive(UPMCAN_CAN,CAN_FIFO1,&UPMCAN_RxMessage);
	if(CAN_MessagePending(UPMCAN_CAN,CAN_FIFO1) >= 2)
	{
		CAN_FIFORelease(UPMCAN_CAN,CAN_FIFO1);
	}
	if((UPMCAN_RxMessage.StdId == _IDCANBUS_ARM_1)&&(UPMCAN_RxMessage.IDE == CAN_ID_STD)
		&&(UPMCAN_RxMessage.DLC == 8)&&(UPMCAN_Checksum(UPMCAN_RxMessage.Data) == UPMCAN_RxMessage.Data[7]))
	{
			if((UPMCAN_RxMessage.Data[0] == 'P')&&(UPMCAN_RxMessage.Data[1] == 'M')&&(UPMCAN_RxMessage.Data[2] == 'A'))
			{
				UPMCAN_Convert_Bytes_to_Float(&UPMCAN_RxMessage.Data[3],&Mass_Actual_Position);
				i1 = Mass_Actual_Position;
			}
			if((UPMCAN_RxMessage.Data[0] == 'P')&&(UPMCAN_RxMessage.Data[1] == 'P')&&(UPMCAN_RxMessage.Data[2] == 'I'))
			{
				UPMCAN_Convert_Bytes_to_Float(&UPMCAN_RxMessage.Data[3],&Pistol_Actual_Position);
				i2 = Pistol_Actual_Position;
			}
	}
//			_pmcan_count ++;
//			if(_pmcan_count == 10)
//			{
//				UIO_LEDORANGE_TOGGLE();
//				_pmcan_count = 0;
//			}

//			_pmcan_count ++;
//			if(_pmcan_count == 10)
//			{
//				UIO_LEDORANGE_TOGGLE();
//				_pmcan_count = 0;
//			}
//			if((UPMCAN_RxMessage.Data[0] == 'O')&&(UPMCAN_RxMessage.Data[1] == 'P')&&(UPMCAN_RxMessage.Data[2] == 'L'))
//			{
//					UPWM_DutyCycle1.byte.a1 = UPMCAN_RxMessage.Data[6];
//					UPWM_DutyCycle1.byte.a2 = UPMCAN_RxMessage.Data[5];
//					UPWM_DutyCycle1.byte.a3 = UPMCAN_RxMessage.Data[4];
//					UPWM_DutyCycle1.byte.a4 = UPMCAN_RxMessage.Data[3];
//					Flag.Piston_Run_CCW = true;
//			}
//			if((UPMCAN_RxMessage.Data[0] == 'O')&&(UPMCAN_RxMessage.Data[1] == 'P')&&(UPMCAN_RxMessage.Data[2] == 'R'))
//			{
//					UPWM_DutyCycle1.byte.a1 = UPMCAN_RxMessage.Data[6];
//					UPWM_DutyCycle1.byte.a2 = UPMCAN_RxMessage.Data[5];
//					UPWM_DutyCycle1.byte.a3 = UPMCAN_RxMessage.Data[4];
//					UPWM_DutyCycle1.byte.a4 = UPMCAN_RxMessage.Data[3];
//					Flag.Piston_Run_CW = true;
//			}
//			if((UPMCAN_RxMessage.Data[0] == 'C')&&(UPMCAN_RxMessage.Data[1] == 'A')&&(UPMCAN_RxMessage.Data[2] == 'N')
//			 &&(UPMCAN_RxMessage.Data[3] == 'O')&&(UPMCAN_RxMessage.Data[4] == 'S'))
//			{
//					Flag.Shifter_open = true;
//			}
//			if((UPMCAN_RxMessage.Data[0] == 'O')&&(UPMCAN_RxMessage.Data[1] == 'S')&&(UPMCAN_RxMessage.Data[2] == 'L'))
//			{
//					UPWM_DutyCycle2.byte.a1 = UPMCAN_RxMessage.Data[6];
//					UPWM_DutyCycle2.byte.a2 = UPMCAN_RxMessage.Data[5];
//					UPWM_DutyCycle2.byte.a3 = UPMCAN_RxMessage.Data[4];
//					UPWM_DutyCycle2.byte.a4 = UPMCAN_RxMessage.Data[3];
//					Flag.Shifter_Run_CCW = true;
//			}
//			if((UPMCAN_RxMessage.Data[0] == 'O')&&(UPMCAN_RxMessage.Data[1] == 'S')&&(UPMCAN_RxMessage.Data[2] == 'R'))
//			{
//					UPWM_DutyCycle2.byte.a1 = UPMCAN_RxMessage.Data[6];
//					UPWM_DutyCycle2.byte.a2 = UPMCAN_RxMessage.Data[5];
//					UPWM_DutyCycle2.byte.a3 = UPMCAN_RxMessage.Data[4];
//					UPWM_DutyCycle2.byte.a4 = UPMCAN_RxMessage.Data[3];
//					Flag.Shifter_Run_CW = true;
//			}
			
//	}
}

void UPMCAN_TIM_IRQHandler(void)
{
	if (TIM_GetITStatus(UPMCAN_TIM, TIM_IT_Update) != RESET)
  {
		UPMCAN_RunMotor();
	}
	TIM_ClearITPendingBit(UPMCAN_TIM, TIM_IT_Update);
}
