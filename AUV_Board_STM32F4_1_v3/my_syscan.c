/*
 * User_CAN.c
 *
 *  Created on: Apr 9, 2018
 *      Author: Chau Thanh Hai
 */
#include "my_syscan.h"

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   CAN Pin define 
		*/
			#define 	USYSCAN_TX_PIN  				   	GPIO_Pin_9			
			#define 	USYSCAN_TX_PORT 				  	GPIOB
			#define 	USYSCAN_TX_CLK							RCC_AHB1Periph_GPIOB			
			#define 	USYSCAN_TX_SOURCE				 		GPIO_PinSource9				
			#define		USYSCAN_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	USYSCAN_RX_PIN  				   	GPIO_Pin_8
			#define 	USYSCAN_RX_PORT 				  	GPIOB
			#define 	USYSCAN_RX_CLK							RCC_AHB1Periph_GPIOB
			#define 	USYSCAN_RX_SOURCE				 		GPIO_PinSource8
			#define		USYSCAN_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		

			#define		USYSCAN_CAN									CAN1
			#define 	USYSCAN_CAN_CLK							RCC_APB1Periph_CAN1
			#define		USYSCAN_CAN_CLK_Cmd		  		RCC_APB1PeriphClockCmd
			#define		USYSCAN_CAN_AF							GPIO_AF_CAN1
			#define 	USYSCAN_CAN_IRQn						CAN1_RX0_IRQn
			
			#define		USYSCAN_CAN_IRQHandler			CAN1_RX0_IRQHandler
			
			#define 	UCAN_TIM_2  				      			TIM3
			#define 	UCAN_TIM_CLK_2				 	 				RCC_APB1Periph_TIM3
			#define 	UCAN_TIM_CLK_Cmd_2   						RCC_APB1PeriphClockCmd
			#define 	UCAN_TIM_IRQn_2    							TIM3_IRQn				
			#define		UCAN_TIM_PreemptionPriority_2		0x00
			#define		UCAN_TIM_SubPriority_2					0x00		
			
			#define 	UCAN_TIM_IRQHandler_2						TIM3_IRQHandler
			
//#define MODULE_CAN_MCP251x 

#ifdef MODULE_CAN_MCP251x
	static const uint8_t num_frames_revc4req = 2;
#else
	static const uint8_t num_frames_revc4req = 1;
#endif

static bool _need_respond_data = false;
static bool _system_ready = false;
static bool _need_system_ready = false;
uint8_t _can_respond_count = 0;
uint8_t _syscan_count = 0;
uint8_t _send_status = 0;

float UPWM_DutyCycle_Pistol;
float UPWM_DutyCycle_Mass;
float UPWM_Position_Pistol;
float UPWM_Position_Mass;

struct Status_flag Flag;

/**
 * @}
 */
static int _IDCANBUS_ARM_1 = 0x121;
static int _IDCANBUS_ARM_2 = 0x122;
static int _IDCANBUS_THRUSTER = 0x123;
static int _IDCANBUS_PISTOL = 0x124;
static int _IDCANBUS_MASS_SHIFTER = 0x125;
static int _IDCANBUS_EPC = 0x126;

uint8_t USYSCAN_RxMsgArr[8];
uint8_t USYSCAN_TxMsgArr[8];
float i1 = 0,i2 = 0;
float z = 0;

CanRxMsg USYSCAN_RxMessage;

//Declare function//
void USYSCAN_Piston_open(void);
void USYSCAN_Piston_Run_CW(void);
void USYSCAN_Piston_Run_CCW(void);
void USYSCAN_Shifter_open(void);
void USYSCAN_Shifter_Run_CW(void);
void USYSCAN_Shifter_Run_CCW(void);
void USYSCAN_RunMotor(void);

void USYSCAN_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	CAN_InitTypeDef        CAN_InitStruct;
	CAN_FilterInitTypeDef  CAN_FilterInitStruct;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStruct;
	
	USYSCAN_TX_CLK_Cmd(USYSCAN_TX_CLK,ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = USYSCAN_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(USYSCAN_TX_PORT,&GPIO_InitStruct);

	USYSCAN_RX_CLK_Cmd(USYSCAN_RX_CLK,ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = USYSCAN_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(USYSCAN_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(USYSCAN_TX_PORT,USYSCAN_TX_SOURCE,USYSCAN_CAN_AF);
	GPIO_PinAFConfig(USYSCAN_RX_PORT,USYSCAN_RX_SOURCE,USYSCAN_CAN_AF);
	
	USYSCAN_CAN_CLK_Cmd(USYSCAN_CAN_CLK,ENABLE);

	/* CAN register init */
	CAN_DeInit(USYSCAN_CAN);

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
	CAN_Init(USYSCAN_CAN, &CAN_InitStruct);

	/* CAN filter init */
	CAN_FilterInitStruct.CAN_FilterNumber = 0;
	CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_16bit;
	CAN_FilterInitStruct.CAN_FilterIdHigh = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterIdLow = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow = _IDCANBUS_ARM_1 << 5;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
//	CAN_SlaveStartBank(14);
	CAN_FilterInit(&CAN_FilterInitStruct);

//	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(USYSCAN_CAN, CAN_IT_FMP0, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
	
//  /*Timer_interrupt for transmitter can */
//	UCAN_TIM_CLK_Cmd_2(UCAN_TIM_CLK_2,ENABLE);
//	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_InitStructure.TIM_Prescaler = 72*100-1;
//	TIM_InitStructure.TIM_Period = 100;
//	TIM_TimeBaseInit(UCAN_TIM_2, &TIM_InitStructure);
//	TIM_Cmd(UCAN_TIM_2, ENABLE);
//	TIM_ITConfig(UCAN_TIM_2, TIM_IT_Update, ENABLE);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = UCAN_TIM_IRQn_2;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UCAN_TIM_PreemptionPriority_2;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UCAN_TIM_SubPriority_2;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
}

void USYSCAN_Transmit(int _IDstd,int _length, uint8_t _data[])
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

  mailbox = CAN_Transmit(USYSCAN_CAN,&_TxMessage);

	//wait until CAN transmission is OK
	int32_t i = 0;
  while((status != CANTXOK) && (i != 0xFFFF))
  {
    status = CAN_TransmitStatus(USYSCAN_CAN,mailbox);
    i++;
  }
	
//	UDELAY_us(100);
	
}

uint8_t USYSCAN_Checksum(uint8_t *_data)
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

void USYSCAN_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out)
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

void USYSCAN_Convert_Bytes_to_Float(uint8_t* _data_in, float *_data_out)
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

void USYSCAN_SystemReady(void)
{
	USYSCAN_TxMsgArr[0] = 'R';
	USYSCAN_TxMsgArr[1] = 'D';
	USYSCAN_TxMsgArr[2] = 'Y';
	USYSCAN_TxMsgArr[3] = 0;
	USYSCAN_TxMsgArr[4] = 0;
	USYSCAN_TxMsgArr[5] = 0;
	USYSCAN_TxMsgArr[6] = 0;
	USYSCAN_TxMsgArr[7] = USYSCAN_Checksum(USYSCAN_TxMsgArr);
	
	USYSCAN_Transmit(_IDCANBUS_EPC, 8, USYSCAN_TxMsgArr);
}

void USYSCAN_OpenThruster(void)
{
	USYSCAN_TxMsgArr[0] = 'C';
	USYSCAN_TxMsgArr[1] = 'A';
	USYSCAN_TxMsgArr[2] = 'N';
	USYSCAN_TxMsgArr[3] = 'O';
	USYSCAN_TxMsgArr[4] = 0;
	USYSCAN_TxMsgArr[5] = 0;
	USYSCAN_TxMsgArr[6] = 0;
	USYSCAN_TxMsgArr[7] = USYSCAN_Checksum(USYSCAN_TxMsgArr);
	
	USYSCAN_Transmit(_IDCANBUS_THRUSTER, 8, USYSCAN_TxMsgArr);
}

void USYSCAN_SystemNotReady(void)
{
	USYSCAN_TxMsgArr[0] = 'A';
	USYSCAN_TxMsgArr[1] = 'R';
	USYSCAN_TxMsgArr[2] = 'M';
	USYSCAN_TxMsgArr[3] = '1';
	USYSCAN_TxMsgArr[4] = 'N';
	USYSCAN_TxMsgArr[5] = 'O';
	USYSCAN_TxMsgArr[6] = 'T';
	USYSCAN_TxMsgArr[7] = USYSCAN_Checksum(USYSCAN_TxMsgArr);
	
	USYSCAN_Transmit(_IDCANBUS_ARM_1, 8, USYSCAN_TxMsgArr);
}

void USYSCAN_Send_Data(void)
{
	switch(_send_status)
	{
		case 0:
			//Send Leak Position
			USYSCAN_TxMsgArr[0] = LEAK_POSITION_1;
			USYSCAN_TxMsgArr[1] = LEAK_POSITION_2;
			USYSCAN_TxMsgArr[2] = LEAK_POSITION_3;
			USYSCAN_TxMsgArr[3] = LEAK_POSITION_4;
			USYSCAN_TxMsgArr[4] = LEAK_POSITION_5;
			USYSCAN_TxMsgArr[5] = 0;
			USYSCAN_TxMsgArr[6] = 0;
			USYSCAN_TxMsgArr[7] = USYSCAN_Checksum(USYSCAN_TxMsgArr);
			USYSCAN_Transmit(_IDCANBUS_ARM_1, 8, USYSCAN_TxMsgArr);
			_send_status ++;
		break;
	
		case 1:
			//Send Mass Position
			USYSCAN_TxMsgArr[0] = 'P';
			USYSCAN_TxMsgArr[1] = 'M';
			USYSCAN_TxMsgArr[2] = 0;
			USYSCAN_Convert_Float_to_Bytes(Mass_Actual_Position, &USYSCAN_TxMsgArr[3]);
			USYSCAN_TxMsgArr[7] = USYSCAN_Checksum(USYSCAN_TxMsgArr);
			USYSCAN_Transmit(_IDCANBUS_ARM_1, 8, USYSCAN_TxMsgArr);
			_send_status ++;
		break;	
	
		case 2:
			//Send Pistol Position
			USYSCAN_TxMsgArr[0] = 'P';
			USYSCAN_TxMsgArr[1] = 'P';
			USYSCAN_TxMsgArr[2] = 0;
			USYSCAN_Convert_Float_to_Bytes(Pistol_Actual_Position, &USYSCAN_TxMsgArr[3]);
			USYSCAN_TxMsgArr[7] = USYSCAN_Checksum(USYSCAN_TxMsgArr);
			USYSCAN_Transmit(_IDCANBUS_ARM_1, 8, USYSCAN_TxMsgArr);
			_send_status ++;
		break;
	
		case 3:
			//Send Altimeter range in meters
			USYSCAN_TxMsgArr[0] = 'R';
			USYSCAN_TxMsgArr[1] = 'A';
			USYSCAN_TxMsgArr[2] = 'M';
			USYSCAN_Convert_Float_to_Bytes(UALTI_in_metres.Value, &USYSCAN_TxMsgArr[3]);
			USYSCAN_TxMsgArr[7] = USYSCAN_Checksum(USYSCAN_TxMsgArr);
			USYSCAN_Transmit(_IDCANBUS_ARM_1, 8, USYSCAN_TxMsgArr);
			_send_status = 0;
		break;
	}
}

bool USYSCAN_IsSystemReady(void)
{
	return _system_ready;
}

void USYSCAN_PassSystemReady(FunctionalState NewState)
{
	_system_ready = (NewState == ENABLE)?true:false;
}

bool USYSCAN_IsNeedCheckSystem(void)
{
	return _need_system_ready;
}

void USYSCAN_AllowRespondCheckSystem(FunctionalState NewState)
{
	_need_system_ready = (NewState == ENABLE)?true:false;
}

void USYSCAN_AllowRespondData(FunctionalState NewState)
{
	_need_respond_data = (NewState == ENABLE)?true:false;
}

/***********************************************************************************************************/
/*-------------------------------------- Peripherals Interrupt Handlers -----------------------------------*/
/***********************************************************************************************************/
void USYSCAN_CAN_IRQHandler(void)
{
	CAN_Receive(USYSCAN_CAN,CAN_FIFO0,&USYSCAN_RxMessage);
	if(CAN_MessagePending(USYSCAN_CAN,CAN_FIFO0) >= 2)
	{
		CAN_FIFORelease(USYSCAN_CAN,CAN_FIFO0);
	}
	//--------Request ALL Data--------//
	if((USYSCAN_RxMessage.StdId == _IDCANBUS_ARM_1)&&(USYSCAN_RxMessage.IDE == CAN_ID_STD)
		&&(USYSCAN_RxMessage.DLC == 8)&&(USYSCAN_Checksum(USYSCAN_RxMessage.Data) == USYSCAN_RxMessage.Data[7]))
	{
			if((USYSCAN_RxMessage.Data[0] == 'C')&&(USYSCAN_RxMessage.Data[1] == 'A')&&(USYSCAN_RxMessage.Data[2] == 'N')
			 &&(USYSCAN_RxMessage.Data[3] == 'O')&&(USYSCAN_RxMessage.Data[4] == 'P'))
			{
					Flag.Pistol_open = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'O')&&(USYSCAN_RxMessage.Data[1] == 'P')&&(USYSCAN_RxMessage.Data[2] == 'R'))
			{
					USYSCAN_Convert_Bytes_to_Float(&USYSCAN_RxMessage.Data[3],&UPWM_DutyCycle_Pistol);
					Flag.Pistol_Run_CW_Joystick = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'O')&&(USYSCAN_RxMessage.Data[1] == 'P')&&(USYSCAN_RxMessage.Data[2] == 'L'))
			{
					USYSCAN_Convert_Bytes_to_Float(&USYSCAN_RxMessage.Data[3],&UPWM_DutyCycle_Pistol);
					Flag.Pistol_Run_CCW_Joystick = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'O')&&(USYSCAN_RxMessage.Data[1] == 'L')&&(USYSCAN_RxMessage.Data[2] == 'P'))
			{
					USYSCAN_Convert_Bytes_to_Float(&USYSCAN_RxMessage.Data[3],&UPWM_Position_Pistol);
					Flag.Pistol_Position = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'C')&&(USYSCAN_RxMessage.Data[1] == 'A')&&(USYSCAN_RxMessage.Data[2] == 'N')
			 &&(USYSCAN_RxMessage.Data[3] == 'O')&&(USYSCAN_RxMessage.Data[4] == 'S'))
			{
					Flag.Mass_open = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'O')&&(USYSCAN_RxMessage.Data[1] == 'L')&&(USYSCAN_RxMessage.Data[2] == 'M'))
			{
					USYSCAN_Convert_Bytes_to_Float(&USYSCAN_RxMessage.Data[3],&UPWM_Position_Mass);
					Flag.Mass_Position = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'O')&&(USYSCAN_RxMessage.Data[1] == 'S')&&(USYSCAN_RxMessage.Data[2] == 'M'))
			{
					Flag.Mass_Stop = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'C')&&(USYSCAN_RxMessage.Data[1] == 'A')&&(USYSCAN_RxMessage.Data[2] == 'N')
			 &&(USYSCAN_RxMessage.Data[3] == 'O'))
			{
					Flag.Send_Data = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'C')&&(USYSCAN_RxMessage.Data[1] == 'A')&&(USYSCAN_RxMessage.Data[2] == 'N')
			 &&(USYSCAN_RxMessage.Data[3] == 'C'))
			{
					Flag.Send_Data = false;
			}
			if((USYSCAN_RxMessage.Data[0] == 'E')&&(USYSCAN_RxMessage.Data[1] == 'N')&&(USYSCAN_RxMessage.Data[2] == 'D')
			 &&(USYSCAN_RxMessage.Data[3] == 'A')&&(USYSCAN_RxMessage.Data[4] == 'R')&&(USYSCAN_RxMessage.Data[5] == 'M')&&(USYSCAN_RxMessage.Data[6] == '2'))
			{
					Flag.End_Frame_ARM2 = true;
			}
			if((USYSCAN_RxMessage.Data[0] == 'E')&&(USYSCAN_RxMessage.Data[1] == 'J')&&(USYSCAN_RxMessage.Data[2] == 'S'))
			{
					Flag.End_Frame_Jetson = true;
			}
	}
}
