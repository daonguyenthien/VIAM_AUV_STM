/* Includes ------------------------------------------------------------------*/
#include "User_HALLSensor.h"
#include "User_PWM.h"
#include "User_CAN.h"
#include "filter.h"
#include "matrix.h"
#include "User_Algorithm.h"
#include "User_ADC.h"
#include "User_Led.h"
#include "User_Encoder.h"
/* Public variables ----------------------------------------------------------*/
 static struct 
{
	uint8_t HA;
	uint8_t LA;
	uint8_t HB;
	uint8_t LB;
	uint8_t HC;
	uint8_t LC;
} UHALL_Position;

static uint16_t	NewPosHall = 0xFFFF;
static uint16_t OldPosHall = 0xFFFF;

static uint32_t Timer_Overload = 0;
static uint32_t Timer_Check_iMotor = 0;
static uint32_t Timer_Check_VeloMotor = 0;

static uint32_t Timer_CounterPerRevolution = 0;
static uint32_t Timer_FilterCounterPerRevolution = 0;
static float Timer_RPM = 0.0;
static uint8_t PoleCounts = 0, UALTHM_Time_Control = 0;
static bool Motor_Running = false;
static bool StartUp_Motor = true;
static bool Timer_Event = false;

/* Private types -------------------------------------------------------------*/
static union r_Speed
{
	float Value;
	
	struct
	{
		unsigned a1:8;
		unsigned a2:8;
		unsigned a3:8;
		unsigned a4:8;
	} byte;
} UHALL_ActualSpeed;

//typedef enum {PID_Speed = 0x14, PID_Position = 0x15, Fuzzy_Speed = 0x16, Fuzzy_Position = 0x17}ControllerMode;

/* Private const/macros ------------------------------------------------------*/
#define		TRUE							1
#define		FALSE							0
#define 	SPEEDMULT					150000.0f*60.0f		// 

//--------Check System--------//
#define   MAXIMUM_I_MOTOR		1000.0f // mA
#define   CHECK_I_MOTOR			50 // 50ms
#define   CHECK_VELO_MOTOR			1000 // 1000ms

//#define 	__TRANSMOTEC_MOTOR_
//#define 	__TBM_7646_XA_MOTOR__
#define __FAULHABER_MOTOR_
//#define _EC_MAXON_MOTOR

#ifdef _EC_MAXON_MOTOR
	#define 	POLEPAIRS					8
	#define 	GEARBOX						14.0f
	// AC:	1
	// BA: 	2
	// BC:	3
	// CB:	4
	// AB:	5
	// CA:	6

	static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CW[8][6] =   // Motor step CW
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
	
		static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CCW[8][6] =   // Motor step CCW
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC		 
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
#endif
	
#ifdef __FAULHABER_MOTOR_
	#define 	POLEPAIRS					1
	#define 	GEARBOX						43.0f
	// AC:	1
	// BA: 	2
	// BC:	3
	// CB:	4
	// AB:	5
	// CA:	6

	static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CW[8][6] =   // Motor step CW
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
	
		static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CCW[8][6] =   // Motor step CCW
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC		 
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
#endif
	
#ifdef __TRANSMOTEC_MOTOR_
	#define 	POLEPAIRS					4
	// AC:	1
	// BA: 	2
	// BC:	3
	// CB:	4
	// AB:	5
	// CA:	6

	static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CW[8][6] =   // Motor step CW
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
	
	static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CCW[8][6] =   // Motor step CCW
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC		 
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
#endif
	
#ifdef __TBM_7646_XA_MOTOR__
		#define 	POLEPAIRS					6
		#define 	GEARBOX						1.0f
	// AB:	1
	// BC: 	2
	// AC:	3
	// CA:	4
	// CB:	5
	// BA:	6

	static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CW[8][6] =   // Motor step
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB		 
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA			 
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
	
	static const uint8_t BLDC_MOTOR_BRIDGE_STATE_CCW[8][6] =   // Motor step
	{
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,		FALSE,	FALSE },  // 0		 		 
		 { FALSE,	TRUE    ,   TRUE ,	FALSE   ,  	FALSE,	FALSE },  // BA
		 { FALSE,	FALSE   ,   FALSE,	TRUE    ,  	TRUE ,	FALSE },  // CB
		 { FALSE,	TRUE    ,   FALSE,	FALSE   ,  	TRUE ,	FALSE },  // CA
		 { TRUE ,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	TRUE  }, 	// AC
		 { FALSE,	FALSE   ,   TRUE ,	FALSE   ,  	FALSE,	TRUE  },  // BC		 
		 { TRUE ,	FALSE   ,   FALSE,	TRUE    ,  	FALSE,	FALSE },  // AB
		 { FALSE,	FALSE   ,   FALSE,	FALSE   ,  	FALSE,	FALSE },  // 0
	};
#endif

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/

/* Private functions body ----------------------------------------------------*/
void UHALL_GPIO_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_Initstruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	// Phase A
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_Initstruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Initstruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Initstruct.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Initstruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Initstruct);	
	
	// Phase B
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_Initstruct.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Initstruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Initstruct.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_Initstruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Initstruct);
	
	// Phase C
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line2;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_Initstruct.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Initstruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Initstruct.NVIC_IRQChannelSubPriority = 0x04;
	NVIC_Initstruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Initstruct);
	
	NewPosHall = GPIO_ReadInputData(GPIOB)&0x0007;
}

void UHALL_ReadPosition(bool _state_Motor)
{
	NewPosHall = GPIO_ReadInputData(GPIOB)&0x0007;

	if(NewPosHall != OldPosHall)
	{
		OldPosHall = NewPosHall;
		if(_state_Motor == Motor_CW)
		{
			UHALL_Position.HA = BLDC_MOTOR_BRIDGE_STATE_CW[NewPosHall][0];
			UHALL_Position.LA = BLDC_MOTOR_BRIDGE_STATE_CW[NewPosHall][1];
		
			UHALL_Position.HB = BLDC_MOTOR_BRIDGE_STATE_CW[NewPosHall][2];
			UHALL_Position.LB = BLDC_MOTOR_BRIDGE_STATE_CW[NewPosHall][3];
		
			UHALL_Position.HC = BLDC_MOTOR_BRIDGE_STATE_CW[NewPosHall][4];
			UHALL_Position.LC = BLDC_MOTOR_BRIDGE_STATE_CW[NewPosHall][5];
		}
		
		else if(_state_Motor == Motor_CCW)
		{
			UHALL_Position.HA = BLDC_MOTOR_BRIDGE_STATE_CCW[NewPosHall][0];
			UHALL_Position.LA = BLDC_MOTOR_BRIDGE_STATE_CCW[NewPosHall][1];
		
			UHALL_Position.HB = BLDC_MOTOR_BRIDGE_STATE_CCW[NewPosHall][2];
			UHALL_Position.LB = BLDC_MOTOR_BRIDGE_STATE_CCW[NewPosHall][3];
		
			UHALL_Position.HC = BLDC_MOTOR_BRIDGE_STATE_CCW[NewPosHall][4];
			UHALL_Position.LC = BLDC_MOTOR_BRIDGE_STATE_CCW[NewPosHall][5];
		}
		
		UPWM_ControlBLDCFET();
	}
}

uint16_t UHALL_GetPosition(void)
{
	return (GPIO_ReadInputData(GPIOB)&0x0007);
//	return NewPosHall;
}

uint8_t UHALL_GetHALLHA(void)
{
	return UHALL_Position.HA;
}

uint8_t UHALL_GetHALLLA(void)
{
	return UHALL_Position.LA;
}

uint8_t UHALL_GetHALLHB(void)
{
	return UHALL_Position.HB;
}

uint8_t UHALL_GetHALLLB(void)
{
	return UHALL_Position.LB;
}
uint8_t UHALL_GetHALLHC(void)
{
	return UHALL_Position.HC;
}

uint8_t UHALL_GetHALLLC(void)
{
	return UHALL_Position.LC;
}

void UHALL_TIM_Configure(void)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	// TIM4 initialization for overflow every 1ms
	// Update Event (Hz) = timer_clock / ((TIM_Prescaler + 1) * (TIM_Period + 1))
	// Update Event (Hz) = 72MHz / ((719 + 1) * (99 + 1)) = 1KHz (1ms)
	TIM_DeInit(TIM4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 480 - 1;
	TIM_TimeBaseInitStruct.TIM_Period = 1500 - 1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	
	TIM_Cmd(TIM4,DISABLE);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

float UHALL_GetActualSpeed(void)
{
	UHALL_ActualSpeed.Value = Timer_RPM/GEARBOX;
	return UHALL_ActualSpeed.Value;
}

uint8_t UHALL_GetBytesActualSpeed(uint8_t byte)
{
	uint8_t value = 0;
	if(byte == 1) value = UHALL_ActualSpeed.byte.a1;
	else if(byte == 2) value = UHALL_ActualSpeed.byte.a2;
	else if(byte == 3) value = UHALL_ActualSpeed.byte.a3;
	else if(byte == 4) value = UHALL_ActualSpeed.byte.a4;
	return value;
}

void UHALL_ResetPosHall(void)
{
	UALTHM_Time_Control = 0;
	Timer_RPM = 0.0f;
	Timer_FilterCounterPerRevolution = 0;
	Timer_CounterPerRevolution = 0;
	Timer_Overload = 0;
	UHALL_ActualSpeed.Value = 0.0f;
	NewPosHall = 0xFFFF;
	OldPosHall = 0xFFFF;
//	StartUp_Motor = true;
}

void UHALL_ResetTimerCheckVeloMotor(void)
{
	Timer_Check_VeloMotor = 0;
}

void UHALL_MotorStatus(StatusofMotor _StatusMotor)
{
	if(_StatusMotor == RUN)
	{
		Motor_Running = true;
	}
	else
	{
		Motor_Running = false;
		UHALL_ResetTimerCheckVeloMotor();
	}
}

void UHALL_CheckStartUpPosition(bool _state_Motor)
{
	if(StartUp_Motor)
	{
		if(((GPIO_ReadInputData(GPIOB)&0x0007) == 0x0004) && (_state_Motor == Motor_CW))
		{
			UHALL_ReadPosition(Motor_CCW);
			UPWM_SetDutyCycle(500);
//			while((GPIO_ReadInputData(GPIOB)&0x0007) == 0x0004);
			UDELAY_ms(2000);
			UPWM_SetDutyCycle(0);
			NewPosHall = 0xFFFF;
			OldPosHall = 0xFFFF;
		}
		else if(((GPIO_ReadInputData(GPIOB)&0x0007) == 0x0002) && (_state_Motor == Motor_CCW))
		{
			UHALL_ReadPosition(Motor_CW);
			UPWM_SetDutyCycle(500);
			UDELAY_ms(2000);
//			while((GPIO_ReadInputData(GPIOB)&0x0007) == 0x0002);
			UPWM_SetDutyCycle(0);
			NewPosHall = 0xFFFF;
			OldPosHall = 0xFFFF;
		}				
	}

	StartUp_Motor = false;
}

bool UHALL_GetTimerEvent(void){
	return Timer_Event;
}

void UHALL_ResetTimerEvent(void){
	Timer_Event = false;
}

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
//		UHALL_ResetTimerCheckVeloMotor();
		
		if(UCAN_GetFlag(Flag_Motor_Run_CW))
		{
			UHALL_ReadPosition(Motor_CW);
		}
		else if(UCAN_GetFlag(Flag_Motor_Run_CCW))
		{
			UHALL_ReadPosition(Motor_CCW);
		}
		
//		PoleCounts++;
//		if(PoleCounts == (POLEPAIRS*2))
//		{
//			PoleCounts = 0;
//			Timer_CounterPerRevolution = (uint32_t)TIM_GetCounter(TIM4) + 1 + Timer_Overload*150;
//			if(Timer_FilterCounterPerRevolution == 0)
//			{
//				Timer_FilterCounterPerRevolution = Timer_CounterPerRevolution;
//			}
//			else
//			{
//				Timer_FilterCounterPerRevolution = (Timer_FilterCounterPerRevolution + Timer_CounterPerRevolution)/2;
//			}
//			Timer_RPM = (float)SPEEDMULT/(float)Timer_FilterCounterPerRevolution;

//			Timer_Overload = 0;
//			TIM_Cmd(TIM4, DISABLE);
//			TIM_SetCounter(TIM4,0);
//			TIM_Cmd(TIM4, ENABLE);
//		}
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
//		UHALL_ResetTimerCheckVeloMotor();
		
		if(UCAN_GetFlag(Flag_Motor_Run_CW))
		{
			UHALL_ReadPosition(Motor_CW);
		}
		else if(UCAN_GetFlag(Flag_Motor_Run_CCW))
		{
			UHALL_ReadPosition(Motor_CCW);
		}
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
//		UHALL_ResetTimerCheckVeloMotor();
		
		if(UCAN_GetFlag(Flag_Motor_Run_CW))
		{
			UHALL_ReadPosition(Motor_CW);
		}
		else if(UCAN_GetFlag(Flag_Motor_Run_CCW))
		{
			UHALL_ReadPosition(Motor_CCW);
		}
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update))
	{
//		Timer_Check_iMotor++;
//		if(Motor_Running == true) Timer_Check_VeloMotor++;
//		
//		if(Motor_Running == true) Timer_Check_VeloMotor++;
//		
//		if(Timer_Check_iMotor >= CHECK_I_MOTOR)
//		{
//			Timer_Check_iMotor = 0;
//			if(UADC_GetValue(iMotor) > MAXIMUM_I_MOTOR)
//			{
//				UCAN_SystemOverLoad();
//				UPWM_StopPWM();
//			}			
//		}
//		
//		if(Timer_Check_VeloMotor >= CHECK_VELO_MOTOR)
//		{
//			Timer_Check_VeloMotor = 0;
//			if(UPWM_GetRefDutyCycle() != 0)
//			{
//				UCAN_StickedMotor();
//				UPWM_StopPWM();
//			}			
//		}
		
		ULED_Toggle(ORANGE);
		Timer_Event = true;
		if ((!UCAN_GetFlag(Flag_Motor_Position))){
			UENC_GetActualSpeed();
		}
		
//		if(UCAN_GetFlag(Flag_Open_Loop)||UCAN_GetFlag(Flag_PID_Control)||UCAN_GetFlag(Flag_Fuzzy_Control))
//		{
//			Timer_Overload++;
//		}
//		
		if(UCAN_GetFlag(Flag_PID_Control)||UCAN_GetFlag(Flag_Fuzzy_Control))
		{
			UALTHM_Time_Control++;
			if(UALTHM_Time_Control == UALTHM_TIME_SAMPLING)
			{
				if (!UCAN_GetFlag(Flag_Motor_Position)){
					if(UCAN_GetFlag(Flag_Motor_Run_CW))
					{
						UALTHM_Controller(Motor_CW);
					}
					else if(UCAN_GetFlag(Flag_Motor_Run_CCW))
					{
						UALTHM_Controller(Motor_CCW);
					}
				}
				else UALTHM_Controller(Motor_CW);
				UALTHM_Time_Control = 0;
			}
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	}
}
