/* Includes ------------------------------------------------------------------*/
#include "User_PWM.h"
#include "User_Algorithm.h"
#include "User_CAN.h"

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
uint16_t UPWM_TimerPeriod = 0;

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
} UPWM_DutyCycle, UPWM_RefDutyCycle;

/* Private const/macros ------------------------------------------------------*/
//const uint16_t PWM_Frequency = 45000; // 45KHz
//const uint16_t PWM_Frequency = 25000; // 25KHz
//const uint16_t PWM_Frequency = 20000; // 20KHz
//const uint16_t PWM_Frequency = 19200; // 19.2KHz
const uint16_t PWM_Frequency = 18000; // 18KHz
//const uint16_t PWM_Frequency = 15000; // 15KHz
const uint32_t Time_Pre_Charge_Bootstrap = 100; // 500 us

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void UPWM_Pre_Charge_Bootstrap(void)
{
	// https://forum.allaboutcircuits.com/threads/sensored-bldc-start-up-failure.44632/
	TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);
	TIM_SelectOCxM(TIM1,TIM_Channel_1,TIM_ForcedAction_Active);
	TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable);
	
	TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Disable);
	TIM_SelectOCxM(TIM1,TIM_Channel_2,TIM_ForcedAction_Active);
	TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Enable);
	
	TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Disable);
	TIM_SelectOCxM(TIM1,TIM_Channel_3,TIM_ForcedAction_Active);
	TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Enable);
	
	UDELAY_us(Time_Pre_Charge_Bootstrap);
	
	TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
}

/* Exported function body ----------------------------------------------------*/

/* Private functions body ----------------------------------------------------*/
	void UPWM_GPIO_Configure(void)
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStruct);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&GPIO_InitStruct);
		
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&GPIO_InitStruct);
		
	}

	void UPWM_TIM_Configure(void)
	{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_OCInitTypeDef TIM_OCInitStruct;
		TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_AFIO,ENABLE);
		
		UPWM_TimerPeriod = (SystemCoreClock / PWM_Frequency) - 1;
		
		UPWM_DutyCycle.Value = 0;
		/* Time Base configuration */
		TIM_TimeBaseInitStruct.TIM_Prescaler = 1 - 1;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = UPWM_TimerPeriod;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;

		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Timing;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStruct.TIM_Pulse = (uint16_t)UPWM_DutyCycle.Value;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
		TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;

		TIM_OC1Init(TIM1, &TIM_OCInitStruct);
		TIM_OC2Init(TIM1, &TIM_OCInitStruct);
		TIM_OC3Init(TIM1, &TIM_OCInitStruct);
		
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Enable;
		TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable;
		TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
		TIM_BDTRInitStruct.TIM_DeadTime = 150;
		TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
		TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_High;
		TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

		TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
		
		/* TIM1 counter enable */
		TIM_Cmd(TIM1, ENABLE);

		/* Main Output Enable */
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		
		UPWM_Pre_Charge_Bootstrap();
	}
	
	void UPWM_ControlBLDCFET(void)
	{
		// Phase A
		if(UHALL_GetHALLHA())
		{
			TIM_SelectOCxM(TIM1,TIM_Channel_1,TIM_OCMode_PWM1);
			TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable);
		}
		else
		{
			TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);
			
			if(UHALL_GetHALLLA())
			{
				TIM_SelectOCxM(TIM1,TIM_Channel_1,TIM_ForcedAction_Active);
				TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable);
			}
			else
			{
				TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);
			}
		}
		
		// Phase B
		if(UHALL_GetHALLHB())
		{
			TIM_SelectOCxM(TIM1,TIM_Channel_2,TIM_OCMode_PWM1);
			TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Enable);
		}
		else
		{
			TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Disable);
			
			if(UHALL_GetHALLLB())
			{
				TIM_SelectOCxM(TIM1,TIM_Channel_2,TIM_ForcedAction_Active);
				TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Enable);
			}
			else
			{
				TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
			}
		}
		
		// Phase C
		if(UHALL_GetHALLHC())
		{
			TIM_SelectOCxM(TIM1,TIM_Channel_3,TIM_OCMode_PWM1);
			TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Enable);
		}
		else
		{
			TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Disable);
			
			if(UHALL_GetHALLLC())
			{
				TIM_SelectOCxM(TIM1,TIM_Channel_3,TIM_ForcedAction_Active);
				TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Enable);
			}
			else
			{
				TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
			}
		}
	}

void UPWM_SetDutyCycle(uint32_t DutyCycle)
{	
	TIM_SetCompare1(TIM1,(uint16_t)DutyCycle);
	TIM_SetCompare2(TIM1,(uint16_t)DutyCycle);
	TIM_SetCompare3(TIM1,(uint16_t)DutyCycle);
}
//------------------------------DutyCycle Get From PC-----------------------------/
void UPWM_SetBytesDutyCycle(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
	UPWM_DutyCycle.byte.a1 = a1;
	UPWM_DutyCycle.byte.a2 = a2;
	UPWM_DutyCycle.byte.a3 = a3;
	UPWM_DutyCycle.byte.a4 = a4;
	
//	UHALL_CheckStartUpPosition();
	
	uint16_t Duty = (uint16_t)(UPWM_DutyCycle.Value*(float)PWM_DC_MAX/100.0);
	if((Duty >= PWM_DC_MIN)&&(Duty <= PWM_DC_MAX))
	{
		UPWM_SetDutyCycle(Duty);
	} 
	else if(Duty > PWM_DC_MAX)
	{
		UPWM_SetDutyCycle(PWM_DC_MAX);
	}
	else if(Duty < PWM_DC_MIN)
	{
		UPWM_SetDutyCycle(0);
		UPWM_StopPWM();
	}
}
//------------------------------DutyCycle Send To PC-----------------------------/
uint8_t UPWM_GetBytesDutyCycle(uint8_t byte)
{
	UPWM_RefDutyCycle.Value = (float)((float)TIM_GetCapture1(TIM1)*100.0/(float)PWM_DC_MAX);
	uint8_t value = 0;
	if(byte == 1) value = UPWM_RefDutyCycle.byte.a1;
	else if( byte == 2) value = UPWM_RefDutyCycle.byte.a2;
	else if( byte == 3) value = UPWM_RefDutyCycle.byte.a3;
	else if (byte == 4) value = UPWM_RefDutyCycle.byte.a4;
	return value;
}

float UPWM_GetRefDutyCycle(void)
{
	UPWM_RefDutyCycle.Value = (float)((float)TIM_GetCapture1(TIM1)*100.0/(float)PWM_DC_MAX);
	return UPWM_RefDutyCycle.Value;
}

void UPWM_StopPWM(void)
{
//	TIM_Cmd(TIM4,DISABLE);
//	TIM_SetCounter(TIM4,0);
//	UCAN_ResetFlag();
//	
	UPWM_Pre_Charge_Bootstrap();
	
	TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);
	TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
	TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
	UDELAY_ms(500);
	UHALL_ResetPosHall();
}
