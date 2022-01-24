/* Includes ------------------------------------------------------------------*/
#include "User_Encoder.h"
#include "User_CAN.h"
#include "User_LED.h"
#include "User_Filter.h"
#include "User_Algorithm.h"
/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

static union r_Position
{
	float Value;
	
	struct
	{
		unsigned a1:8;
		unsigned a2:8;
		unsigned a3:8;
		unsigned a4:8;
	} byte;
} UENC_ActualPosition;

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
} UENC_ActualSpeed;
/* Private const/macros ------------------------------------------------------*/

#define __FAULHABER_MOTOR_
//#define _EC_MAXON_MOTOR

#define VISME						4   // 4mm/revolution 
#ifdef _EC_MAXON_MOTOR
	#define 	GEARBOX						14.0f
	#define 	PULSE_PER_ROTATION 500.0f
#endif

#ifdef __FAULHABER_MOTOR_
	#define 	GEARBOX						43.0f
	#define 	PULSE_PER_ROTATION 1000.0f
#endif

/* Private variables ---------------------------------------------------------*/
__IO uint16_t revolution_count = 0;
FirstOrderParameter myFilterEnc;

/* Private function prototypes -----------------------------------------------*/

void UENC_Configure(void)	{
	// GPIO Tim 2 channel 1, 2 configuration.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef UENC_GPIOInitStruct;
	UENC_GPIOInitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	UENC_GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	UENC_GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &UENC_GPIOInitStruct);
	
	
	//TIM2 configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	TIM_TimeBaseInitTypeDef UENC_TIM2TimeBaseInitStruct;
	UENC_TIM2TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	UENC_TIM2TimeBaseInitStruct.TIM_CounterMode  	= TIM_CounterMode_Up;
	UENC_TIM2TimeBaseInitStruct.TIM_Prescaler			= 0;
	UENC_TIM2TimeBaseInitStruct.TIM_Period 				= GEARBOX*PULSE_PER_ROTATION - 1;
	UENC_TIM2TimeBaseInitStruct.TIM_RepetitionCounter	= 0;
	TIM_TimeBaseInit(TIM2, &UENC_TIM2TimeBaseInitStruct);
	
	

	//Encoder Interface configuration
	
	
	TIM_ICInitTypeDef		UENC_TIM2_ICInitStruct;
	TIM_ICStructInit(&UENC_TIM2_ICInitStruct);
	UENC_TIM2_ICInitStruct.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
	UENC_TIM2_ICInitStruct.TIM_ICFilter = 6;
//	UENC_TIM2_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	UENC_TIM2_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	//Mode x4;
	TIM_ICInit(TIM2, &UENC_TIM2_ICInitStruct);
	TIM2->CNT = 0;
	
	NVIC_InitTypeDef		TIM2_NVIC_InitStruct;
	TIM2_NVIC_InitStruct.NVIC_IRQChannel	= TIM2_IRQn;
	TIM2_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 0x01;
	TIM2_NVIC_InitStruct.NVIC_IRQChannelSubPriority					= 0x00;
	TIM2_NVIC_InitStruct.NVIC_IRQChannelCmd									= ENABLE;
	NVIC_Init(&TIM2_NVIC_InitStruct);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	
}

uint32_t UENC_GetCounterValue(void){
	myFilterEnc.a =  0.6321f;
	myFilterEnc.b =  0.3679f;
	uint32_t _base_pulse_count = revolution_count*GEARBOX*PULSE_PER_ROTATION;
	uint16_t  _current_count = TIM_GetCounter(TIM2);
	//uint32_t temp_count = (uint32_t)UFIL_1stOrderFilter_Output(&myFilterEnc, _base_pulse_count + _current_count);
	return  _base_pulse_count + _current_count;
}

float UENC_GetSpeed(void){
	return UENC_ActualSpeed.Value;
}

float UENC_GetActualSpeed(void)
{
	static int preCounterValue = 0;
	uint16_t currentCounterValue = UENC_GetCounterValue();
	uint16_t temp = currentCounterValue - preCounterValue;
	preCounterValue = currentCounterValue;
	if (temp < 0) temp = -temp;
	UENC_ActualSpeed.Value = temp/PULSE_PER_ROTATION/4*1000.0*60.0/UALTHM_TIME_SAMPLING;
	return UENC_ActualSpeed.Value;
}

uint8_t UENC_GetBytesActualSpeed(uint8_t byte)
{
	uint8_t value = 0;
	if(byte == 1) value = UENC_ActualSpeed.byte.a1;
	else if(byte == 2) value = UENC_ActualSpeed.byte.a2;
	else if(byte == 3) value = UENC_ActualSpeed.byte.a3;
	else if(byte == 4) value = UENC_ActualSpeed.byte.a4;
	return value;
}

float UENC_GetActualPosition(void)
{
	UENC_ActualPosition.Value = (float)UENC_GetCounterValue()/(GEARBOX*PULSE_PER_ROTATION);
	return UENC_ActualPosition.Value;
}

uint8_t UENC_GetBytesActualPosition(uint8_t byte)
{
	uint8_t value = 0;
	if(byte == 1) value = UENC_ActualPosition.byte.a1;
	else if(byte == 2) value = UENC_ActualPosition.byte.a2;
	else if(byte == 3) value = UENC_ActualPosition.byte.a3;
	else if(byte == 4) value = UENC_ActualPosition.byte.a4;
	return value;
}

uint32_t UENCmm2pulse(float _distance){
	uint32_t pulse_per_rotation = GEARBOX*PULSE_PER_ROTATION*4;
	uint32_t pulse_per_millimeter = pulse_per_rotation/VISME;
	uint32_t count = _distance*pulse_per_millimeter;
	return count;
}

float UENCpulse2mm(uint32_t _pulse){
	uint32_t pulse_per_rotation = GEARBOX*PULSE_PER_ROTATION*4;
	float mm = _pulse/pulse_per_rotation*VISME;
	return mm;
}

void TIM2_IRQHandler(void){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (UCAN_GetFlag(Flag_Motor_Run_CW))
			revolution_count++;
		else if (UCAN_GetFlag(Flag_Motor_Run_CCW))
			if (revolution_count > 0) revolution_count--;
//		ULED_Toggle(GREEN);
	}
}