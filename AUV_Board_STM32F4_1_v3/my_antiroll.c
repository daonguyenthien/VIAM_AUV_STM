/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "my_antiroll.h"
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef enum MotorDirection {CW = 0, CCW = 1, STOP = 2} MotorDirection;
/* Private define ------------------------------------------------------------*/

		/** 
		* @brief   ENC define 
		*/
			#define UARENC_Channel_A								GPIO_Pin_0  //PA0_Y
			#define UARENC_Channel_A_SOURCE					GPIO_PinSource0
			
			#define UARENC_Channel_B       					GPIO_Pin_1  //PA1_G
			#define UARENC_Channel_B_SOURCE					GPIO_PinSource1
			
			#define UARENC_PORT											GPIOA
			#define UARENC_CLK											RCC_AHB1Periph_GPIOA
			#define UARENC_CLK_Cmd									RCC_AHB1PeriphClockCmd
			#define UARENC_AF												GPIO_AF_TIM2
			
			#define UARENC_TIM											TIM2
			#define UARENC_TIM_CLK									RCC_APB1Periph_TIM2
			#define UARENC_TIM_CLK_Cmd							RCC_APB1PeriphClockCmd
			#define UARENC_TIM_IRQn									TIM2_IRQn
			#define UARENC_TIM_PreemptionPriority		0x00
			#define UARENC_TIM_SubPriority					0X01
			
		/** 
		* @brief   Timer for interrupt define 
		*/
			#define UARENC_TIM_IT										TIM7
			#define UARENC_TIM_IT_CLK								RCC_APB1Periph_TIM7
			#define UARENC_TIM_IT_CLK_Cmd						RCC_APB1PeriphClockCmd
			#define UARENC_TIM_IT_IRQn							TIM7_IRQn
			#define UARENC_TIM_IT_IRQHandler				TIM7_IRQHandler
			
			
			
			
			#define PWM_PIN											GPIO_Pin_8
			#define PWM_SOURCE									GPIO_PinSource8
			#define PWM_AF											GPIO_AF_TIM1
			#define PWM_PORT										GPIOA
			#define PWM_CLK											RCC_AHB1Periph_GPIOA
			#define PWM_CLK_Cmd									RCC_AHB1PeriphClockCmd
			
/** 
		* @brief   DIR Pin define 
		*/
																																		
			#define DIR_R_PIN										GPIO_Pin_2								//HIGH			LOW			LOW			HIGH
			#define DIR_L_PIN										GPIO_Pin_3								//LOW				HINGH		LOW			HIGH
																																		//CW				CCW			STOP		BURN
			#define DIR_R_PORT									GPIOA
			#define DIR_R_CLK										RCC_AHB1Periph_GPIOA
			#define DIR_R_CLK_Cmd								RCC_AHB1PeriphClockCmd

			#define DIR_L_PORT									GPIOA
			#define DIR_L_CLK										RCC_AHB1Periph_GPIOA
			#define DIR_L_CLK_Cmd								RCC_AHB1PeriphClockCmd
			
/** 
		* @brief   Timer for PWM define 
		*/
		
			#define PWM_TIM											TIM1
			#define	PWM_TIM_Channel							TIM_Channel_1
			#define PWM_TIM_CLK									RCC_APB2Periph_TIM1
			#define PWM_TIM_CLK_Cmd							RCC_APB2PeriphClockCmd
			#define PWM_TIM_Period							99
			#define PWM_TIM_Pulse								79
				
			#define MAX_PULSE 95
			#define STOP_PULSE 55
			#define SAMPLINGTIME 20 				//Sampling time of 20ms
			#define MOTOR_PPR 2970	
			
			#define PWM_FREQ 10000
			#define PWM_RESOLUTION	1000000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t targetDuty;
int16_t Count;
static float pulse;

/*
	For PID
	*/
float error, error1=0, error2=0, value=0;

int32_t Rotary, Counter;
int8_t direct, dir;
double Position_PV;
float Setpoint = 0;
float Kp=0, Ki=0, Kd=0;

/*End PID*/
/* Private function prototypes -----------------------------------------------*/
static void UANRO_ENC_Config(uint16_t SampleTime);
static void UANRO_PWM_Config(uint32_t freq);
static void UARENCResetEncoder(void);
static void UANRO_CW_Direction(void);
static void UANRO_PWM_Out(uint16_t duty1);
static void UANRO_CW_Direction(void);
static void UANRO_CCW_Direction(void);
static void UANRO_Stop_Motor(void);
static void UANRO_ResetEncoder(void);
static void TIM2_Config(void);
static void TIM7_Config(uint16_t SampleTime);
static void UANRO_Limit_value(void);
static float UANRO_PID_Control_Position(float setPoint, float measure, float Kp, float Ki, float Kd);
/* Private functions ---------------------------------------------------------*/

  /*! \addtogroup Config 
   *  Config encoder, pwm timer
   *  @{
   */
/**
  * @brief  ENCODER Configuration
  * @param  None
  * @retval None
  */
void UANRO_Config(void){
	
	UANRO_ENC_Config(SAMPLINGTIME);
	UANRO_PWM_Config(PWM_FREQ);
}

void UANRO_Setpoint(float _setpoint){
	Setpoint = _setpoint;
}

void UANRO_SetParameter(float _Kp, float _Ki, float _Kd){
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
}

static void UANRO_ENC_Config(uint16_t SampleTime){
	/*TIM2, PA0 Channel 1, PA1 Channel 2*/
	/**/
	TIM2_Config();
	TIM7_Config(SampleTime);
	/*LED for indication that TIM7 is working*/
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//	GPIO_InitTypeDef 									IC_GPIO_InitStruct;
//	IC_GPIO_InitStruct.GPIO_Mode 			= GPIO_Mode_OUT;
//	IC_GPIO_InitStruct.GPIO_OType 		= GPIO_OType_PP;
//	IC_GPIO_InitStruct.GPIO_PuPd 			= GPIO_PuPd_NOPULL;
//	IC_GPIO_InitStruct.GPIO_Speed			= GPIO_Speed_100MHz;
//	IC_GPIO_InitStruct.GPIO_Pin 			= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_Init(GPIOD, &IC_GPIO_InitStruct);
}


static void UANRO_PWM_Config(uint32_t freq){
	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//	GPIO_InitTypeDef 										IC_GPIO_InitStruct;
//	IC_GPIO_InitStruct.GPIO_Mode 			= GPIO_Mode_OUT;
//	IC_GPIO_InitStruct.GPIO_OType 		= GPIO_OType_PP;
//	IC_GPIO_InitStruct.GPIO_PuPd 			= GPIO_PuPd_NOPULL;
//	IC_GPIO_InitStruct.GPIO_Speed			= GPIO_Speed_100MHz;
//	IC_GPIO_InitStruct.GPIO_Pin 			= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_Init(GPIOD, &IC_GPIO_InitStruct);
	
	// use A9 A10 FOR DIRECTION
	DIR_R_CLK_Cmd(DIR_R_CLK, ENABLE);
	GPIO_InitTypeDef 													DIRECTION_R_GPIO_InitStruct;
	DIRECTION_R_GPIO_InitStruct.GPIO_Mode 			= GPIO_Mode_OUT;
	DIRECTION_R_GPIO_InitStruct.GPIO_OType 			= GPIO_OType_PP;
	DIRECTION_R_GPIO_InitStruct.GPIO_PuPd 			= GPIO_PuPd_NOPULL;
	DIRECTION_R_GPIO_InitStruct.GPIO_Speed			= GPIO_Speed_100MHz;
	DIRECTION_R_GPIO_InitStruct.GPIO_Pin 				= DIR_R_PIN;
	GPIO_Init(DIR_R_PORT, &DIRECTION_R_GPIO_InitStruct);
	
	DIR_L_CLK_Cmd(DIR_L_CLK, ENABLE);
	GPIO_InitTypeDef 													DIRECTION_L_GPIO_InitStruct;
	DIRECTION_L_GPIO_InitStruct.GPIO_Mode 			= GPIO_Mode_OUT;
	DIRECTION_L_GPIO_InitStruct.GPIO_OType 			= GPIO_OType_PP;
	DIRECTION_L_GPIO_InitStruct.GPIO_PuPd 			= GPIO_PuPd_NOPULL;
	DIRECTION_L_GPIO_InitStruct.GPIO_Speed			= GPIO_Speed_2MHz;
	DIRECTION_L_GPIO_InitStruct.GPIO_Pin 				= DIR_L_PIN;
	GPIO_Init(DIR_L_PORT, &DIRECTION_L_GPIO_InitStruct);
	
	//USE A8 FOR PWM 
	PWM_CLK_Cmd(PWM_CLK, ENABLE);
	GPIO_InitTypeDef 																	PWM_GPIO_InitStruct;
	PWM_GPIO_InitStruct.GPIO_Pin 										= PWM_PIN;
	PWM_GPIO_InitStruct.GPIO_Mode 									= GPIO_Mode_AF;
	PWM_GPIO_InitStruct.GPIO_OType 									= GPIO_OType_PP;
	PWM_GPIO_InitStruct.GPIO_PuPd 									= GPIO_PuPd_NOPULL;
	PWM_GPIO_InitStruct.GPIO_Speed 									= GPIO_Speed_2MHz;
	
	GPIO_PinAFConfig(PWM_PORT, PWM_SOURCE, PWM_AF);
	
	GPIO_Init(PWM_PORT, &PWM_GPIO_InitStruct);
	// Time base
	
	PWM_TIM_CLK_Cmd(PWM_TIM_CLK, ENABLE);
	TIM_TimeBaseInitTypeDef 													PWM_TimeBaseInitStruct;
	PWM_TimeBaseInitStruct.TIM_ClockDivision 				= TIM_CKD_DIV1;
	PWM_TimeBaseInitStruct.TIM_Prescaler     				= SystemCoreClock/freq - 1;
	PWM_TimeBaseInitStruct.TIM_CounterMode				 	= TIM_CounterMode_Up;
	PWM_TimeBaseInitStruct.TIM_RepetitionCounter    = 0;
	PWM_TimeBaseInitStruct.TIM_Period								= PWM_TIM_Period;
	
	/*Timer_tick_frequence = Timer_default_frequence/(Prescaler+1)
	  TIM_Period = (Timer_tick_frequence/PWM_frequency )-1
		Pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 – 1*/
		
	TIM_TimeBaseInit(PWM_TIM, &PWM_TimeBaseInitStruct);
	
	//Output compare
	TIM_OCInitTypeDef 																PWM_OCInitStruct;
	PWM_OCInitStruct.TIM_OCMode											= TIM_OCMode_PWM1;
	PWM_OCInitStruct.TIM_Pulse											= PWM_TIM_Pulse;
	PWM_OCInitStruct.TIM_OCPolarity									= TIM_OCPolarity_High;
	PWM_OCInitStruct.TIM_OCNPolarity								= TIM_OCNPolarity_High;
	PWM_OCInitStruct.TIM_OCIdleState								= TIM_OCIdleState_Set;
	PWM_OCInitStruct.TIM_OCNIdleState								= TIM_OCNIdleState_Set;
	PWM_OCInitStruct.TIM_OutputState								= TIM_OutputState_Enable;
	PWM_OCInitStruct.TIM_OutputNState								= TIM_OutputNState_Enable;
	
	TIM_OC1Init(PWM_TIM, &PWM_OCInitStruct);
	//TIM_OC2Init(PWM_TIM, &PWM_OCInitStruct);
	
	TIM_ARRPreloadConfig(PWM_TIM, ENABLE);
	TIM_CCxCmd(PWM_TIM, PWM_TIM_Channel, TIM_CCx_Enable);
	TIM_CCxNCmd(PWM_TIM, PWM_TIM_Channel, TIM_CCxN_Disable);
	TIM_Cmd(PWM_TIM, ENABLE);
	
	TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
}


/**
  * @brief  Config TIM2 for reading encoder.
  * @param  None.
  * @retval None
  */

static void TIM2_Config(void){
	/*Provide clock for timer 2*/
	UARENC_TIM_CLK_Cmd(UARENC_TIM_CLK, ENABLE);
	/*Provide clock for GPIOA*/
	UARENC_CLK_Cmd(UARENC_CLK, ENABLE);
	
	GPIO_InitTypeDef 									IC_GPIO_InitStruct;
	IC_GPIO_InitStruct.GPIO_Mode 			= GPIO_Mode_AF;
	IC_GPIO_InitStruct.GPIO_OType 		= GPIO_OType_PP;
	IC_GPIO_InitStruct.GPIO_PuPd 			= GPIO_PuPd_UP;
	IC_GPIO_InitStruct.GPIO_Speed			= GPIO_Speed_100MHz;
	IC_GPIO_InitStruct.GPIO_Pin 			= UARENC_Channel_A | UARENC_Channel_B;
	
	GPIO_PinAFConfig(UARENC_PORT, UARENC_Channel_A_SOURCE, UARENC_AF);
	GPIO_PinAFConfig(UARENC_PORT, UARENC_Channel_B_SOURCE, UARENC_AF);
	
	GPIO_Init(UARENC_PORT, &IC_GPIO_InitStruct);
	
	TIM_TimeBaseInitTypeDef						IC_TimeBaseInitStruct;
	IC_TimeBaseInitStruct.TIM_ClockDivision 	= TIM_CKD_DIV1;
	IC_TimeBaseInitStruct.TIM_Period 					= 0xffff;
	IC_TimeBaseInitStruct.TIM_Prescaler 			= 0;
	IC_TimeBaseInitStruct.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(UARENC_TIM, &IC_TimeBaseInitStruct);
	
	
	
	TIM_ICInitTypeDef 									TIM_ICInitStructure;  
	TIM_ICStructInit(&TIM_ICInitStructure);
	
 	TIM_ICInit(UARENC_TIM, &TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(UARENC_TIM,TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, 	TIM_ICPolarity_Rising);	
	TIM_ClearFlag(UARENC_TIM, TIM_FLAG_Update);
	TIM_ITConfig(UARENC_TIM, TIM_IT_Update, ENABLE);
	TIM_SetCounter(UARENC_TIM,30000);
	//TIM_SetAutoreload(TIM2, 0xffff);
	
	NVIC_InitTypeDef  				NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = UARENC_TIM_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UARENC_TIM_PreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = UARENC_TIM_SubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  TIM_ITConfig(UARENC_TIM,TIM_IT_CC1,ENABLE);
	TIM_Cmd(UARENC_TIM, ENABLE);
}


/**
  * @brief  Config TIM7 for sampling interrupt
  * @param  None
  * @retval None
  */
static void TIM7_Config(uint16_t SampleTime){
	UARENC_TIM_IT_CLK_Cmd(UARENC_TIM_IT_CLK, ENABLE);
	
	TIM_TimeBaseInitTypeDef						TimeBaseInitStruct;
	TimeBaseInitStruct.TIM_ClockDivision 		= TIM_CKD_DIV1;
	TimeBaseInitStruct.TIM_Period 					= SampleTime*2;  	//dem SampleTime*2 thi tran va reset dem lai -> SampleTime(ms) la thoi gian dinh thi
	TimeBaseInitStruct.TIM_Prescaler 				= 41999;					//tan so cua prescaler (84MHz/x) - 1 = 41999 -> 2000Hz
	TimeBaseInitStruct.TIM_CounterMode 			= TIM_CounterMode_Up;
	TIM_TimeBaseInit(UARENC_TIM_IT, &TimeBaseInitStruct);
	
	NVIC_InitTypeDef TIM7_NVICInitStruct;
	TIM7_NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	TIM7_NVICInitStruct.NVIC_IRQChannelSubPriority	  		= 0;
	TIM7_NVICInitStruct.NVIC_IRQChannel 									= UARENC_TIM_IT_IRQn;
	TIM7_NVICInitStruct.NVIC_IRQChannelCmd								= ENABLE;
	NVIC_Init(&TIM7_NVICInitStruct);
	
	TIM_ITConfig(UARENC_TIM_IT, TIM_IT_Update, ENABLE);
	TIM_Cmd(UARENC_TIM_IT, ENABLE);
}
 /*! @} */

  /*! \addtogroup PWM control function 
   *  Config encoder, pwm timer
   *  @{
   */

static void UANRO_ResetEncoder(void){
	Setpoint = 0;
	TIM_SetCounter(TIM2, 30000);
}

static void UANRO_PWM_Out(uint16_t duty1){
	TIM_SetCompare1(PWM_TIM, duty1);
}


/**
  * @brief  CW_Direction
  * @param  None
  * @retval None
  */
static void UANRO_CW_Direction(void){
	GPIO_SetBits(DIR_R_PORT, DIR_R_PIN);
	GPIO_ResetBits(DIR_L_PORT, DIR_L_PIN);
}

/**
  * @brief  CCW_Direction
  * @param  None
  * @retval None
  */
static void UANRO_CCW_Direction(void){
	GPIO_SetBits(DIR_L_PORT, DIR_L_PIN);
	GPIO_ResetBits(DIR_R_PORT, DIR_R_PIN);
}

/**
  * @brief  Stop_Motor
  * @param  None
  * @retval None
  */
static void UANRO_Stop_Motor(void){
	GPIO_ResetBits(DIR_R_PORT, DIR_R_PIN);
	GPIO_ResetBits(DIR_L_PORT, DIR_L_PIN);
}

 /*! @} */


  /*! \addtogroup PID control functions
   *  Config encoder, pwm timer
   *  @{
   */

static void UANRO_Limit_value(void)
{
	if(value >= MAX_PULSE) value = MAX_PULSE;
	if(value <= STOP_PULSE) value = STOP_PULSE;
}

static float UANRO_PID_Control_Position(float setPoint, float measure, float Kp, float Ki, float Kd)
{ 
	error = setPoint - measure;
	value = Kp*(error) + Ki*(SAMPLINGTIME/1000/2)*(error + error1) + (Kd/SAMPLINGTIME/1000)*(error - 2*error1 + error2);
	error2 = error1;
	error1 = error;
	
	if(setPoint>30000)
	{
	if (error>70)
		{
			UANRO_CW_Direction();
			value = STOP_PULSE + value;
			UANRO_Limit_value();
			return value;
    }
  else if(error<-70)
		{
			UANRO_CCW_Direction();
			value = STOP_PULSE - value;
			UANRO_Limit_value();
			return value;
		}
  else
		{
			UANRO_Stop_Motor();
			UANRO_ResetEncoder();
      value = STOP_PULSE;
			return value;
		}
	}
	else
	{
		if (error<-70)
		{
			UANRO_CCW_Direction();
			value = STOP_PULSE - value;
			UANRO_Limit_value();
			return value;
    }
		else if(error>70)
		{
			UANRO_CW_Direction();
			value = STOP_PULSE + value;
			UANRO_Limit_value();
			return value;
		}
		else
		{
			UANRO_Stop_Motor();
			UANRO_ResetEncoder();
      value = STOP_PULSE;
			return value;
		}
	}
}


 /*! @} */

/**
  * @}
  */



/**
  * @brief  Interrupt TIM2
  * @param  None
  * @retval None
  */

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
			Counter = TIM_GetCounter(TIM2);
//			if((Counter >= (MOTOR_PPR*4)) | (Counter <= -(MOTOR_PPR*4)))
//         {
//					 TIM_SetCounter(TIM2,30000);
//				 }
			//GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		}
}

/**
  * @brief  Interrupt TIM7
  * @param  None
  * @retval None
  */
void UARENC_TIM_IT_IRQHandler(void)
{
	if (TIM_GetITStatus(UARENC_TIM_IT, TIM_IT_Update) == SET)
		{
			TIM_ClearITPendingBit(UARENC_TIM_IT, TIM_IT_Update);
			pulse = (Setpoint*MOTOR_PPR*4)/360 + 30000;
			Counter = TIM_GetCounter(UARENC_TIM);
			targetDuty = UANRO_PID_Control_Position(pulse, Counter, Kp, Ki, Kd);
			UANRO_PWM_Out(targetDuty);
			//GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
			
//			send_data();
//			DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
//			DMA_SetCurrDataCounter(DMA1_Stream3, 12);
//			DMA_Cmd(DMA1_Stream3, ENABLE);
		}
}


/**
  * @brief  Reset Encoder
  * @param  None
  * @retval None
  */




/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
