/**
  ******************************************************************************
  * @file    AUV_Driver_BLDC\BLDC_Driver_ARM_v3\main.c 
  * @author  Hai Chau Thanh
	* @mailbox thanhhaipif96@gmail.com
	* @company viamlab
  * @version ARM_V3.0.0
  * @date    01-March-2019
  * @brief   Main program body.
*/ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "User_PWM.h"
#include "User_HALLSensor.h"
#include "User_CAN.h"
#include "User_ADC.h"
#include "User_ResetHardfault.h"
#include "User_Algorithm.h"
#include "User_Delay.h"
#include "User_LED.h"
#include "User_Brake.h"
#include "User_Encoder.h"

/* Private typedef -----------------------------------------------------------*/
RCC_ClocksTypeDef 				RCC_ClockFreq;
ErrorStatus 							HSEStartUpStatus;

void Clock_HSE_Configuration(uint32_t clock_in);
/* Private define ------------------------------------------------------------*/
#define clock_16Mhz  0
#define clock_24Mhz  0x00040000
#define clock_32Mhz  0x00080000
#define clock_40Mhz  0x000C0000
#define clock_48Mhz  0x00100000
#define clock_56Mhz  0x00140000
#define clock_64Mhz  0x00180000
#define clock_72Mhz  0x001C0000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t TimingDelay = 0;
float A,B,C;
float PWM;
uint16_t pHall;
uint8_t CAN_Data[8];
uint32_t clock_source = 0;
uint8_t CAN_count = 0;
uint32_t counter, pulse;
float __position = 50.0f;
float Kp = 0.005, Ki = 0.0f, Kd = 0.001;
float posi = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Delay(__IO uint32_t nTime);
int main(void)
{ 
	Clock_HSE_Configuration(clock_72Mhz);
	RCC_GetClocksFreq(&RCC_ClockFreq);			
	clock_source = RCC_ClockFreq.SYSCLK_Frequency; 
	
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 us
//	SysTick_Config(SystemCoreClock / 1000000);
	UDELAY_Configure();
	ULED_Configure();
	UHALL_GPIO_Configure();
	UPWM_GPIO_Configure();
	UADC_GPIO_Configure();
	UPWM_TIM_Configure();
//	UBRAKE_Configuration();
	UDELAY_ms(500);

	UCAN_GPIO_Configure();
	UHALL_TIM_Configure();
	UDELAY_ms(500);

	UENC_Configure();
	UCAN_CAN_Configure();
	UDELAY_ms(500);	
	UADC_ADC_DMA_Configure();
	UPWM_StopPWM();

	UDELAY_ms(2000);
//	//---------------------------------Initial----------------------------------//
	UALTHM_UpdatePIDParams(Kp, Ki , Kd);
	UALTHM_UpdateFuzzyParams(1000, 5, 0.01);
	//pulse = UENCmm2pulse(__position);
	UALTHM_UpdateParameters(Update_SetPosition,0,0,0,0);
	
	UCAN_PassSystemReady(ENABLE);
	CAN_Data[0] = 'R';
	CAN_Data[1] = 'E';
	CAN_Data[2] = 'Q';
	CAN_Data[3] = 'A';
	CAN_Data[4] = 'L';
	CAN_Data[5] = 'L';
	CAN_Data[6] = 0x0A;
	CAN_Data[7] = UCAN_Checksum(CAN_Data);
	
	CAN_Data[0] = 'C';
	CAN_Data[1] = 'A';
	CAN_Data[2] = 'N';
	CAN_Data[3] = 'O';
	CAN_Data[4] = 0x00;
	CAN_Data[5] = 0x00;
	CAN_Data[6] = 0x00;
	CAN_Data[7] = UCAN_Checksum(CAN_Data);

//	UALTHM_UpdatePIDParams(1, 0, 0);
	
	// Test
	// CW: from Pump
//UCAN_Test_2();
// UCAN_Test();
 
//Test mode in UCAN_Test();

	ULED_Toggle(ORANGE);
	UDELAY_ms(1000);
	ULED_Toggle(ORANGE);
//	UCAN_Test();
//	UCAN_Test_2();
////	UCAN_Test();
	
  while (1)
  {
//		UCAN_Thruster();
//		UCAN_Rudder();
//		UCAN_Mass();
//		UCAN_Pistol();
//		UCAN_Altimeter();
//		UCAN_Leaksensor();
		
//		UCAN_SendPosition();
//		UDELAY_ms(100);
		
//		posi = UENC_GetActualPosition();
//		ULED_Toggle(ORANGE);
//		ULED_Toggle(GREEN);
		
//		UCAN_Test();
//		UCAN_Test_2();
		
		
//		if(flag_run)
//		{
//			UCAN_Test_2();
//			UCAN_Test();
//			flag_run = 0;
//		}

//		UCAN_Test();
//		UALTHM_UpdatePIDParams(Kp, Ki, Kd);
//		UALTHM_UpdateSetPosition(__position);
//		counter = UENC_GetCounterValue();
//		pulse = UENCmm2pulse(__position);

//		pHall = UHALL_GetPosition();
		//CAN_Data[1] = CAN_count;
//		CAN_Data[0] = CAN_count;
//		CAN_Data[1] = CAN_count + 1;
//		CAN_Data[2] = CAN_count + 2;
//		CAN_Data[3] = CAN_count + 3;
//		CAN_Data[4] = CAN_count + 4;
//		CAN_Data[5] = CAN_count + 5;
//		CAN_Data[6] = CAN_count + 6;
//		CAN_Data[7] = CAN_count + 7;
//		UCAN_Transmit(CAN1,0x123,8,CAN_Data);
//		CAN_count++;
		
		//UDELAY_us(10);
		//CAN_count++;
//		UHALL_ReadPosition(Motor_CW);
//		UPWM_SetBytesDutyCycle(0x00, 0x00, 0xA0, 0x41);
//		PWM = UHALL_GetActualSpeed();
//		A = UADC_GetValue(Temp_On_Chip);
//		B = UADC_GetValue(Temp_LM35);
//		C = UADC_GetValue(iMotor);
//		UDELAY_ms(500);

//		if(UCAN_NeedRespondData() == true)
//		{
//			UCAN_Respond_ALLData();
//			
//			UCAN_AllowRespondData(DISABLE);
//		}
//		
//		if(UCAN_IsNeedCheckSystem())
//		{ 
//			if(UCAN_IsSystemReady()) UCAN_SystemReady();
//			else UCAN_SystemNotReady();
//			UCAN_AllowRespondCheckSystem(DISABLE);
//		}

//	if (UHALL_GetTimerEvent() == true){
//		if(UCAN_GetFlag(Flag_PID_Control)||UCAN_GetFlag(Flag_Fuzzy_Control))
//			{
//				if (!UCAN_GetFlag(Flag_Motor_Position)){
//					if(UCAN_GetFlag(Flag_Motor_Run_CW))
//					{
//						UALTHM_Controller(Motor_CW);
//					}
//					else if(UCAN_GetFlag(Flag_Motor_Run_CCW))
//					{
//						UALTHM_Controller(Motor_CCW);
//					}
//				}
//				else UALTHM_Controller(Motor_CCW);
//			}
//		}
//	UHALL_ResetTimerEvent();
	}
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;
  while(TimingDelay != 0)
  {
  }
}

void Clock_HSE_Configuration(uint32_t clock_in)
{
  RCC_DeInit(); 														
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);									
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    /* PLLCLK = clock_in */
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1, clock_in);	
    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08){ }
  }
  else{ while (1){}}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
