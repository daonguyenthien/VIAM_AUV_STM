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
#include "main.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_flash.h"
#include "my_io.h"
#include "my_can.h"
#include "my_delay.h"
#include "my_bms24v40ah.h"
#include "my_dvl.h"
#include "my_mx28.h"
#include "my_hardwarereset.h"
#include "my_keller_pa3.h"
#include "my_antiroll.h"
#include "UGV_SBUS.h"
#include "UGV_UART_JOYSTICK.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
uint8_t CAN_DataTest[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
uint16_t _test_Goal_MX28 = 0;
//UKellerPA3_Status_Typedef Status_Pressure_Sensor;

CAN_DataTypeDef CAN_Data;

uint8_t b = 0;
uint8_t c = 0;
uint8_t d = 0;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{ 
 /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files before to branch to application main.
       To reconfigure the default setting of SystemInit() function, 
       refer to system_stm32f4xx.c file */

	SystemClock_Config();
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  //SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

	//Wait System Stablelize
	UDELAY_Configuration();
//	UDELAY_ms(5000);
	
	UIO_Configuration();
	UDELAY_ms(500);
	
	UCAN_Configure();
	UDELAY_ms(500);
	
	USART6_DMA_Rx_Config();
	UDELAY_ms(500);
	
//	UANRO_Config();
//	UDELAY_ms(500);
//  UCAN_Configure();
//	UDELAY_ms(500);

//	UKELLER_Configuration();	
//	UDELAY_ms(100);
//	UKELLER_KELLER_Init(250);
//	UDELAY_ms(500);

//	UBMS40_Configuration();
//	UDELAY_ms(500);
//	UBMS40_Cmd(ENABLE);
//	UDELAY_ms(100);

//	UDVL_Configuration();

	UMX28_Configuration();
	UDELAY_ms(500);	
	UMX28_Init();
	UDELAY_ms(500);
	UMX28_pingServo(254);

//	UDELAY_ms(5000);
//	UMX28_setGoalPosition(1, 4095);
//	UHRST_Configuration();
	
//  UCAN_PassSystemReady(ENABLE);
//	UIO_LEDORANGE_ON();
//	UDELAY_ms(1000);
	
//	uint8_t data[1] = {0x29};
//	UCAN_Transmit(0X25, 1, data);
	

	
//	CalcCRC16(buff, 3, &a1, &a2);
  /* Infinite loop */
	
//	Close_Thruster();
//	Open_Piston();
//	Open_Shifter();
	Flag.Send_Data = true;
	Flag.Joystick_Disable = true;
	scalechannels[7] = 0;
	scalechannels[5] = 1;
//	GPIO_SetBits(GPIOC, GPIO_Pin_10);
	UIO_LEDORANGE_ON();
	UDELAY_ms(1000);
	UIO_LEDORANGE_OFF();
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	UMX28_setGoalPosition(254,0);
//	Run_Thruster(20);
  while (1)
  {
		b = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11);
		c = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12);
		d = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13);
		
		if(b==1 || c==1 || d==1)
		{
			UIO_LEDORANGE_ON();
		}
//		UMX28_setGoalPosition(254,250);
//		UDELAY_ms(20000);
//		GPIO_ResetBits(GPIOC, GPIO_Pin_10);
//		GPIO_SetBits(GPIOC, GPIO_Pin_10);
//		UDELAY_ms(1000);
//		GPIO_ResetBits(GPIOC, GPIO_Pin_10);
//		UDELAY_ms(1000);
		
//		Run_Thruster(20);
//		UIO_LEDORANGE_TOGGLE();
//		UDELAY_ms(1000);
//		UMX28_setGoalPosition(254,rudder_angle);
//		Close_Thruster();
//		Open_Piston();
//		Open_Shifter();
//		UIO_LEDORANGE_TOGGLE();
//		UDELAY_ms(1000);
  }
	return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  // Resets the clock configuration to the default reset state
    RCC_DeInit();

    // Enable external crystal (HSE)
    RCC_HSEConfig(RCC_HSE_ON);
    // Wait until HSE ready to use or not
    ErrorStatus errorStatus = RCC_WaitForHSEStartUp();

    if (errorStatus == SUCCESS)
    {
        // Configure the PLL for 168MHz SysClk and 48MHz for USB OTG, SDIO
        RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
        // Enable PLL
        RCC_PLLCmd(ENABLE);
        // Wait until main PLL clock ready
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        // Set flash latency
        FLASH_SetLatency(FLASH_Latency_5);

        // AHB 168MHz
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        // APB1 42MHz
        RCC_PCLK1Config(RCC_HCLK_Div4);
        // APB2 84 MHz
        RCC_PCLK2Config(RCC_HCLK_Div2);

        // Set SysClk using PLL
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    }
    else
    {
        // Do something to indicate that error clock configuration
        while (1);
    }

    SystemCoreClockUpdate();
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
