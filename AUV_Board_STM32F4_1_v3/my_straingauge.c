/* Includes ------------------------------------------------------------------*/
#include "my_straingauge.h"

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   STRAIN-GAUGE Pin define 
		*/
			#define   USTR_ADC_NbrOfConversion	30	
			#define		USTR_ADC_SampleTime				ADC_SampleTime_144Cycles
			#define 	USTR_ADC									ADC1
			#define   USTR_ADC_CLK							RCC_APB2Periph_ADC1
			#define		USTR_ADC_CLK_Cmd					RCC_APB2PeriphClockCmd
			
			// Define Priority ADC Channel
			#define		USTR_ADC1_PRIO						1
			#define		USTR_ADC2_PRIO						2
			#define		USTR_ADC3_PRIO						3
			#define		USTR_ADC4_PRIO						4
			#define		USTR_ADC5_PRIO						5
			#define		USTR_ADC6_PRIO						6
			#define		USTR_ADC7_PRIO						7
			#define		USTR_ADC8_PRIO						8
			
			#define 	USTR_ADC1_PIN  				   	GPIO_Pin_0			
			#define 	USTR_ADC1_PORT 				  	GPIOA
			#define 	USTR_ADC1_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC1_SOURCE				 	GPIO_PinSource0				
			#define		USTR_ADC1_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd		
			#define   USTR_ADC1_CHANNEL					ADC_Channel_0
			
			#define 	USTR_ADC2_PIN  				   	GPIO_Pin_1			
			#define 	USTR_ADC2_PORT 				  	GPIOA
			#define 	USTR_ADC2_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC2_SOURCE				 	GPIO_PinSource1				
			#define		USTR_ADC2_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			#define   USTR_ADC2_CHANNEL					ADC_Channel_1
			
			#define 	USTR_ADC3_PIN  				   	GPIO_Pin_2			
			#define 	USTR_ADC3_PORT 				  	GPIOA
			#define 	USTR_ADC3_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC3_SOURCE				 	GPIO_PinSource2				
			#define		USTR_ADC3_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			#define   USTR_ADC3_CHANNEL					ADC_Channel_2
			
			#define 	USTR_ADC4_PIN  				   	GPIO_Pin_3			
			#define 	USTR_ADC4_PORT 				  	GPIOA
			#define 	USTR_ADC4_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC4_SOURCE				 	GPIO_PinSource3				
			#define		USTR_ADC4_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			
			#define   USTR_ADC4_CHANNEL					ADC_Channel_3
			
			#define 	USTR_ADC5_PIN  				   	GPIO_Pin_4			
			#define 	USTR_ADC5_PORT 				  	GPIOA
			#define 	USTR_ADC5_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC5_SOURCE				 	GPIO_PinSource4				
			#define		USTR_ADC5_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			#define   USTR_ADC5_CHANNEL					ADC_Channel_4
			
			#define 	USTR_ADC6_PIN  				   	GPIO_Pin_5			
			#define 	USTR_ADC6_PORT 				  	GPIOA
			#define 	USTR_ADC6_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC6_SOURCE				 	GPIO_PinSource5				
			#define		USTR_ADC6_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			#define   USTR_ADC6_CHANNEL					ADC_Channel_5
			
			#define 	USTR_ADC7_PIN  				   	GPIO_Pin_6			
			#define 	USTR_ADC7_PORT 				  	GPIOA
			#define 	USTR_ADC7_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC7_SOURCE				 	GPIO_PinSource6				
			#define		USTR_ADC7_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			#define   USTR_ADC7_CHANNEL					ADC_Channel_6
			
			#define 	USTR_ADC8_PIN  				   	GPIO_Pin_7			
			#define 	USTR_ADC8_PORT 				  	GPIOA
			#define 	USTR_ADC8_CLK							RCC_AHB1Periph_GPIOA			
			#define 	USTR_ADC8_SOURCE				 	GPIO_PinSource7				
			#define		USTR_ADC8_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			#define   USTR_ADC8_CHANNEL					ADC_Channel_7
			
			#define   USTR_DMA_Channel					DMA_Channel_0
			#define   USTR_DMA_Stream						DMA2_Stream0
			#define		USTR_DMA_CLK_Cmd		  		RCC_AHB1PeriphClockCmd
			#define 	USTR_DMA_CLK							RCC_AHB1Periph_DMA2
			#define		USTR_DMA_Stream_IRQn			DMA2_Stream0_IRQn
			#define		USTR_DMA_PreemptionPriority			0
			#define		USTR_DMA_SubPriority						1
			
			#define		USTR_DMA_Stream_IRQHandler			DMA2_Stream0_IRQHandler
			

/* Private variables ---------------------------------------------------------*/
static uint16_t ADCxConvertedValue[USTR_ADC_NbrOfConversion];
static Strain_Gauge_StatusTypeDef _strain_gauge_status;

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/
void USTR_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	
	USTR_ADC1_CLK_Cmd(USTR_ADC1_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = USTR_ADC1_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(USTR_ADC1_PORT,&GPIO_InitStruct);
	
	USTR_ADC2_CLK_Cmd(USTR_ADC2_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = USTR_ADC2_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(USTR_ADC2_PORT,&GPIO_InitStruct);
	
	USTR_ADC3_CLK_Cmd(USTR_ADC3_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = USTR_ADC3_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(USTR_ADC3_PORT,&GPIO_InitStruct);
	
//	USTR_ADC4_CLK_Cmd(USTR_ADC4_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = USTR_ADC4_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(USTR_ADC4_PORT,&GPIO_InitStruct);
//	
//	USTR_ADC5_CLK_Cmd(USTR_ADC5_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = USTR_ADC5_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(USTR_ADC5_PORT,&GPIO_InitStruct);
//	
//	USTR_ADC6_CLK_Cmd(USTR_ADC6_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = USTR_ADC6_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(USTR_ADC6_PORT,&GPIO_InitStruct);
//	
//	USTR_ADC7_CLK_Cmd(USTR_ADC7_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = USTR_ADC7_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(USTR_ADC7_PORT,&GPIO_InitStruct);
//	
//	USTR_ADC8_CLK_Cmd(USTR_ADC8_CLK,ENABLE);	
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = USTR_ADC8_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(USTR_ADC8_PORT,&GPIO_InitStruct);
	
	/* ADC Common Init **********************************************************/
	USTR_ADC_CLK_Cmd(USTR_ADC_CLK, ENABLE);
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);
	
	/* ADC1 Init ****************************************************************/
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE; // Multiple Channels
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; // Keep Sampling
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = USTR_ADC_NbrOfConversion/10;
	ADC_Init(USTR_ADC, &ADC_InitStruct);
	
  /* DMA2 Stream0 channel0 configuration **************************************/
	USTR_DMA_CLK_Cmd(USTR_DMA_CLK,ENABLE);	
  DMA_InitStruct.DMA_Channel = USTR_DMA_Channel;  
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1 -> DR));
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADCxConvertedValue;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = USTR_ADC_NbrOfConversion;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(USTR_DMA_Stream, &DMA_InitStruct);
	
	/* DMA Transfer complete interrupt configure */
	DMA_ITConfig(USTR_DMA_Stream, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = USTR_DMA_Stream_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = USTR_DMA_PreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = USTR_DMA_SubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	DMA_Cmd(USTR_DMA_Stream, ENABLE);	
	/* ADC1 regular channel configuration **************************************/
  ADC_RegularChannelConfig(USTR_ADC, USTR_ADC1_CHANNEL, USTR_ADC1_PRIO, USTR_ADC_SampleTime);
	ADC_RegularChannelConfig(USTR_ADC, USTR_ADC2_CHANNEL, USTR_ADC2_PRIO, USTR_ADC_SampleTime);
	ADC_RegularChannelConfig(USTR_ADC, USTR_ADC3_CHANNEL, USTR_ADC3_PRIO, USTR_ADC_SampleTime);
//	ADC_RegularChannelConfig(USTR_ADC, USTR_ADC4_CHANNEL, USTR_ADC4_PRIO, USTR_ADC_SampleTime);
//	ADC_RegularChannelConfig(USTR_ADC, USTR_ADC5_CHANNEL, USTR_ADC5_PRIO, USTR_ADC_SampleTime);
//	ADC_RegularChannelConfig(USTR_ADC, USTR_ADC6_CHANNEL, USTR_ADC6_PRIO, USTR_ADC_SampleTime);
//	ADC_RegularChannelConfig(USTR_ADC, USTR_ADC7_CHANNEL, USTR_ADC7_PRIO, USTR_ADC_SampleTime);
//	ADC_RegularChannelConfig(USTR_ADC, USTR_ADC8_CHANNEL, USTR_ADC8_PRIO, USTR_ADC_SampleTime);
	
 /* Enable DMA request after last transfer */
  ADC_DMARequestAfterLastTransferCmd(USTR_ADC, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(USTR_ADC, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(USTR_ADC, ENABLE);
	
	ADC_SoftwareStartConv(USTR_ADC);
}

void USTR_GetStatus(Strain_Gauge_StatusTypeDef *_strain_gauge_st)
{
	_strain_gauge_st->Strain_Gauge_P1 = _strain_gauge_status.Strain_Gauge_P1;
	_strain_gauge_st->Strain_Gauge_P2 = _strain_gauge_status.Strain_Gauge_P2;
	_strain_gauge_st->Strain_Gauge_P3 = _strain_gauge_status.Strain_Gauge_P3;
//	_strain_gauge_st->Strain_Gauge_P4 = _strain_gauge_status.Strain_Gauge_P4;
//	_strain_gauge_st->Strain_Gauge_P5 = _strain_gauge_status.Strain_Gauge_P5;
//	_strain_gauge_st->Strain_Gauge_P6 = _strain_gauge_status.Strain_Gauge_P6;
//	_strain_gauge_st->Strain_Gauge_P7 = _strain_gauge_status.Strain_Gauge_P7;
//	_strain_gauge_st->Strain_Gauge_P8 = _strain_gauge_status.Strain_Gauge_P8;
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void USTR_DMA_Stream_IRQHandler(void)
{	
	if(DMA_GetITStatus(USTR_DMA_Stream, DMA_IT_TCIF0) != RESET)
	{	
		uint8_t i = 10;
		float temp_ST1 = 0.0f, temp_ST2 = 0.0f, temp_ST3 = 0.0f;/*, temp_ST4 = 0.0f,
					temp_ST5 = 0.0f, temp_ST6 = 0.0f, temp_ST7 = 0.0f, temp_ST8 = 0.0f;*/
		
		for(i = 0; i < (USTR_ADC_NbrOfConversion/3); i++)
		{
			temp_ST1 += ADCxConvertedValue[3*i];
			temp_ST2 += ADCxConvertedValue[3*i + 1];
			temp_ST3 += ADCxConvertedValue[3*i + 2];
//			temp_ST4 += ADCxConvertedValue[8*i + 3];
//			temp_ST5 += ADCxConvertedValue[8*i + 4];
//			temp_ST6 += ADCxConvertedValue[8*i + 5];
//			temp_ST7 += ADCxConvertedValue[8*i + 6];
//			temp_ST8 += ADCxConvertedValue[8*i + 7];
		}
		
		_strain_gauge_status.Strain_Gauge_P1 = temp_ST1/(USTR_ADC_NbrOfConversion/8);
		_strain_gauge_status.Strain_Gauge_P2 = temp_ST2/(USTR_ADC_NbrOfConversion/8);
		_strain_gauge_status.Strain_Gauge_P3 = temp_ST3/(USTR_ADC_NbrOfConversion/8);
//		_strain_gauge_status.Strain_Gauge_P4 = temp_ST4/(USTR_ADC_NbrOfConversion/8);
//		_strain_gauge_status.Strain_Gauge_P5 = temp_ST5/(USTR_ADC_NbrOfConversion/8);
//		_strain_gauge_status.Strain_Gauge_P6 = temp_ST6/(USTR_ADC_NbrOfConversion/8);
//		_strain_gauge_status.Strain_Gauge_P7 = temp_ST7/(USTR_ADC_NbrOfConversion/8);
//		_strain_gauge_status.Strain_Gauge_P8 = temp_ST8/(USTR_ADC_NbrOfConversion/8);
		
		DMA_ClearITPendingBit(USTR_DMA_Stream,DMA_IT_DMEIF0|DMA_IT_FEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0|DMA_IT_TEIF0);
	}
}

