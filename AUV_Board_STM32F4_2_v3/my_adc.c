#include "my_adc.h"

#define UADC_GPIO_PORT			GPIOA
#define UADC_GPIO_Pin 			GPIO_Pin_0						//Roll-Hall-B
#define UADC_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define UADC_GPIO_CLK_Cmd		RCC_AHB1PeriphClockCmd

#define UADC								ADC1
#define UADC_Channel				ADC_Channel_0
#define UADC_CLK 						RCC_APB2Periph_ADC1
#define UADC_CLK_Cmd				RCC_APB2PeriphClockCmd

#define UADC_DMA																	DMA2
#define UADC_DMA_Channel													DMA_Channel_0
#define UADC_DMA_Stream														DMA2_Stream0
#define UADC_DMA_Stream_IRQn											DMA2_Stream0_IRQn
#define UADC_DMA_CLK															RCC_AHB1Periph_DMA2
#define UADC_DMA_CLK_Cmd													RCC_AHB1PeriphClockCmd
#define UADC_DMA_IRQPreemptionPriority						0x00
#define UADC_DMA_IRQSubPriority										0x06
#define	USARTx_JOYSTICK_DMA_Stream_IRQHandler			DMA2_Stream0_IRQHandler

volatile uint32_t xsen_angle = 0;

void ADC_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_Initstructure;
	DMA_InitTypeDef DMA_Initstructure;
	NVIC_InitTypeDef 	NVIC_InitStruct;
	
	UADC_GPIO_CLK_Cmd(UADC_GPIO_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = UADC_GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UADC_GPIO_PORT, &GPIO_InitStructure);
	
	UADC_DMA_CLK_Cmd(UADC_DMA_CLK,ENABLE);
	DMA_Initstructure.DMA_Channel = UADC_DMA_Channel;
	DMA_Initstructure.DMA_Memory0BaseAddr = (uint32_t)&xsen_angle;
	DMA_Initstructure.DMA_PeripheralBaseAddr = (uint32_t)(&(UADC->DR));
	DMA_Initstructure.DMA_DIR = DMA_DIR_PeripheralToMemory;									//peripheral to memory
	DMA_Initstructure.DMA_BufferSize = 1;
	DMA_Initstructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Initstructure.DMA_MemoryInc = DMA_PeripheralInc_Disable;
	DMA_Initstructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_Initstructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_Initstructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Initstructure.DMA_Priority = DMA_Priority_Medium;
	DMA_Initstructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Initstructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_Initstructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_Initstructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(UADC_DMA_Stream, &DMA_Initstructure);
	DMA_Cmd(UADC_DMA_Stream, ENABLE);
	
//	NVIC_InitStruct.NVIC_IRQChannel = UADC_DMA_Stream_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UADC_DMA_IRQPreemptionPriority;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = UADC_DMA_IRQSubPriority;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	DMA_ITConfig(UADC_DMA_Stream, DMA_IT_TC, ENABLE);
	
	UADC_CLK_Cmd(UADC_CLK,ENABLE);
	ADC_Initstructure.ADC_ContinuousConvMode = ENABLE;
	ADC_Initstructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_Initstructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; 
	ADC_Initstructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_Initstructure.ADC_NbrOfConversion = 1;
	ADC_Initstructure.ADC_ScanConvMode = ENABLE;
	ADC_Initstructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(UADC, &ADC_Initstructure);
	
	ADC_RegularChannelConfig(UADC,UADC_Channel,1,ADC_SampleTime_480Cycles);
	ADC_Cmd(UADC, ENABLE);
	ADC_DMACmd(UADC, ENABLE);
	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_SoftwareStartConv(UADC);
}

void USARTx_JOYSTICK_DMA_Stream_IRQHandler(void)
{
	if(DMA_GetITStatus(UADC_DMA_Stream, DMA_IT_TCIF1) != RESET)
	{
	}
	DMA_ClearITPendingBit(UADC_DMA_Stream,DMA_IT_DMEIF1|DMA_IT_FEIF1|DMA_IT_HTIF1|DMA_IT_TCIF1|DMA_IT_TEIF1);
}
