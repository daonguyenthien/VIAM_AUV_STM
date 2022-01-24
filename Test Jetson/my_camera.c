/* Includes ------------------------------------------------------------------*/
#include "my_camera.h"

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static uint8_t USART_DataTransmit[9];
/* Private const/macros ------------------------------------------------------*/
static void UCAM_TransmitData(uint8_t *_TX_Buffer,int _Length);
/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   COM Pin define 
		*/
			#define 	UCAM_TX_PIN  				   	GPIO_Pin_10			
			#define 	UCAM_TX_PORT 				  	GPIOC
			#define 	UCAM_TX_CLK							RCC_AHB1Periph_GPIOC			
			#define 	UCAM_TX_SOURCE				 	GPIO_PinSource10				
			#define		UCAM_TX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

			#define 	UCAM_RX_PIN  				   	GPIO_Pin_11
			#define 	UCAM_RX_PORT 				  	GPIOC
			#define 	UCAM_RX_CLK							RCC_AHB1Periph_GPIOC
			#define 	UCAM_RX_SOURCE				 	GPIO_PinSource11
			#define		UCAM_RX_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd
			
			#define		UCAM_USART							UART4
			#define 	UCAM_USART_CLK					RCC_APB1Periph_UART4
			#define		UCAM_USART_CLK_Cmd		  RCC_APB1PeriphClockCmd
			#define 	UCAM_USART_BAUDRATE			9600
			#define		UCAM_USART_AF						GPIO_AF_UART4
			#define 	UCAM_USART_IRQn					UART4_IRQn
			
			#define		UCAM_UART_IRQHandler		UART4_IRQHandler
			
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported function body ----------------------------------------------------*/
void UCAM_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	UCAM_TX_CLK_Cmd(UCAM_TX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UCAM_TX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UCAM_TX_PORT,&GPIO_InitStruct);
	
	UCAM_RX_CLK_Cmd(UCAM_RX_CLK,ENABLE);	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = UCAM_RX_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(UCAM_RX_PORT,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(UCAM_TX_PORT,UCAM_TX_SOURCE,UCAM_USART_AF);
	GPIO_PinAFConfig(UCAM_RX_PORT,UCAM_RX_SOURCE,UCAM_USART_AF);
	
	UCAM_USART_CLK_Cmd(UCAM_USART_CLK,ENABLE);
	USART_InitStruct.USART_BaudRate = UCAM_USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_0_5;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(UCAM_USART,&USART_InitStruct);
	
	USART_Cmd(UCAM_USART,ENABLE);	
}

void UCAM_AddressSet(uint8_t _id)
{
	USART_DataTransmit[0] = 0x81;
	USART_DataTransmit[1] = 0x30;
	USART_DataTransmit[2] = _id;
	USART_DataTransmit[3] = 0xFF;
	UCAM_TransmitData(USART_DataTransmit,4);
}

void UCAM_IF_Clear()
{
	USART_DataTransmit[0] = 0x81;
	USART_DataTransmit[1] = 0x01;
	USART_DataTransmit[2] = 0x00;
	USART_DataTransmit[3] = 0x01;
	USART_DataTransmit[4] = 0xFF;
	UCAM_TransmitData(USART_DataTransmit,5);
}

void UCAM_Power(uint8_t _state)
{
	USART_DataTransmit[0] = 0x81;
	USART_DataTransmit[1] = 0x01;
	USART_DataTransmit[2] = 0x04;
	USART_DataTransmit[3] = 0x00;
	USART_DataTransmit[4] = _state;
	USART_DataTransmit[5] = 0xFF;
	UCAM_TransmitData(USART_DataTransmit,6);
}

void UCAM_Zoom(uint8_t _standard)
{
	USART_DataTransmit[0] = 0x81;
	USART_DataTransmit[1] = 0x01;
	USART_DataTransmit[2] = 0x04;
	USART_DataTransmit[3] = 0x07;
	USART_DataTransmit[4] = _standard;
	USART_DataTransmit[5] = 0xFF;
	UCAM_TransmitData(USART_DataTransmit,6);
}

void UCAM_Bright(uint8_t _bright)
{
	USART_DataTransmit[0] = 0x81;
	USART_DataTransmit[1] = 0x01;
	USART_DataTransmit[2] = 0x04;
	USART_DataTransmit[3] = 0x0D;
	USART_DataTransmit[4] = _bright;
	USART_DataTransmit[5] = 0xFF;
	UCAM_TransmitData(USART_DataTransmit,6);
}

void UCAM_Focus_Mode(uint8_t _mode)
{
	USART_DataTransmit[0] = 0x81;
	USART_DataTransmit[1] = 0x01;
	USART_DataTransmit[2] = 0x04;
	USART_DataTransmit[3] = 0x38;
	USART_DataTransmit[4] = _mode;
	USART_DataTransmit[5] = 0xFF;
	UCAM_TransmitData(USART_DataTransmit,6);
}

void UCAM_Focus_SetUp(uint8_t _limit)
{
	USART_DataTransmit[0] = 0x81;
	USART_DataTransmit[1] = 0x01;
	USART_DataTransmit[2] = 0x04;
	USART_DataTransmit[3] = 0x08;
	USART_DataTransmit[4] = _limit;
	USART_DataTransmit[5] = 0xFF;
	UCAM_TransmitData(USART_DataTransmit,6);
}

void UCAM_TransmitData(uint8_t *_TX_Buffer,int _Length)
{
	for(int i = 0; i < _Length; i++)
	{
		while(USART_GetFlagStatus(UCAM_USART,USART_FLAG_TXE) != SET);
		USART_SendData(UCAM_USART,_TX_Buffer[i]);
	}
}

/* Private functions body ----------------------------------------------------*/

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
