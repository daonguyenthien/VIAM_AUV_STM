/*------------------------------------------------------------------------*/
/* STM32F100: MMCv3/SDv1/SDv2 (SPI mode) control module                   */
/*------------------------------------------------------------------------*/
/*
/  Copyright (C) 2018, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

#include "ff.h"
#include "diskio.h"
#include "stm32f4xx.h"
#include "my_delay.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

/* Configure Hardware*/
//Timer
#define 	UFATFS_TIM  				      		TIM7
#define 	UFATFS_TIM_CLK					 	 		RCC_APB1Periph_TIM7	
#define 	UFATFS_TIM_CLK_Cmd    				RCC_APB1PeriphClockCmd
#define   UFATFS_TIM_IRQn								TIM7_IRQn

#define 	UFATFS_TIM_IRQHandler					TIM7_IRQHandler

#define 	UFATFS_CD_PIN  				   	GPIO_Pin_4			
#define 	UFATFS_CD_PORT 				  	GPIOC
#define 	UFATFS_CD_CLK							RCC_AHB1Periph_GPIOC			
#define 	UFATFS_CD_SOURCE				 	GPIO_PinSource4				
#define		UFATFS_CD_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd	

//SPI
#define 	UFATFS_NSS_PIN  				   	GPIO_Pin_4			
#define 	UFATFS_NSS_PORT 				  	GPIOA
#define 	UFATFS_NSS_CLK							RCC_AHB1Periph_GPIOA			
#define 	UFATFS_NSS_SOURCE				 		GPIO_PinSource4				
#define		UFATFS_NSS_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd			

#define 	UFATFS_SCK_PIN  				   	GPIO_Pin_5
#define 	UFATFS_SCK_PORT 				  	GPIOA
#define 	UFATFS_SCK_CLK							RCC_AHB1Periph_GPIOA
#define 	UFATFS_SCK_SOURCE				 		GPIO_PinSource5
#define		UFATFS_SCK_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd

#define 	UFATFS_MISO_PIN  				   	GPIO_Pin_6
#define 	UFATFS_MISO_PORT 				  	GPIOA
#define 	UFATFS_MISO_CLK							RCC_AHB1Periph_GPIOA
#define 	UFATFS_MISO_SOURCE				 	GPIO_PinSource6
#define		UFATFS_MISO_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd

#define 	UFATFS_MOSI_PIN  				   	GPIO_Pin_7
#define 	UFATFS_MOSI_PORT 				  	GPIOA
#define 	UFATFS_MOSI_CLK							RCC_AHB1Periph_GPIOA
#define 	UFATFS_MOSI_SOURCE				 	GPIO_PinSource7
#define		UFATFS_MOSI_CLK_Cmd		  	 	RCC_AHB1PeriphClockCmd

#define		UFATFS_SPI							SPI1
#define 	UFATFS_SPI_CLK					RCC_APB2Periph_SPI1
#define		UFATFS_SPI_CLK_Cmd		  RCC_APB2PeriphClockCmd
#define		UFATFS_SPI_AF						GPIO_AF_SPI1
#define 	UFATFS_SPI_IRQn					SPI1_IRQn

#define		UFATFS_SPI_IRQHandler		SPI1_IRQHandler

/* MMC/SD command */
#define	MMC_CD		!(GPIO_ReadInputDataBit(UFATFS_CD_PORT,UFATFS_CD_PIN))	/* Card detect (yes:true, no:false, default:true) */
#define	MMC_WP		0 /* Write protected (yes:true, no:false, default:false) */

#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

#define CS_HIGH()	GPIO_SetBits(UFATFS_NSS_PORT, UFATFS_NSS_PIN)
#define CS_LOW()	GPIO_ResetBits(UFATFS_NSS_PORT, UFATFS_NSS_PIN)

/* Card type flags (CardType) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		0x06		/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

static volatile
DSTATUS Stat = STA_NOINIT;	/* Physical drive status */

static volatile
UINT Timer1, Timer2;	/* 1kHz decrement timer stopped at zero (disk_timerproc()) */

static
BYTE CardType;			/* Card type flags */

static SPI_InitTypeDef SPI_InitStruct;

/* Mode of SPI */
typedef enum {
	MODE_8BIT = 0,
	MODE_16BIT
} UFATFS_SPI_ModeBits;

/*-----------------------------------------------------------------------*/
/* SPI controls (Platform dependent)                                     */
/*-----------------------------------------------------------------------*/

/*---------------------------------------------------------*/
/* User provided RTC function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called back     */
/* from FatFs module.                                      */

#if !FF_FS_NORTC && !FF_FS_READONLY
DWORD get_fattime (void)
{
	RTCTIME rtc;

	/* Get local time */
	if (!rtc_gettime(&rtc)) return 0;

	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(rtc.year - 1980) << 25)
			| ((DWORD)rtc.month << 21)
			| ((DWORD)rtc.mday << 16)
			| ((DWORD)rtc.hour << 11)
			| ((DWORD)rtc.min << 5)
			| ((DWORD)rtc.sec >> 1);
}
#endif

static void FCLK_SLOW() 
{ 
	SPI_Cmd(UFATFS_SPI, DISABLE); 
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_Init(UFATFS_SPI, &SPI_InitStruct); 
	SPI_Cmd(UFATFS_SPI, ENABLE); 	
}	/* Set SCLK = PCLK / 64 */

static void FCLK_FAST() 
{ 
	SPI_Cmd(UFATFS_SPI, DISABLE); 
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_Init(UFATFS_SPI, &SPI_InitStruct); 
	SPI_Cmd(UFATFS_SPI, ENABLE); 	
}	/* Set SCLK = PCLK / 2 */

static void SPI_MODE8BITS() 
{ 
	SPI_Cmd(UFATFS_SPI, DISABLE); 
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_Init(UFATFS_SPI, &SPI_InitStruct); 
	SPI_Cmd(UFATFS_SPI, ENABLE); 
}

static void SPI_MODE16BITS() 
{ 
	SPI_Cmd(UFATFS_SPI, DISABLE); 
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_Init(UFATFS_SPI, &SPI_InitStruct); 
	SPI_Cmd(UFATFS_SPI, ENABLE); 
}

static
void timer_configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	UFATFS_TIM_CLK_Cmd(UFATFS_TIM_CLK, ENABLE);		
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_Prescaler = ((SystemCoreClock/4)/100000) * 2 - 1;     
  TIM_TimeBaseStruct.TIM_Period = 100 - 1;
	TIM_TimeBaseInit(UFATFS_TIM, &TIM_TimeBaseStruct);
	TIM_ITConfig(UFATFS_TIM,TIM_IT_Update,ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = UFATFS_TIM_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_Cmd(UFATFS_TIM, ENABLE);
}

static 
void gpio_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//Set up SPI port A (Structure in STM32f4 User_manual page 36)
	UFATFS_SCK_CLK_Cmd(UFATFS_SCK_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Pin = UFATFS_SCK_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;															//GPIO Alternate function Mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UFATFS_SCK_PORT, &GPIO_InitStruct);	
	GPIO_PinAFConfig(UFATFS_SCK_PORT, UFATFS_SCK_SOURCE, UFATFS_SPI_AF);
	
	UFATFS_MISO_CLK_Cmd(UFATFS_MISO_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Pin = UFATFS_MISO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;															//GPIO Alternate function Mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UFATFS_MISO_PORT, &GPIO_InitStruct);	
	GPIO_PinAFConfig(UFATFS_MISO_PORT, UFATFS_MISO_SOURCE, UFATFS_SPI_AF);
	
	UFATFS_MOSI_CLK_Cmd(UFATFS_MOSI_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Pin = UFATFS_MOSI_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;															//GPIO Alternate function Mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UFATFS_MOSI_PORT, &GPIO_InitStruct);
	GPIO_PinAFConfig(UFATFS_MOSI_PORT, UFATFS_MOSI_SOURCE, UFATFS_SPI_AF);
	
	//The SS(slave select) is PA4 (Structure in STM32f4 User_manual page 36)
	UFATFS_NSS_CLK_Cmd(UFATFS_NSS_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Pin = UFATFS_NSS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UFATFS_NSS_PORT, &GPIO_InitStruct);
	
	//The CD(card delect) is PC4
	UFATFS_CD_CLK_Cmd(UFATFS_CD_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Pin = UFATFS_CD_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UFATFS_CD_PORT, &GPIO_InitStruct);
	
	CS_HIGH();
}

static 
void spi_configuration(void)
{	
	SPI_DeInit(UFATFS_SPI);
	/* SPI_MASTER configuration ------------------------------------------------*/
	UFATFS_SPI_CLK_Cmd(UFATFS_SPI_CLK, ENABLE);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;																//Clock is high when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;															//Data is sampled at the second edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft|SPI_NSSInternalSoft_Set;					//Pick SS(slave select) port by software when want to receive from slave
	
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	
	
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(UFATFS_SPI, &SPI_InitStruct);
	
	SPI_Cmd(UFATFS_SPI, ENABLE);
}

/* Initialize MMC interface */
static
void init_spi (void)
{
	gpio_configuration();
	spi_configuration();		/* Enable SPI function */
	timer_configuration();
	CS_HIGH();			/* Set CS# high */

	UDELAY_ms(10);	/* 10ms */
}


/* Exchange a byte */
static
BYTE xchg_spi (
	BYTE dat	/* Data to send */
)
{
	SPI_I2S_SendData(UFATFS_SPI, dat);
	while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_TXE) == RESET);			//Wait for transmit. Transmit OK, turn on FLAG_TXE
	while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_RXNE) == RESET);			//Wait for receive. Receive OK, turn on FLAG_RXNE
	return SPI_I2S_ReceiveData(UFATFS_SPI);		/* Return received byte */
}


/* Receive multiple byte */
static
void rcvr_spi_multi (
	BYTE *buff,		/* Pointer to data buffer */
	UINT btr		/* Number of bytes to receive (even number) */
)
{
	WORD d;


	SPI_MODE16BITS(); /* Put SPI into 16-bit mode */

	SPI_I2S_SendData(UFATFS_SPI, 0xFFFF);		/* Start the first SPI transaction */
	btr -= 2;
	do {					/* Receive the data block into buffer */
		while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_TXE) == RESET);	/* Wait for end of the SPI transaction */
		while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_RXNE) == RESET);
		d = SPI_I2S_ReceiveData(UFATFS_SPI);						/* Get received word */
		SPI_I2S_SendData(UFATFS_SPI, 0xFFFF);					/* Start next transaction */
		buff[1] = d; buff[0] = d >> 8; 		/* Store received data */
		buff += 2;
	} while (btr -= 2);
	while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_TXE) == RESET);	/* Wait for end of the SPI transaction */
	while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	d = SPI_I2S_ReceiveData(UFATFS_SPI);							/* Get last word received */
	buff[1] = d; buff[0] = d >> 8;			/* Store it */

	SPI_MODE8BITS();	/* Put SPI into 8-bit mode */
}


#if FF_FS_READONLY == 0
/* Send multiple byte */
static
void xmit_spi_multi (
	const BYTE *buff,	/* Pointer to the data */
	UINT btx			/* Number of bytes to send (even number) */
)
{
	WORD d;

	SPI_MODE16BITS();		/* Put SPI into 16-bit mode */

	d = buff[0] << 8 | buff[1]; buff += 2;
	SPI_I2S_SendData(UFATFS_SPI, d);	/* Send the first word */
	btx -= 2;
	do {
		d = buff[0] << 8 | buff[1]; buff += 2;	/* Word to send next */
		while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_TXE) == RESET);	/* Wait for end of the SPI transaction */
		while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_RXNE) == RESET);
		(void)SPI_I2S_ReceiveData(UFATFS_SPI);							/* Discard received word */
		SPI_I2S_SendData(UFATFS_SPI, d);						/* Start next transaction */
	} while (btx -= 2);
	while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_TXE) == RESET);	/* Wait for end of the SPI transaction */
	while(SPI_I2S_GetFlagStatus(UFATFS_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	(void)SPI_I2S_ReceiveData(UFATFS_SPI);							/* Discard received word */

	SPI_MODE8BITS();	/* Put SPI into 8-bit mode */
}
#endif


/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
int wait_ready (	/* 1:Ready, 0:Timeout */
	UINT wt			/* Timeout [ms] */
)
{
	BYTE d;


	Timer2 = wt;
	do {
		d = xchg_spi(0xFF);
		/* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
	} while (d != 0xFF && Timer2);	/* Wait for card goes ready or timeout */

	return (d == 0xFF) ? 1 : 0;
}



/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static
void deselect (void)
{
	CS_HIGH();		/* Set CS# high */
	xchg_spi(0xFF);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}



/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/

static
int select (void)	/* 1:OK, 0:Timeout */
{
	CS_LOW();		/* Set CS# low */
	xchg_spi(0xFF);	/* Dummy clock (force DO enabled) */
	if (wait_ready(500)) return 1;	/* Wait for card ready */

	deselect();
	return 0;	/* Timeout */
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from the MMC                                    */
/*-----------------------------------------------------------------------*/

static
int rcvr_datablock (	/* 1:OK, 0:Error */
	BYTE *buff,			/* Data buffer */
	UINT btr			/* Data block length (byte) */
)
{
	BYTE token;


	Timer1 = 200;
	do {							/* Wait for DataStart token in timeout of 200ms */
		token = xchg_spi(0xFF);
		/* This loop will take a time. Insert rot_rdq() here for multitask envilonment. */
	} while ((token == 0xFF) && Timer1);
	if(token != 0xFE) return 0;		/* Function fails if invalid DataStart token or timeout */

	rcvr_spi_multi(buff, btr);		/* Store trailing data to the buffer */
	xchg_spi(0xFF); xchg_spi(0xFF);			/* Discard CRC */

	return 1;						/* Function succeeded */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to the MMC                                         */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
static
int xmit_datablock (	/* 1:OK, 0:Failed */
	const BYTE *buff,	/* Ponter to 512 byte data to be sent */
	BYTE token			/* Token */
)
{
	BYTE resp;


	if (!wait_ready(500)) return 0;		/* Wait for card ready */

	xchg_spi(token);					/* Send token */
	if (token != 0xFD) {				/* Send data if token is other than StopTran */
		xmit_spi_multi(buff, 512);		/* Data */
		xchg_spi(0xFF); xchg_spi(0xFF);	/* Dummy CRC */

		resp = xchg_spi(0xFF);				/* Receive data resp */
		if ((resp & 0x1F) != 0x05) return 0;	/* Function fails if the data packet was not accepted */
	}
	return 1;
}
#endif


/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (		/* Return value: R1 resp (bit7==1:Failed to send) */
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* Send a CMD55 prior to ACMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if (cmd != CMD12) {
		deselect();
		if (!select()) return 0xFF;
	}

	/* Send command packet */
	xchg_spi(0x40 | cmd);				/* Start + command index */
	xchg_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xchg_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xchg_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xchg_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xchg_spi(n);

	/* Receive command resp */
	if (cmd == CMD12) xchg_spi(0xFF);	/* Diacard following one byte when CMD12 */
	n = 10;								/* Wait for response (10 bytes max) */
	do {
		res = xchg_spi(0xFF);
	} while ((res & 0x80) && --n);

	return res;							/* Return received response */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive number (0) */
)
{
	BYTE n, cmd, ty, ocr[4];


	if (drv) return STA_NOINIT;			/* Supports only drive 0 */
	init_spi();							/* Initialize SPI */

	if (Stat & STA_NODISK) return Stat;	/* Is card existing in the soket? */

	FCLK_SLOW();
	for (n = 10; n; n--) xchg_spi(0xFF);	/* Send 80 dummy clocks */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Put the card SPI/Idle state */
		Timer1 = 1000;						/* Initialization timeout = 1 sec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
			for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);	/* Get 32 bit return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* Is the card supports vcc of 2.7-3.6V? */
				while (Timer1 && send_cmd(ACMD41, 1UL << 30)) ;	/* Wait for end of initialization with ACMD41(HCS) */
				if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* Card id SDv2 */
				}
			}
		} else {	/* Not SDv2 card */
			if (send_cmd(ACMD41, 0) <= 1) 	{	/* SDv1 or MMC? */
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
			}
			while (Timer1 && send_cmd(cmd, 0)) ;		/* Wait for end of initialization */
			if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
				ty = 0;
		}
	}
	CardType = ty;	/* Card type */
	deselect();

	if (ty) {			/* OK */
		FCLK_FAST();			/* Set fast clock */
		Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
	} else {			/* Failed */
		Stat = STA_NOINIT;
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv		/* Physical drive number (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only drive 0 */

	return Stat;	/* Return disk status */
}



/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,		/* Physical drive number (0) */
	BYTE *buff,		/* Pointer to the data buffer to store read data */
	LBA_t sector,	/* Start sector number (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	DWORD sect = (DWORD)sector;


	if (drv || !count) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */

	if (!(CardType & CT_BLOCK)) sect *= 512;	/* LBA ot BA conversion (byte addressing cards) */

	if (count == 1) {	/* Single sector read */
		if ((send_cmd(CMD17, sect) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512)) {
			count = 0;
		}
	}
	else {				/* Multiple sector read */
		if (send_cmd(CMD18, sect) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}



/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive number (0) */
	const BYTE *buff,	/* Ponter to the data to write */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	DWORD sect = (DWORD)sector;


	if (drv || !count) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check drive status */
	if (Stat & STA_PROTECT) return RES_WRPRT;	/* Check write protect */

	if (!(CardType & CT_BLOCK)) sect *= 512;	/* LBA ==> BA conversion (byte addressing cards) */

	if (count == 1) {	/* Single sector write */
		if ((send_cmd(CMD24, sect) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE)) {
			count = 0;
		}
	}
	else {				/* Multiple sector write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);	/* Predefine number of sectors */
		if (send_cmd(CMD25, sect) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD)) count = 1;	/* STOP_TRAN token */
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive number (0) */
	BYTE cmd,		/* Control command code */
	void *buff		/* Pointer to the conrtol data */
)
{
	DRESULT res;
	BYTE n, csd[16];
	DWORD st, ed, csize;
	LBA_t *dp;


	if (drv) return RES_PARERR;					/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */

	res = RES_ERROR;

	switch (cmd) {
	case CTRL_SYNC :		/* Wait for end of internal write process of the drive */
		if (select()) res = RES_OK;
		break;

	case GET_SECTOR_COUNT :	/* Get drive capacity in unit of sector (DWORD) */
		if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
			if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
				csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(LBA_t*)buff = csize << 10;
			} else {					/* SDC ver 1.XX or MMC ver 3 */
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(LBA_t*)buff = csize << (n - 9);
			}
			res = RES_OK;
		}
		break;

	case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
		if (CardType & CT_SD2) {	/* SDC ver 2.00 */
			if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
				xchg_spi(0xFF);
				if (rcvr_datablock(csd, 16)) {				/* Read partial block */
					for (n = 64 - 16; n; n--) xchg_spi(0xFF);	/* Purge trailing data */
					*(DWORD*)buff = 16UL << (csd[10] >> 4);
					res = RES_OK;
				}
			}
		} else {					/* SDC ver 1.XX or MMC */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
				if (CardType & CT_SD1) {	/* SDC ver 1.XX */
					*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
				} else {					/* MMC */
					*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
				}
				res = RES_OK;
			}
		}
		break;

	case CTRL_TRIM :	/* Erase a block of sectors (used when _USE_ERASE == 1) */
		if (!(CardType & CT_SDC)) break;				/* Check if the card is SDC */
		if (disk_ioctl(drv, MMC_GET_CSD, csd)) break;	/* Get CSD */
		if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;	/* Check if sector erase can be applied to the card */
		dp = buff; st = (DWORD)dp[0]; ed = (DWORD)dp[1];	/* Load sector block */
		if (!(CardType & CT_BLOCK)) {
			st *= 512; ed *= 512;
		}
		if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000)) {	/* Erase sector block */
			res = RES_OK;	/* FatFs does not check result of this command */
		}
		break;

	default:
		res = RES_PARERR;
	}

	deselect();

	return res;
}



/*-----------------------------------------------------------------------*/
/* Device timer function                                                 */
/*-----------------------------------------------------------------------*/
/* This function must be called from timer interrupt routine in period
/  of 1 ms to generate card control timing.
*/

void disk_timerproc (void)
{
	WORD n;
	BYTE s;


	n = Timer1;						/* 1kHz decrement timer stopped at 0 */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;

	s = Stat;
	if (MMC_WP) {	/* Write protected */
		s |= STA_PROTECT;
	} else {		/* Write enabled */
		s &= ~STA_PROTECT;
	}
	if (MMC_CD) {	/* Card is in socket */
		s &= ~STA_NODISK;
	} else {		/* Socket empty */
		s |= (STA_NODISK | STA_NOINIT);
	}
	Stat = s;
}

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
void UFATFS_TIM_IRQHandler(void)
{
	disk_timerproc();
	TIM_ClearITPendingBit(UFATFS_TIM,TIM_IT_Update);
}
