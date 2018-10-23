
#include <stdio.h>
#include "stm32f10x.h"
#include "..\usart\usart.h"

#if 1 /* FreeRTOS : Enable */
	#include "FreeRTOS.h"
	#include "task.h"
	#define DISABLE_INT()    taskENTER_CRITICAL()
	#define ENABLE_INT()     taskEXIT_CRITICAL()
#else
	#define ENABLE_INT()	__set_PRIMASK(0)
	#define DISABLE_INT()	__set_PRIMASK(1)
#endif

typedef struct {
	USART_TypeDef *usart;

	__IO uint8_t* pTxBuf;
	__IO uint8_t* pRxBuf;

	uint16_t TxBufSize;
	uint16_t RxBufSize;

	__IO uint32_t TxWrite;
	__IO uint32_t TxRead;

	__IO uint32_t RxWrite;
	__IO uint32_t RxRead;

	void (*UsartSendMode)(void);
	void (*UsartReciveMode)(void);
} USART_T;

#define USART_BUFFER_SIZE  (1024)
#define __BUF_MASK         (USART_BUFFER_SIZE - 1)

#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))

static USART_T gLcmUsart;
static __IO uint8_t gUsartLcm_TX_Buffer[USART_BUFFER_SIZE];
static __IO uint8_t gUsartLcm_RX_Buffer[USART_BUFFER_SIZE];

static USART_T gCloudUsart;
static __IO uint8_t gUsartCloud_TX_Buffer[USART_BUFFER_SIZE];
static __IO uint8_t gUsartCloud_RX_Buffer[USART_BUFFER_SIZE];

static USART_T gControlUsart;
static __IO uint8_t gUsartControl_TX_Buffer[USART_BUFFER_SIZE];
static __IO uint8_t gUsartControl_RX_Buffer[USART_BUFFER_SIZE];

#ifdef COMMUNICATION_RS485_ENABLE
#define RS485_RCC_CLK    RCC_APB2Periph_GPIOB
#define RS485_PORT_PORT  GPIOB
#define RS485_PIN_NUM    GPIO_Pin_2

static void _rs485_controller_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RS485_RCC_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = RS485_PIN_NUM;
	GPIO_Init(RS485_PORT_PORT, &GPIO_InitStructure);
}

static void _rs485_recive_mode(void)
{
	RS485_PORT_PORT->BRR |= RS485_PIN_NUM;
}

static void _rs485_send_mode(void)
{
	RS485_PORT_PORT->BSRR |= RS485_PIN_NUM;
}
#endif
#if 0
void GPIO_Tese(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	while(1)
	{
		GPIOB->BRR |= GPIO_Pin_10;
		GPIOB->BSRR |= GPIO_Pin_11;
		GPIOB->BSRR |= GPIO_Pin_10;
		GPIOB->BRR |= GPIO_Pin_11;
	}
}
#endif

static void _UsartVarInit(void)
{
	gLcmUsart.usart = USART1;
	gLcmUsart.pTxBuf = gUsartLcm_TX_Buffer;
	gLcmUsart.pRxBuf = gUsartLcm_RX_Buffer;
	gLcmUsart.TxBufSize = USART_BUFFER_SIZE;
	gLcmUsart.RxBufSize = USART_BUFFER_SIZE;
	gLcmUsart.TxWrite = 0;
	gLcmUsart.TxRead  = 0;
	gLcmUsart.RxWrite = 0;
	gLcmUsart.RxRead  = 0;
	gLcmUsart.UsartSendMode   = 0;//_rs485_send_mode;
	gLcmUsart.UsartReciveMode = 0;//_rs485_recive_mode;

	gCloudUsart.usart = USART2;
	gCloudUsart.pTxBuf = gUsartCloud_TX_Buffer;
	gCloudUsart.pRxBuf = gUsartCloud_RX_Buffer;
	gCloudUsart.TxBufSize = USART_BUFFER_SIZE;
	gCloudUsart.RxBufSize = USART_BUFFER_SIZE;
	gCloudUsart.TxWrite = 0;
	gCloudUsart.TxRead  = 0;
	gCloudUsart.RxWrite = 0;
	gCloudUsart.RxRead  = 0;
	gCloudUsart.UsartSendMode   = 0;
	gCloudUsart.UsartReciveMode = 0;

	gControlUsart.usart = USART3;
	gControlUsart.pTxBuf = gUsartControl_TX_Buffer;
	gControlUsart.pRxBuf = gUsartControl_RX_Buffer;
	gControlUsart.TxBufSize = USART_BUFFER_SIZE;
	gControlUsart.RxBufSize = USART_BUFFER_SIZE;
	gControlUsart.TxWrite = 0;
	gControlUsart.TxRead  = 0;
	gControlUsart.RxWrite = 0;
	gControlUsart.RxRead  = 0;
	gControlUsart.UsartSendMode   = 0;//_rs485_send_mode;
	gControlUsart.UsartReciveMode = 0;//_rs485_recive_mode;
}

typedef struct {
	const uint8_t  Index;
	const uint32_t Baud;
}BAUDRATE_START;

const BAUDRATE_START BaudRateTabel[] = {
	0x00,   1200,
	0x01,   2400,
	0x02,   4800,
	0x03,   9600,
	0x04,  19200,
	0x05,  38400,
	0x06,  57600,
	0x07, 115200,
	0x08,  28800,
	0x09,  76800,
	0x0A,  62500,
	0x0B, 125000,
	0x0C, 250000,
	0x0D, 230400,
	0x0E, 345600,
	0x0F, 460800,
	0x10, 625000,
	0x11, 691200,
	0x12, 921600
};

#define BAUDTABLE_LENGTH    (sizeof(BaudRateTabel) / sizeof(*BaudRateTabel))
#define SELFBAUDRATE_FLAG   (0xFE)
//#define BaudRateH           (SYSTEM_INFO->HigBaud)
//#define BaudRateL           (SYSTEM_INFO->LowBaud)
#define BAUDRATE_MAX        (6250000)
uint32_t BaudRateAnalysis(uint8_t Index)
{
	if(BAUDTABLE_LENGTH > Index)
		return BaudRateTabel[Index].Baud;
//	else if(SELFBAUDRATE_FLAG == Index)
//		return (BAUDRATE_MAX / ((BaudRateH << 8) | BaudRateL));
	else
		return BaudRateTabel[0x07].Baud;
}

//#define USART2_USER_GPIOD_PD5_PD6
//#define USART3_USER_GPIOC_PC10_PC11
//#define USART3_USER_GPIOD_PD8_PD9
static void _UsartHardInit(uint32_t BaudRateIndex)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	uint32_t baudrate = 115200;

	baudrate = BaudRateAnalysis(BaudRateIndex);

	/* USART-1 : LCM-Module */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);

	/* USART-2 : Wi-Fi Module */
#ifdef USART2_USER_GPIOD_PD5_PD6
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
#else //defined : GPIOA PA2_PA3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
#ifdef ESP_UPDATA
	USART_InitStructure.USART_BaudRate = 115200;
#else
	USART_InitStructure.USART_BaudRate = 9600;
#endif
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);

	/* USART-3 : Main-Controller */
#ifdef USART3_USER_GPIOC_PC10_PC11
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#elif defined(USART3_USER_GPIOD_PD8_PD9)
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
#else //defined : GPIOB PB10_PB11 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
}

static void _UsartNvicConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* USART-1 : LCM-Module */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* USART-2 : Wi-Fi Module */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* USART-2 : Main-Controller */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void usart_init(uint32_t BaudRateIndex)
{
	_UsartVarInit();

	_UsartHardInit(BaudRateIndex);

	_UsartNvicConfig();

//	_rs485_controller_init();

//	GPIO_Tese();
}

/*
*********************************************************************************************************
*	函 数 名: UartSend
*	功能说明: 填写数据到UART发送缓冲区,并启动发送中断。中断处理函数发送完毕后，自动关闭发送中断
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void UartSend(USART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint32_t i = 0;
#if 0 // User : OS Mode
	for(i = 0; i < _usLen; i++)
	{
//		printf("%u\n\r", i);
		while(1)
		{
			while (USART_GetFlagStatus(_pUart->usart, USART_FLAG_TC) == RESET);

			DISABLE_INT();
			if(_pUart->TxRead == (_pUart->TxWrite+1))
			{
				ENABLE_INT();
				continue;
			}
			else
			{
				ENABLE_INT();
				break;
			}
		}

		_pUart->pTxBuf[_pUart->TxWrite] = _ucaBuf[i];
		DISABLE_INT();
		if (++_pUart->TxWrite >= _pUart->TxBufSize)
		{
			_pUart->TxWrite = 0;
		}
		ENABLE_INT();
	}
#else // User : Non-OS Mode
	for(i = 0; i < _usLen; i++)
	{
		while(1)
		{
			while (USART_GetFlagStatus(_pUart->usart, USART_FLAG_TC) == RESET);

			USART_ITConfig(_pUart->usart, USART_IT_TXE, DISABLE);
			if(_pUart->TxRead == (_pUart->TxWrite+1))
			{
				USART_ITConfig(_pUart->usart, USART_IT_TXE, ENABLE);
				continue;
			}
			else
			{
//				USART_ITConfig(_pUart->usart, USART_IT_TXE, ENABLE);
				break;
			}
		}

		_pUart->pTxBuf[_pUart->TxWrite] = _ucaBuf[i];
//		USART_ITConfig(_pUart->usart, USART_IT_TXE, DISABLE);
		if (++_pUart->TxWrite >= _pUart->TxBufSize)
		{
			_pUart->TxWrite = 0;
		}
		USART_ITConfig(_pUart->usart, USART_IT_TXE, ENABLE);
	}
#endif

	USART_ITConfig(_pUart->usart, USART_IT_TXE, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: UartGetChar
*	功能说明: 从串口接收缓冲区读取1字节数据 （用于主程序调用）
*	形    参: _pUart : 串口设备
*			  _pByte : 存放读取数据的指针
*	返 回 值: 0 表示无数据  1表示读取到数据
*********************************************************************************************************
*/
static uint8_t UartGetChar(USART_T *_pUart, uint8_t *_pByte)
{
	DISABLE_INT();
	if(_pUart->RxWrite != _pUart->RxRead)
	{
		*_pByte = _pUart->pRxBuf[_pUart->RxRead];

		if( ++_pUart->RxRead >=  _pUart->RxBufSize)
			 _pUart->RxRead = 0;

		ENABLE_INT();
		return 1;
	}
	else
	{
		ENABLE_INT();
		return 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: UartIRQ
*	功能说明: 供中断服务程序调用，通用串口中断处理函数
*	形    参: _pUart : 串口设备
*	返 回 值: 无
*********************************************************************************************************
*/
//static inline void UartIRQ_ReceiveTransmit_Handler(USART_T *Receive, USART_T *Transmit)
//{
//	if (USART_GetITStatus(Receive->usart, USART_IT_RXNE) != RESET)
//	{
//		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
//		Transmit->pRxBuf[Transmit->RxWrite] = USART_ReceiveData(Receive->usart);

//		if (++Transmit->RxWrite >= Transmit->RxBufSize)
//		{
//			Transmit->RxWrite = 0;
//		}
//	}

//	/* 处理发送缓冲区空中断 */
//	if (USART_GetITStatus(Transmit->usart, USART_IT_TXE) != RESET)
//	{
//		if(Transmit->TxRead == Transmit->TxWrite)
//		{
//			/* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
//			USART_ITConfig(Transmit->usart, USART_IT_TXE, DISABLE);

//			/* 使能数据发送完毕中断 */
//			USART_ITConfig(Transmit->usart, USART_IT_TC, ENABLE);
//		}
//		else
//		{
//			/* 从发送FIFO取1个字节写入串口发送数据寄存器 */
//			USART_SendData(Transmit->usart, Transmit->pTxBuf[Transmit->TxRead]);
//			if (++Transmit->TxRead >= Transmit->TxBufSize)
//			{
//				Transmit->TxRead = 0;
//			}
//		}

//	}
//	/* 数据bit位全部发送完毕的中断 */
//	else if (USART_GetITStatus(Transmit->usart, USART_IT_TC) != RESET)
//	{
//		if (Transmit->TxRead == Transmit->TxWrite)
//		{
//			/* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
//			USART_ITConfig(Transmit->usart, USART_IT_TC, DISABLE);

//			/* 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线 */
//			if (Transmit->UsartReciveMode)
//			{
//				Transmit->UsartReciveMode();
//			}
//		}
//		else
//		{
//			/* 正常情况下，不会进入此分支 */

//			/* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
//			USART_SendData(Transmit->usart, Transmit->pTxBuf[Transmit->TxRead]);
//			if (++Transmit->TxRead >= Transmit->TxBufSize)
//			{
//				Transmit->TxRead = 0;
//			}
//		}
//	}
//}

//void USART1_IRQHandler(void)
//{
//	UartIRQ_ReceiveTransmit_Handler(&gLcmUsart, &gCloudUsart);
//}

//void USART2_IRQHandler(void)
//{
//	UartIRQ_ReceiveTransmit_Handler(&gCloudUsart, &gLcmUsart);
//}

//void USART3_IRQHandler(void)
//{
//	UartIRQ_ReceiveTransmit_Handler(&gControlUsart, &gCloudUsart);
//}

static inline void UartIRQ_Receive_Handler(USART_T *_pUart)
{
	/* 处理接收中断  */
	if (USART_GetITStatus(_pUart->usart, USART_IT_RXNE) != RESET)
	{
		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
		_pUart->pRxBuf[_pUart->RxWrite] = USART_ReceiveData(_pUart->usart);

		if (++_pUart->RxWrite >= _pUart->RxBufSize)
		{
			_pUart->RxWrite = 0;
		}
	}
}

static inline void UartIRQ_Transmit_Handler(USART_T *_pUart)
{
	/* 处理发送缓冲区空中断 */
	if (USART_GetITStatus(_pUart->usart, USART_IT_TXE) != RESET)
	{
		if(_pUart->TxRead == _pUart->TxWrite)
		{
			/* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
			USART_ITConfig(_pUart->usart, USART_IT_TXE, DISABLE);

			/* 使能数据发送完毕中断 */
			USART_ITConfig(_pUart->usart, USART_IT_TC, ENABLE);
		}
		else
		{
			/* 从发送FIFO取1个字节写入串口发送数据寄存器 */
			USART_SendData(_pUart->usart, _pUart->pTxBuf[_pUart->TxRead]);
			if (++_pUart->TxRead >= _pUart->TxBufSize)
			{
				_pUart->TxRead = 0;
			}
		}

	}
	/* 数据bit位全部发送完毕的中断 */
	else if (USART_GetITStatus(_pUart->usart, USART_IT_TC) != RESET)
	{
		if (_pUart->TxRead == _pUart->TxWrite)
		{
			/* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
			USART_ITConfig(_pUart->usart, USART_IT_TC, DISABLE);

			/* 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线 */
			if (_pUart->UsartReciveMode)
			{
				_pUart->UsartReciveMode();
			}
		}
		else
		{
			/* 正常情况下，不会进入此分支 */

			/* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
			USART_SendData(_pUart->usart, _pUart->pTxBuf[_pUart->TxRead]);
			if (++_pUart->TxRead >= _pUart->TxBufSize)
			{
				_pUart->TxRead = 0;
			}
		}
	}
}

static void UartIRQ(USART_T *_pUart)
{
	UartIRQ_Receive_Handler(_pUart);
	UartIRQ_Transmit_Handler(_pUart);
}

void USART1_IRQHandler(void)
{
	UartIRQ(&gLcmUsart);
}

void USART2_IRQHandler(void)
{
	UartIRQ(&gCloudUsart);
}

void USART3_IRQHandler(void)
{
	UartIRQ(&gControlUsart);
}
/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为UART指针
*	形    参: _ucPort: 端口号(COM1 - COM3)
*	返 回 值: uart指针
*********************************************************************************************************
*/
USART_T *ComToUart(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
		return &gLcmUsart;
	}
	else if (_ucPort == COM2)
	{
		return &gCloudUsart;
	}
	else if (_ucPort == COM3)
	{
		return &gControlUsart;
	}
	else
	{
		return 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: comSendBuf
*	功能说明: 向串口发送一组数据。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM3)
*			  _ucaBuf: 待发送的数据缓冲区
*			  _usLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen)
{
	USART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	if(pUart->UsartSendMode)
	{
		pUart->UsartSendMode();		/* 如果是RS485通信，可以在这个函数中将RS485设置为发送模式 */
	}

	UartSend(pUart, _ucaBuf, _usLen);
}

/*
*********************************************************************************************************
*	函 数 名: comSendChar
*	功能说明: 向串口发送1个字节。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM3)
*			  _ucByte: 待发送的数据
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1);
}

uint32_t comCanReadNum(COM_PORT_E _ucPort)
{
	USART_T *pUart;
	uint32_t Numbers;

	if(0 == (pUart = ComToUart(_ucPort)))
		return 0;
	else
	{
		while (USART_GetFlagStatus(pUart->usart, USART_FLAG_TC) == RESET);
#if 0
		DISABLE_INT();
		if(pUart->RxRead == pUart->RxWrite)
			Numbers = 0;
		else if(pUart->RxRead < pUart->RxWrite)
			Numbers = pUart->RxWrite - pUart->RxRead;
		else	
			Numbers = pUart->RxBufSize - (pUart->RxWrite - pUart->RxRead);
		ENABLE_INT();
#else
		USART_ITConfig(pUart->usart, USART_IT_RXNE, DISABLE);
		if(pUart->RxRead == pUart->RxWrite)
			Numbers = 0;
		else if(pUart->RxRead < pUart->RxWrite)
			Numbers = pUart->RxWrite - pUart->RxRead;
		else	
			Numbers = pUart->RxBufSize - (pUart->RxWrite - pUart->RxRead);
		USART_ITConfig(pUart->usart, USART_IT_RXNE, ENABLE);
#endif

		return Numbers;
	}
}

uint32_t comCanWriteNum(COM_PORT_E _ucPort)
{
	
	USART_T *pUart;
	uint32_t Numbers;

	if(0 == (pUart = ComToUart(_ucPort)))
		return 0;
	else
	{
#if 0
		DISABLE_INT();
		if(pUart->TxRead == pUart->TxWrite)
			Numbers = pUart->TxBufSize; 
		else if(pUart->TxRead < pUart->TxWrite)
			Numbers = pUart->TxBufSize - (pUart->TxWrite - pUart->TxRead);
		else 
			Numbers = pUart->TxRead - pUart->TxWrite;
		ENABLE_INT();
#else
		USART_ITConfig(pUart->usart, USART_IT_TXE, DISABLE);
		if(pUart->TxRead == pUart->TxWrite)
			Numbers = pUart->TxBufSize; 
		else if(pUart->TxRead < pUart->TxWrite)
			Numbers = pUart->TxBufSize - (pUart->TxWrite - pUart->TxRead);
		else 
			Numbers = pUart->TxRead - pUart->TxWrite;
		USART_ITConfig(pUart->usart, USART_IT_TXE, ENABLE);
#endif
		return Numbers;
	}
}

/*
*********************************************************************************************************
*	函 数 名: comGetChar
*	功能说明: 从串口缓冲区读取1字节，非阻塞。无论有无数据均立即返回
*	形    参: _ucPort: 端口号(COM1 - COM3)
*			  _pByte: 接收到的数据存放在这个地址
*	返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte)
{
	USART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}

	return UartGetChar(pUart, _pByte);
}

/*
*********************************************************************************************************
*	函 数 名: comClearTxFifo
*	功能说明: 清零串口发送缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM3)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearTxFifo(COM_PORT_E _ucPort)
{
	USART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->TxWrite = 0;
	pUart->TxRead = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comClearRxFifo
*	功能说明: 清零串口接收缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM3)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearRxFifo(COM_PORT_E _ucPort)
{
	USART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->RxWrite = 0;
	pUart->RxRead = 0;
}

/*
*********************************************************************************************************
*	函 数 名: fputc
*	功能说明: 重定义putc函数，这样可以使用printf函数从串口1打印输出
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int fputc(int ch, FILE *f)
{
#if 1	/* 将需要printf的字符通过串口中断FIFO发送出去，printf函数会立即返回 */
	comSendChar(COM1, ch);

	return ch;
#else	/* 采用阻塞方式发送每个字符,等待数据发送完毕 */
	/* 写一个字节到USART1 */
	USART_SendData(USART1, (uint8_t) ch);

	/* 等待发送结束 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	return ch;
#endif
}

/*
*********************************************************************************************************
*	函 数 名: fgetc
*	功能说明: 重定义getc函数，这样可以使用getchar函数从串口1输入数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int fgetc(FILE *f)
{

#if 1	/* 从串口接收FIFO中取1个数据, 只有取到数据才返回 */
	uint8_t ucData;

	while(comGetChar(COM1, &ucData) == 0);

	return ucData;
#else
	/* 等待串口1输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
#endif
}

