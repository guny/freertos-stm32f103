
#ifndef _USART_H_
#define _USART_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/* ¶¨Òå¶Ë¿ÚºÅ */
typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 */
	COM2 = 1,	/* USART2, PA2, PA3 */
	COM3 = 2,	/* USART3, PB10, PB11 */
}COM_PORT_E;

void usart_init(uint32_t BaudRateIndex);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);
uint32_t comCanReadNum(COM_PORT_E _ucPort);
uint32_t comCanWriteNum(COM_PORT_E _ucPort);

void comClearTxFifo(COM_PORT_E _ucPort);
void comClearRxFifo(COM_PORT_E _ucPort);

#ifdef __cplusplus
 }
#endif
 
#endif // _USART_H_
