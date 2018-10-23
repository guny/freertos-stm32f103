
#include <stdint.h>
#include <string.h>

#include ".\usart\usart.h"
#include ".\cJSON\cJSON.h"
#include ".\cloud\cloud_protocol.h"

#define CLOUD_FRAMER_HEAD0 0xFE
#define CLOUD_FRAMER_HEAD1 0xFE
#define FRAMER_LENGTH_MAX 512
static uint8_t FramerBuff[FRAMER_LENGTH_MAX];
static uint32_t frame_length = 0;
static uint8_t frame_sn = 0;

static void esp_receiver_mcu_ack(uint8_t* data, uint32_t length)
{
	uint8_t ackbuff[10];

	memset(ackbuff, 0x00, sizeof(ackbuff));
	ackbuff[0] = CLOUD_FRAMER_HEAD0;//frame head
	ackbuff[1] = CLOUD_FRAMER_HEAD1;
	ackbuff[3] = 0x05;
	ackbuff[4] = data[0] + 1;//cmd
	ackbuff[5] = data[1];//sn
	ackbuff[6] = data[2];//flag
	ackbuff[7] = data[3];
	ackbuff[8] = data_check(ackbuff + 4, 4);	

	uart0_tx_buffer(ackbuff, 9);
}

static int CloudProtocolGetOnePacket(void)
{
	static uint32_t ReceiverStatus = 0;
	static uint32_t conuter;
	static uint8_t  check_value;
	uint8_t temp;

	while(1 == comGetChar(COM2, &temp))
	{
		if(3 == ReceiverStatus)
		{
			FramerBuff[conuter++] = temp;
			check_value ^= FramerBuff[conuter - 1];
			if(frame_length <= conuter)
			{
				conuter = ReceiverStatus = 0;
				if(0 == check_value)
					return 0;
			}
			else if(FRAMER_LENGTH_MAX <= conuter)
				frame_length = conuter = ReceiverStatus = 0;
		}
		else if(2 == ReceiverStatus)
		{
			frame_length = frame_length * 0x100 + temp;
			if(1 <= conuter++)
			{
				memset(FramerBuff, 0x00, sizeof(FramerBuff));
				check_value = conuter = 0;
				ReceiverStatus = 3;
			}
		}
		else if(CLOUD_FRAMER_HEAD1 == temp && 1 == ReceiverStatus)
		{
			frame_length = conuter = 0;
			ReceiverStatus = 2;
		}
		else if(CLOUD_FRAMER_HEAD0 == temp && 0 == ReceiverStatus)
			ReceiverStatus = 1;
		else
			ReceiverStatus = 0;
	}

	return -1;
}

void cloud_process(void)
{
	if(0 == CloudProtocolGetOnePacket())
	{
		comSendBuf(COM1, FramerBuff, frame_length);
		memset(FramerBuff, 0x00, sizeof(FramerBuff));
		frame_length = 0;
	}
}
