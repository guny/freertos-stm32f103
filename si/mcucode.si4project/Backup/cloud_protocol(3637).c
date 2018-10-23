
#include <stdint.h>
#include <string.h>

#include ".\usart\usart.h"
#include ".\cJSON\cJSON.h"
#include ".\cloud\cloud_protocol.h"

#define CLOUD_FRAMER_HEAD0 0xFE
#define CLOUD_FRAMER_HEAD1 0xFE
#define FRAMER_LENGTH_MAX 256
static uint8_t FramerBuff[FRAMER_LENGTH_MAX];
static uint32_t frame_length = 0;
static uint8_t frame_sn = 0;

static uint8_t data_check(uint8_t* src, uint32_t length)
{
	uint32_t i;
	uint8_t value;

	for(i = 0, value = 0; i < length; i++)
		value ^= src[i];

	return value;
}

static void mcu_receiver_esp_ack(uint8_t* data, uint32_t length)
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

	comSendBuf(COM2, ackbuff, 9);
}

void wifi_rest(void)
{
	uint8_t buff[10];
	memset(buff, 0, sizeof(buff));

	buff[0] = CLOUD_FRAMER_HEAD0;//frame head
	buff[1] = CLOUD_FRAMER_HEAD1;
	buff[3] = 0x05;
	buff[4] = MCUESP_MCU_WIFI_CONFIG;
	buff[5] = frame_sn++;
	buff[8] = 0x5A;
	buff[9] = data_check(buff + 4, 5);
	comSendBuf(COM2, buff, 10);
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

uint8_t buff0[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x03};// --探头
uint8_t buff1[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x03};
uint8_t buff2[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x02};// --点线
uint8_t buff3[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x02};
uint8_t buff4[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x04};// --能量加
uint8_t buff5[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x05};// --能量减
uint8_t buff6[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x06};// --持续时间加
uint8_t buff7[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x07};// --持续时间减
uint8_t buff8[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x08};// --间歇时间加
uint8_t buff9[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x09};// --间歇时间减
uint8_t buffa[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x0a};
uint8_t buffb[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x0a};
uint8_t buffc[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x0b};
uint8_t buffd[] = {0x5a,0xa5,0x06,0x83,0x23,0x09,0x1,0x55,0x0b};
uint8_t buffe[] = {0x5a,0xa5,0x06,0x83,0x30,0x00,0x1,0x00,0x00};
void cloud_process(void)
{
	uint32_t value;

	if(0 == CloudProtocolGetOnePacket())
	{
		mcu_receiver_esp_ack(FramerBuff, frame_length);
		//comSendBuf(COM1, FramerBuff, frame_length);
		if(0 == strncmp((const char *)FramerBuff + 4, "{\"Control\":", 11))
		{
			if(0 == strncmp((const char *)FramerBuff + 15, "20", 2))
				comSendBuf(COM1, buff0, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "21", 2))
				comSendBuf(COM3, buff1, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "19", 2))
				comSendBuf(COM3, buff2, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "18", 2))
				comSendBuf(COM3, buff3, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "10", 2))
				comSendBuf(COM3, buff4, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "11", 2))
				comSendBuf(COM3, buff5, sizeof(buff5));
			else if(0 == strncmp((const char *)FramerBuff + 15, "12", 2))
				comSendBuf(COM3, buff6, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "13", 2))
				comSendBuf(COM3, buff7, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "14", 2))
				comSendBuf(COM3, buff8, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "15", 2))
				comSendBuf(COM3, buff9, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "16", 2))
				comSendBuf(COM3, buffa, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "17", 2))
				comSendBuf(COM3, buffb, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "22", 2))
				comSendBuf(COM3, buffc, 9);
			else if(0 == strncmp((const char *)FramerBuff + 15, "23", 2))
				comSendBuf(COM3, buffd, 9);
		}
		else if(0 == strncmp((const char *)FramerBuff + 4, "{\"pay\":", 7))
		{
			value = atoi(FramerBuff + 11);
			buffe[sizeof(buffe) - 2] = value / 0x100;
			buffe[sizeof(buffe) - 1] = value % 0x100;
			comSendBuf(COM3, buffe, 9);
		}
		memset(FramerBuff, 0x00, sizeof(FramerBuff));
		frame_length = 0;
	}
}
