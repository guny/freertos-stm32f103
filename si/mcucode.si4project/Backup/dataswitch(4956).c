
#include <stdint.h>
#include <string.h>
#include ".\usart\usart.h"
#include ".\crc\crc16.h"
#include ".\include\config.h"
#include ".\include\dataswitch.h"

#define FRAME_HEADER0                GusRigister.Rigister[0x03]//0x5A
#define FRAME_HEADER1                GusRigister.Rigister[0x0A]//0xA5

#define COMMAND_FRAME_HEAD0          0xAA
#define COMMAND_FRAME_END0           0xCC
#define COMMAND_FRAME_END1           0x33
#define COMMAND_FRAME_END2           0xC3
#define COMMAND_FRAME_END3           0x3C

#define _RECEIVE_CRC_ENABLE         (0x10 == (GusRigister.Rigister[0x0C] & 0x10))
#define FRAME_LENGTH_MAX             260

typedef struct {
	uint8_t DataBuff[FRAME_LENGTH_MAX];
	uint32_t DataLength;
	uint32_t ReceiveStatus;
} GUSCOMMAND_RECEIVER_STRUCT;

int8_t GetOneLcmPacket(COM_PORT_E Com, GUSCOMMAND_RECEIVER_STRUCT* Command)
{
	uint8_t  temp;
#if 1
	while(1 == comGetChar(Com, &temp))
	{
		//printf("status %u lenght %u >> 0x%.2X\n\r", Command->ReceiveStatus, Command->DataLength, temp);
		if(3 == Command->ReceiveStatus)
		{
			Command->DataBuff[Command->DataLength++] = temp;
			if((FRAME_LENGTH_MAX + 1) < Command->DataLength)
			{
				Command->ReceiveStatus = 0;
				return -2;
			}
			else if(6 <= Command->DataLength && \
							COMMAND_FRAME_END0 == Command->DataBuff[Command->DataLength - 4] && \
				 			COMMAND_FRAME_END1 == Command->DataBuff[Command->DataLength - 3] && \
							COMMAND_FRAME_END2 == Command->DataBuff[Command->DataLength - 2] && \
				 			COMMAND_FRAME_END3 == Command->DataBuff[Command->DataLength - 1])
			{
				Command->ReceiveStatus = 0;
				return 0;
			}
		}
		else if(2 == Command->ReceiveStatus)
		{
			Command->DataBuff[Command->DataLength++] = temp;
			if((FRAME_LENGTH_MAX + 4) <= Command->DataLength)
			{
				Command->ReceiveStatus = 0;
				return -2;
			}
			else if(3 < Command->DataLength && Command->DataBuff[2] == (Command->DataLength - 3))
			{
				Command->ReceiveStatus = 0;
				return 0;
			}
		}
		else if(1 == Command->ReceiveStatus)
		{
			Command->DataBuff[Command->DataLength++] = temp;
			if(FRAME_HEADER1 == temp)
				Command->ReceiveStatus = 2;
			else
				Command->ReceiveStatus = 3;
		}
		else if(0 == Command->ReceiveStatus && (FRAME_HEADER0 == temp || COMMAND_FRAME_HEAD0 == temp))
		{
			Command->DataLength = 0;
			memset(Command->DataBuff, 0, FRAME_LENGTH_MAX);
			Command->DataBuff[Command->DataLength++] = temp;
			Command->ReceiveStatus = 1;
		}
	}
#else //guscloud frame
	while(1 == comGetChar(Com, &temp))
	{
		if(2 == Command->ReceiveStatus)
		{
			Command->DataBuff[Command->DataLength++] = temp;
			if((FRAME_LENGTH_MAX + 4) <= Command->DataLength)
			{
				return -2;
			}
			else if(3 < Command->DataLength && Command->DataBuff[2] == (Command->DataLength - 3))
			{
				Command->ReceiveStatus = 0;
				return 0;
			}
		}
		else if(1 == Command->ReceiveStatus && FRAME_HEADER1 == temp)
		{
			Command->DataBuff[Command->DataLength++] = FRAME_HEADER1;
			Command->ReceiveStatus = 2;
		}
		else if(0 == Command->ReceiveStatus && FRAME_HEADER0 == temp)
		{
			Command->DataLength = 0;
			memset(Command->DataBuff, 0, FRAME_LENGTH_MAX);
			Command->DataBuff[Command->DataLength++] = FRAME_HEADER0;
			Command->ReceiveStatus = 1;
		}
	}
#endif
	return -1;
}


void user_main_lcm_dataswitch_process(void)
{
	uint16_t check_crc16;
	static GUSCOMMAND_RECEIVER_STRUCT LcmCommand;
	static GUSCOMMAND_RECEIVER_STRUCT ControllerCommand;

	if(0 == GetOneLcmPacket(COM1, &LcmCommand))
	{
		comSendBuf(COM3, LcmCommand.DataBuff, LcmCommand.DataLength);
		//comSendBuf(COM1, LcmCommand.DataBuff, LcmCommand.DataLength);
		if(0x80 == LcmCommand.DataBuff[0x03])
		{
			if(_RECEIVE_CRC_ENABLE)
			{
				check_crc16 = LcmCommand.DataBuff[LcmCommand.DataLength - 1] | \
											LcmCommand.DataBuff[LcmCommand.DataLength - 2];
				if(check_crc16 == CRC16(0xFFFF, LcmCommand.DataBuff + 3, LcmCommand.DataLength - 5))
					user_config_register(LcmCommand.DataBuff + 4, LcmCommand.DataLength - 6);
			}
			else
				user_config_register(LcmCommand.DataBuff + 4, LcmCommand.DataLength - 4);
		}
	}

	if(0 == GetOneLcmPacket(COM3, &ControllerCommand))
	{
		comSendBuf(COM1, ControllerCommand.DataBuff, ControllerCommand.DataLength);
		if(0x80 == ControllerCommand.DataBuff[0x03])
		{
			if(_RECEIVE_CRC_ENABLE)
			{
				check_crc16 = ControllerCommand.DataBuff[ControllerCommand.DataLength - 1] | \
											ControllerCommand.DataBuff[ControllerCommand.DataLength - 2];
				if(check_crc16 == CRC16(0xFFFF, ControllerCommand.DataBuff + 3, ControllerCommand.DataLength - 5))
					user_config_register(ControllerCommand.DataBuff + 4, ControllerCommand.DataLength - 6);
			}
			else
				user_config_register(ControllerCommand.DataBuff + 4, ControllerCommand.DataLength - 4);
		}
	}
}
