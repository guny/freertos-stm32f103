
#include <stdint.h>
#include "..\crc\crc16.h"

static const  uint16_t crc16L[] =
{
    0x0000, 0xc0c1, 0xc181, 0x0140,
    0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741,
    0x0500, 0xc5c1, 0xc481, 0x0440,
};

static const  uint16_t crc16H[] =
{
    0x0000, 0xcc01, 0xd801, 0x1400,
    0xf001, 0x3c00, 0x2800, 0xe401,
    0xa001, 0x6c00, 0x7800, 0xb401,
    0x5000, 0x9c01, 0x8801, 0x4400,
};

uint16_t CRC16(uint16_t BraekPoint, uint8_t *Buffer, uint32_t Length)
{
	uint16_t CRCValue;
	uint8_t Dat;

	CRCValue = BraekPoint;

	while(Length--)
	{
		Dat = *Buffer++;
		Dat ^= CRCValue;
		CRCValue >>= 8;
		CRCValue ^= crc16L[Dat & 0x0F];
		CRCValue ^= crc16H[(Dat>>4) & 0x0F];
	}

	return CRCValue;
}


