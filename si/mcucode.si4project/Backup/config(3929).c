
#include <stdint.h>
#include <string.h>
#include ".\spi\spi.h"
#include ".\usart\usart.h"
#include ".\crc\crc16.h"
#include ".\cloud\cloud_protocol.h"
#include ".\include\config.h"

static void Flash_Read(uint8_t* dst, uint32_t addr, uint32_t lenght)
{
	/* Read data from SPI FLASH memory */
	sFLASH_ReadBuffer(dst, addr, lenght);
}

static void Flash_Write(uint8_t* src, uint32_t addr, uint32_t lenght)
{
	/* Erase SPI FLASH Sector to write on */
  sFLASH_EraseSector(addr);

  /* Write Tx_Buffer data to SPI FLASH memory */
  sFLASH_WriteBuffer(src, addr, lenght);
}

#define OFFSET(TYPE , MEMBER)((unsigned long)(&(((TYPE *)0)->MEMBER)))
#define CrcCheckLength               OFFSET(GUSRIGISTER_T, crc)
#define GUSRIGISTER_FLASH_ADDR0     (0x00000000)
#define GUSRIGISTER_FLASH_ADDR1     (0x00010000)

static uint8_t Rigister[256];
GUSRIGISTER_T GusRigister;

void GusRisgster_Write(void)
{
	GusRigister.times++;
	GusRigister.crc = CRC16(0xFFFF, (uint8_t *)&GusRigister, CrcCheckLength);
	if(GusRigister.times & 0x01)
		Flash_Write((uint8_t *)&GusRigister, GUSRIGISTER_FLASH_ADDR1, sizeof(GUSRIGISTER_T));
	else
		Flash_Write((uint8_t *)&GusRigister, GUSRIGISTER_FLASH_ADDR0, sizeof(GUSRIGISTER_T));
}

static void GusRisgster_Init(void)
{
	GusRigister.Rigister[0x01] = 0x07;
	GusRigister.Rigister[0x05] = 0x00;
	GusRigister.Rigister[0x09] = 0x01;

	GusRigister.Rigister[0x02] = 0x00;
	GusRigister.Rigister[0x0C] = 0x00;
	
	GusRigister.Rigister[0x03] = 0x5A;
	GusRigister.Rigister[0x0A] = 0xA5;
	
	GusRigister.Rigister[0x06] = 0x40;
	GusRigister.Rigister[0x07] = 0x00;
	GusRigister.Rigister[0x08] = 0x00;

	memset((uint8_t *)&GusRigister.times, 0xFF, sizeof(GusRigister.times));
}

void GusRisgster_Read(void)
{
	GUSRIGISTER_T GusRegisterTemp;

	sFLASH_Init();

	memset(Rigister, 0, sizeof(Rigister));
	Flash_Read((uint8_t *)&GusRigister,     GUSRIGISTER_FLASH_ADDR0, sizeof(GUSRIGISTER_T));
	Flash_Read((uint8_t *)&GusRegisterTemp, GUSRIGISTER_FLASH_ADDR1, sizeof(GUSRIGISTER_T));
	
	GusRisgster_Init();

	if(0xFFFFFFFF == GusRigister.times && 0xFFFFFFFF == GusRegisterTemp.times)
	{
		GusRisgster_Init();
		GusRisgster_Write();
	}
	else if(GusRigister.times > GusRegisterTemp.times)
	{
		if(GusRigister.crc == CRC16(0xFFFF, (uint8_t *)&GusRigister, CrcCheckLength))
		{
			Flash_Write((uint8_t *)&GusRigister, GUSRIGISTER_FLASH_ADDR1, sizeof(GUSRIGISTER_T));
			return ;
		}
		else if(GusRegisterTemp.crc == CRC16(0xFFFF, (uint8_t *)&GusRegisterTemp, CrcCheckLength))
		{
			memcpy((uint8_t *)&GusRigister, (uint8_t *)&GusRegisterTemp, sizeof(GUSRIGISTER_T));
			Flash_Write((uint8_t *)&GusRigister, GUSRIGISTER_FLASH_ADDR0, sizeof(GUSRIGISTER_T));
			return ;
		}
//		else
//		{
//			GusRisgster_Init();
//			GusRisgster_Write();
//		}
	}
	else if(GusRigister.times < GusRegisterTemp.times)
	{
		if(GusRegisterTemp.crc == CRC16(0xFFFF, (uint8_t *)&GusRegisterTemp, CrcCheckLength))
		{
			memcpy((uint8_t *)&GusRigister, (uint8_t *)&GusRegisterTemp, sizeof(GUSRIGISTER_T));
			Flash_Write((uint8_t *)&GusRigister, GUSRIGISTER_FLASH_ADDR0, sizeof(GUSRIGISTER_T));
			return ;
		}
		else if(GusRigister.crc == CRC16(0xFFFF, (uint8_t *)&GusRigister, CrcCheckLength))
		{
			Flash_Write((uint8_t *)&GusRigister, GUSRIGISTER_FLASH_ADDR1, sizeof(GUSRIGISTER_T));
			return ;
		}
//		else
//		{
//			GusRisgster_Init();
//			GusRisgster_Write();
//		}
	}
}

void user_set_config_register(void)
{
	GusRigister.Rigister[0x01] = Rigister[0x11];
	GusRigister.Rigister[0x05] = Rigister[0x15];
	GusRigister.Rigister[0x09] = Rigister[0x19];

	GusRigister.Rigister[0x02] = Rigister[0x12];
	GusRigister.Rigister[0x0C] = Rigister[0x1C];
	
	GusRigister.Rigister[0x03] = Rigister[0x13];
	GusRigister.Rigister[0x0A] = Rigister[0x1A];
	
	GusRigister.Rigister[0x06] = Rigister[0x16];
	GusRigister.Rigister[0x07] = Rigister[0x17];
	GusRigister.Rigister[0x08] = Rigister[0x18];
}

void user_config_register(uint8_t* src, uint32_t length)
{
	uint8_t addr = src[0];
	uint32_t i = 0;

	for(i = 0; i < (length - 1); i++)
		Rigister[((addr + i) & 0xFF)] = src[i + 1];

	if(0x5A == Rigister[0x1D])
	{
		user_set_config_register();

		//GusRisgster_Write();
		
		usart_init(GusRigister.Rigister[0x01]);
	}
	else if(0xA5 == Rigister[0x1D])
	{
		user_set_config_register();
		
		usart_init(GusRigister.Rigister[0x01]);
	}
	else if(0x55 == Rigister[0x1D])
	{
		wifi_rest();
	}

	//comSendBuf(COM1, Rigister, sizeof(Rigister));
	Rigister[0x1D] = 0x00;
}
