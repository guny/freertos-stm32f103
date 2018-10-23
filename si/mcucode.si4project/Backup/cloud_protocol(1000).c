
#include <stdint.h>
#include ".\usart\usart.h"
#include ".\cJSON\cJSON.h"
#include ".\cloud\cloud_protocol.h"

#define CLOUD_FRAMER_HEAD0 0xFE
#define CLOUD_FRAMER_HEAD1 0xFE

static int CloudProtocolGetOnePacket(char* dst, unsigned long* length)
{
	
}
