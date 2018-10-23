
#ifndef __CLOUD_PROTOCOL_H__
#define __CLOUD_PROTOCOL_H__

#ifdef __cplusplus
 extern "C" {
#endif

typedef enum {
	MCUESP_ESP_TICK_ASK = 0x00,
	MCUESP_MCU_TICK_ACK,
	MCUESP_MCU_WIFI_CONFIG,
	MCUESP_ESP_WIFI_ACK,
	MCUESP_ESP_DATA_OUT,
	MCUESP_MCU_DATA_IN_ACK,
	MCUESP_MCU_DATA_OUT,
	MCUESP_ESP_DATA_IN_ACK
} MCUESP_SWITCH_CMD_T;


void cloud_process(void);
void wifi_rest(void);

#ifdef __cplusplus
 }
#endif

#endif //__CLOUD_PROTOCOL_H__
