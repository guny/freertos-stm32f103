
#include <stdint.h>
#include <stdio.h>

#include "stm32f10x_gpio.h"
#include "..\led\led.h"

#define GPIO_LED1_CLK    RCC_APB2Periph_GPIOB
#define GPIO_LED1_PORT   GPIOB
#if 0
#define GPIO_LED1_PIN    GPIO_Pin_9
#else
#define GPIO_LED1_PIN    GPIO_Pin_7
#endif

typedef enum {LED_OFF = 0, LED_ON = ~LED_OFF} LED_STATUS_Type;
LED_STATUS_Type LedStatus = LED_OFF;

static void _led_on(void);
static void _led_off(void);

void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(GPIO_LED1_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_LED1_PIN;
	GPIO_Init(GPIO_LED1_PORT, &GPIO_InitStructure);

	LedStatus = LED_OFF;
//	_led_on();
	_led_off();
}

static void _led_on(void)
{
	LedStatus = LED_ON;
	GPIO_SetBits(GPIO_LED1_PORT,GPIO_LED1_PIN);
}

static void _led_off(void)
{
	LedStatus = LED_OFF;
	GPIO_ResetBits(GPIO_LED1_PORT,GPIO_LED1_PIN);
}

void led_twinkling(void)
{
	if(LED_OFF == LedStatus)
		_led_on();
	else
		_led_off();		
}
