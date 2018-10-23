
#include <stdint.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"

#include "stm32f10x.h"

#include ".\led\led.h"
#include ".\usart\usart.h"
#include ".\time\time.h"
#include ".\include\dataswitch.h"

#ifdef MGR_TASK_ENABLE
/* 定时器频率，50us一次中断 */
#define  timerINTERRUPT_FREQUENCY	20000

/* 中断优先级 */
#define  timerHIGHEST_PRIORITY		1

/* 被系统调用 */
volatile uint32_t ulHighFrequencyTimerTicks = 0UL;

/*
*********************************************************************************************************
*	函 数 名: vSetupTimerTest
*	功能说明: 创建定时器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void vSetupSysInfoTest(void)
{
	bsp_SetTIMforInt(TIM4, timerINTERRUPT_FREQUENCY, timerHIGHEST_PRIORITY, 0);
}

/*
*********************************************************************************************************
*	函 数 名: TIM6_IRQHandler
*	功能说明: TIM6中断服务程序。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void TIM4_IRQHandler( void )
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		ulHighFrequencyTimerTicks++;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
#endif

static void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

static void SysTick_Configuration(void)
{
  /* Select AHB clock(HCLK) as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

static void prvSetupHardware(void)
{
	SysTick_Configuration();

	NVIC_Configuration();

	usart_init(0x07);

	led_init();
}

#define LED_TWINKLING_FRE    (100UL)/* 10Hz */
static void vLED_Task(void* pvParameters)
{
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();
	for( ; ; )
	{
		led_twinkling();

		vTaskDelayUntil(&xLastWakeTime, LED_TWINKLING_FRE);
	}
}

#ifdef MGR_TASK_ENABLE
static void vMgr_Task(void *pvParameters)
{
	uint8_t pcWriteBuffer[500];
	uint8_t ucCMD;

	for( ; ; )
	{
		if (comGetChar(COM1, &ucCMD))
		{
			if('K' == ucCMD || 'k' == ucCMD)
			{
				printf("=================================================\r\n");
				printf("任务名      任务状态 优先级   剩余栈 任务序号\r\n");
				vTaskList((char *)&pcWriteBuffer);
				printf("%s\r\n", pcWriteBuffer);

				printf("\r\n任务名       运行计数         使用率\r\n");
				vTaskGetRunTimeStats((char *)&pcWriteBuffer);
				printf("%s\r\n", pcWriteBuffer);
			}
		}

		vTaskDelay(20);
	}
}
#endif

static void vWiF_Task(void *pvParameters)
{
	for( ; ; )
	{
		vTaskDelay(20);
	}
}

static void vDat_Task(void *pvParameters)
{
	for( ; ; )
	{
		user_main_lcm_dataswitch_process();

		vTaskDelay(20);
	}
}

TaskHandle_t LedTask_Handler;
TaskHandle_t WifTask_Handler;
TaskHandle_t DatTask_Handler;

int main(void)
{
	prvSetupHardware();

#ifdef MGR_TASK_ENABLE
	vSetupSysInfoTest();
#endif

	xTaskCreate(vLED_Task, "LedTask", configMINIMAL_STACK_SIZE, NULL, 1, &LedTask_Handler);
	xTaskCreate(vWiF_Task, "WiFTask", configMINIMAL_STACK_SIZE, NULL, 2, &WifTask_Handler);
	xTaskCreate(vDat_Task, "DatTask", configMINIMAL_STACK_SIZE, NULL, 2, &DatTask_Handler);
#ifdef MGR_TASK_ENABLE
	xTaskCreate(vMgr_Task, "MgrTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
#endif

	/* Start the scheduler. */
	vTaskStartScheduler();

	for( ; ; );

//	return 0;
}
