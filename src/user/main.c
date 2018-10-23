
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

TaskHandle_t LedTask_Handler;
#define LED_TWINKLING_FRE    (50UL)/* 10Hz */
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
TaskHandle_t MgrTask_Handler;
static void vMgr_Task(void *pvParameters)
{
	uint8_t pcWriteBuffer[500];

	for( ; ; )
	{
		printf("=================================================\r\n");
		printf("任务名\t\t任务状态\t优先级\t剩余栈\t任务序号\r\n");
		vTaskList((char *)&pcWriteBuffer);
		printf("%s\r\n", pcWriteBuffer);

		printf("\r\n任务名\t\t运行计数\t\t使用率\r\n");
		vTaskGetRunTimeStats((char *)&pcWriteBuffer);
		printf("%s\r\n", pcWriteBuffer);

		vTaskDelay(1000);
	}
}
#endif

TaskHandle_t WiFTask_Handler;
static void vWiF_Task(void *pvParameters)
{
#if 0
	static uint32_t vWiF_Task_Times = 0;
	for( ; ; )
	{
		printf("vWiF_Task %u\n\r", vWiF_Task_Times++);
		vTaskDelay(20);
	}
#else
	for( ; ; )
	{
		vTaskDelay(20);
	}
#endif
}

TaskHandle_t DatTask_Handler;
static void vDat_Task(void *pvParameters)
{
#if 0
	static uint32_t vDat_Task_Times = 0;
	for( ; ; )
	{
		printf("vDat_Task %u\n\r", vDat_Task_Times++);
		vTaskDelay(20);
	}
#else
	for( ; ; )
	{
		vTaskDelay(20);
	}
#endif
}

TaskHandle_t AppTask_Handler;
static void vApp_Task(void *pvParameters)
{
	taskENTER_CRITICAL();

	//xTaskCreate(vDat_Task, "DatTask", configMINIMAL_STACK_SIZE, NULL, 2, &DatTask_Handler);
	//xTaskCreate(vWiF_Task, "WiFTask", configMINIMAL_STACK_SIZE, NULL, 2, &WiFTask_Handler);

	vTaskDelete(AppTask_Handler);

	taskEXIT_CRITICAL();
}

int main(void)
{
	prvSetupHardware();

#ifdef MGR_TASK_ENABLE
	vSetupSysInfoTest();
#endif

	xTaskCreate(vLED_Task, "LedTask", configMINIMAL_STACK_SIZE, NULL, 1, &LedTask_Handler);
	xTaskCreate(vApp_Task, "AppTask", configMINIMAL_STACK_SIZE, NULL, 1, &AppTask_Handler);

#ifdef MGR_TASK_ENABLE
	xTaskCreate(vMgr_Task, "MgrTask", configMINIMAL_STACK_SIZE * 4, NULL, 4, &MgrTask_Handler);
#endif

	/* Start the scheduler. */
	vTaskStartScheduler();

	for( ; ; );

//	return 0;
}
