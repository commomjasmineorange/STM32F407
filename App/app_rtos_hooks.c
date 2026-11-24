#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"

volatile uint32_t g_rtos_fault_code = 0;

/* 任务栈溢出检测 */
void vApplicationStackOverflowHook(
    TaskHandle_t task,
    char *task_name)
{
    (void)task;
    (void)task_name;

    __disable_irq();

    g_rtos_fault_code = 1;

    for (;;)
    {
        __NOP();
    }
}

/* 动态内存分配失败 */
void vApplicationMallocFailedHook(void)
{
    __disable_irq();

    g_rtos_fault_code = 2;

    for (;;)
    {
        __NOP();
    }
}