/**
 * @file    app_rtos_hooks.c
 * @brief   FreeRTOS 内核异常回调函数（栈溢出与动态内存分配失败捕获）
 * @note    与 FreeRTOSConfig.h 中的以下开关配套使用：
 *          - configCHECK_FOR_STACK_OVERFLOW == 2 -> 触发 vApplicationStackOverflowHook()
 *          - configUSE_MALLOC_FAILED_HOOK   == 1 -> 触发 vApplicationMallocFailedHook()
 *          异常触发后立即关闭全局中断并死循环保留现场，便于调试器查看 g_rtos_fault_code 与调用栈。
 */

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"

/*
 * 故障定位码（可在调试器中直接查看）：
 *   0 = 正常
 *   1 = 发生任务栈溢出（Stack Overflow）
 *   2 = pvPortMalloc 动态内存分配失败（Heap 不足）
 */
volatile uint32_t g_rtos_fault_code = 0U;

/**
 * @brief  任务栈溢出回调（方法 2：在任务切换时检查栈尾魔数是否被踩坏）
 * @param  task      发生溢出的任务句柄
 * @param  task_name 发生溢出的任务名称字符串（可在调试器直接看到哪个任务爆栈）
 * @note   禁止在异常钩子中调用 printf 或阻塞 API，应尽快锁死现场。
 */
void vApplicationStackOverflowHook(
    TaskHandle_t task,
    char *task_name)
{
    (void)task;
    (void)task_name;

    __disable_irq();
    g_rtos_fault_code = 1U;

    for (;;)
    {
        __NOP();
    }
}

/**
 * @brief  FreeRTOS 动态堆内存分配失败回调
 * @note   当 xTaskCreate / xQueueCreate 等内部调用 pvPortMalloc() 返回 NULL 时触发。
 *         若触发此钩子，需检查 configTOTAL_HEAP_SIZE 配置或排查内存泄漏。
 */
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    g_rtos_fault_code = 2U;

    for (;;)
    {
        __NOP();
    }
}