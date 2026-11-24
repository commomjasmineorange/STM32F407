#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * FreeRTOS V9.0.0 应用配置文件
 * 平台：STM32F407ZGT6 / Cortex-M4F / arm-none-eabi-gcc
 * 移植层：portable/GCC/ARM_CM4F
 * 内存管理：heap_4.c
 * 时基：SysTick -> FreeRTOS，TIM6 -> HAL（由 CubeMX 配置）
 * 建议存放位置：Core/Inc/FreeRTOSConfig.h
 */

#include <stdint.h>
#include "stm32f4xx.h" /* SystemCoreClock、__NVIC_PRIO_BITS、CMSIS 内核接口 */

/*==================== 1. 内核调度 ====================*/

/* 抢占式调度：高优先级任务就绪后可以抢占低优先级任务。 */
#define configUSE_PREEMPTION 1

/* 同优先级就绪任务按时间片轮转。 */
#define configUSE_TIME_SLICING 1

/* Cortex-M4 使用优化的最高就绪优先级查找方式。 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

/* 使用 CMSIS 维护的实际 HCLK，启动调度器前完成系统时钟初始化。 */
#define configCPU_CLOCK_HZ (SystemCoreClock)

/* 每秒 1000 个内核节拍，即 1 Tick = 1 ms。 */
#define configTICK_RATE_HZ 1000U

/* 任务优先级数量：有效范围 0~6，数值越大优先级越高。 */
#define configMAX_PRIORITIES 7

/* 空闲任务栈深度，单位为 StackType_t（F407 上为 4 字节）。 */
#define configMINIMAL_STACK_SIZE 128U

/* 任务名称最大长度，包含字符串结束符 '\0'。 */
#define configMAX_TASK_NAME_LEN 16

/* 使用 32 位 Tick 计数，适用于长时间运行。 */
#define configUSE_16_BIT_TICKS 0

/* 空闲任务主动让出 CPU 给同优先级的就绪任务。 */
#define configIDLE_SHOULD_YIELD 1

/* 暂不启用无节拍低功耗模式。 */
#define configUSE_TICKLESS_IDLE 0

/*==================== 2. 内存管理 ====================*/

/* 允许 xTaskCreate、xQueueCreate 等动态分配接口。 */
#define configSUPPORT_DYNAMIC_ALLOCATION 1

/* 暂不启用静态分配，避免额外提供空闲/定时器任务静态内存。 */
#define configSUPPORT_STATIC_ALLOCATION 0

/* heap_4.c 管理的内存池：24 KiB，包含任务栈、TCB、队列等。 */
#define configTOTAL_HEAP_SIZE (24U * 1024U)

/* 由 heap_4.c 自己定义 ucHeap，不需要应用层额外定义。 */
#define configAPPLICATION_ALLOCATED_HEAP 0

/* 暂不为每个任务分配独立 Newlib 重入结构；printf 应统一管理。 */
#define configUSE_NEWLIB_REENTRANT 0

/*==================== 3. 任务同步与通信 ==============*/

/* V9.0.0 支持每任务一个通知值，可用于 UART DMA/IDLE 唤醒任务。 */
#define configUSE_TASK_NOTIFICATIONS 1

/* 互斥锁，支持优先级继承。 */
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1

/* 计数信号量；二值信号量也可正常使用。 */
#define configUSE_COUNTING_SEMAPHORES 1

/* 当前项目暂不使用队列集。 */
#define configUSE_QUEUE_SETS 0

/* 用于调试器识别队列和信号量的注册表容量。 */
#define configQUEUE_REGISTRY_SIZE 8

/*==================== 4. Hook 与异常检测 ==============*/

/* 不启用空闲任务 Hook 与节拍 Hook。 */
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0

/* 栈溢出检测方式 2（并非能检测所有栈溢出）。
 * 必须实现 vApplicationStackOverflowHook(TaskHandle_t, char *)。
 */
#define configCHECK_FOR_STACK_OVERFLOW 2

/* 堆分配失败时调用 vApplicationMallocFailedHook(void)，必须实现。 */
#define configUSE_MALLOC_FAILED_HOOK 1

/*==================== 5. 软件定时器 ===================*/

/* timers.c 必须加入编译。启用软件定时器及其服务任务。 */
#define configUSE_TIMERS 1

/* 定时器服务任务优先级（0~6 范围内）。 */
#define configTIMER_TASK_PRIORITY 3

/* 定时器服务任务的命令队列长度。 */
#define configTIMER_QUEUE_LENGTH 10

/* 定时器服务任务栈深度：256 Word = 1024 字节。 */
#define configTIMER_TASK_STACK_DEPTH 256U

/*==================== 6. 调试与统计 ===================*/

/* 开启任务状态追踪，便于后续查看任务运行状态。 */
#define configUSE_TRACE_FACILITY 1

/* 暂不启用 vTaskList 等格式化统计接口。 */
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* 不统计每个任务的 CPU 运行时间，无需额外统计时基。 */
#define configGENERATE_RUN_TIME_STATS 0

/* 不使用协程，croutine.c 无需加入编译。 */
#define configUSE_CO_ROUTINES 0

/*==================== 7. 可选 API（V9.0.0） ===========*/

#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelay 1

/* 重要：V9.0.0 使用 vTaskDelayUntil()，不是 xTaskDelayUntil()。 */
#define INCLUDE_vTaskDelayUntil 1

#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_eTaskGetState 1
#define INCLUDE_xTaskGetIdleTaskHandle 1

/* 允许向定时器服务任务投递延后执行函数；事件组 FromISR 接口需要它。 */
#define INCLUDE_xTimerPendFunctionCall 1

/*==================== 8. Cortex-M4 中断优先级 ==========*/

/* STM32F407 实现 4 位 NVIC 优先级。使用 NVIC_PRIORITYGROUP_4。 */
#define configPRIO_BITS __NVIC_PRIO_BITS

/* 数值越小，硬件中断优先级越高。 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* 将 HAL/CMSIS 使用的优先级数字转换成寄存器中的高位编码。 */
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))

/* F407 上为 0x50：FreeRTOS 临界区的 BASEPRI 屏蔽阈值。
 * NVIC 优先级 0~4：不能调用 FreeRTOS API。
 * NVIC 优先级 5~15：可以调用允许在 ISR 使用的 FromISR 接口。
 * USART1 与 RX DMA 建议使用相同抢占优先级 6，子优先级 0。
 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))

/*==================== 9. 内核断言 =====================*/

/* 断言失败即关闭中断并停在现场，可用调试器检查调用栈。
 * 用于开发调试；不在断言中 printf 或 HAL_Delay。
 */
#define configASSERT(x)      \
    do                       \
    {                        \
        if (!(x))            \
        {                    \
            __disable_irq(); \
            for (;;)         \
            {                \
            }                \
        }                    \
    } while (0)

/*==================== 10. 内核异常入口映射 =============*/

/* port.c 直接提供这三个 CMSIS 标准异常处理函数。
 * 不要再在 stm32f4xx_it.c 中定义同名函数；
 * SVC 与 PendSV 尤其不能使用普通 C 函数包装调用。
 * HAL 的 1 ms 时基必须另由 TIM6 等未占用的定时器维护。
 */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
