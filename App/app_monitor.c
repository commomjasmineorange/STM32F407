/**
 * @file    app_monitor.c
 * @brief   FreeRTOS 系统运行健康监控任务实现
 * @note    周期性采集以下诊断指标（可在调试器 Watch 窗口实时观察）：
 *          1. s_free_heap     ：当前 heap_4 剩余空闲字节数。
 *          2. s_min_free_heap ：上电以来历史最小剩余字节数（用于评估内存安全裕量）。
 *          3. s_task_status[] ：每个任务的运行状态、优先级与栈高水位（usStackHighWaterMark，
 *                               单位为 Word，越接近 0 说明栈越接近溢出）。
 *          依赖 FreeRTOSConfig.h 中 configUSE_TRACE_FACILITY == 1。
 */

#include "app_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/*================ 任务配置 ================*/

#define MONITOR_TASK_STACK_SIZE 256U  /* 256 Word = 1024 字节 */
#define MONITOR_TASK_PRIORITY   1U    /* 低优先级：监控不影响通信与采集实时性 */
#define MONITOR_TASK_PERIOD_MS  1000U /* 每 1000ms 刷新一次系统状态快照 */

/* 最大可同时记录的任务数量（需大于系统实际任务数，含 Idle 与 Tmr Svc） */
#define MONITOR_MAX_TASKS       16U

/*================ 监控状态快照（供调试器观察） ================*/

static TaskStatus_t s_task_status[MONITOR_MAX_TASKS];

static volatile UBaseType_t s_task_count    = 0U; /* 当前系统活跃任务总数 */
static volatile size_t      s_free_heap     = 0U; /* 当前剩余动态堆大小（字节） */
static volatile size_t      s_min_free_heap = 0U; /* 历史最小剩余堆大小（字节） */
static volatile uint32_t    s_snapshot_fail = 0U; /* 快照数组容量不足导致的失败次数 */

/*================ 内部函数声明 ================*/

static void Monitor_Task(void *argument);

/*================ 任务实现 ================*/

/**
 * @brief  系统健康监控主循环（周期 1000ms）
 */
static void Monitor_Task(void *argument)
{
    (void)argument;

    /* 在循环外记录初始唤醒节拍，配合 vTaskDelayUntil 实现无漂移周期采样 */
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        /* 1. 查询当前 heap_4 剩余空闲堆大小 */
        s_free_heap = xPortGetFreeHeapSize();

        /* 2. 查询上电以来历史最小剩余堆大小（水位越低风险越高） */
        s_min_free_heap = xPortGetMinimumEverFreeHeapSize();

        /*
         * 3. 获取所有任务的状态快照（包含每个任务的栈高水位 usStackHighWaterMark）。
         *    注意：uxTaskGetSystemState() 内部会短暂挂起调度器，不宜过高频率调用。
         */
        s_task_count = uxTaskGetSystemState(s_task_status, MONITOR_MAX_TASKS, NULL);

        /* 若系统任务总数超过 MONITOR_MAX_TASKS，接口返回 0 */
        if (s_task_count == 0U)
        {
            s_snapshot_fail++;
        }

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(MONITOR_TASK_PERIOD_MS));
    }
}

/**
 * @brief  创建系统监控任务
 */
void APP_Monitor_TaskCreate(void)
{
    BaseType_t ret = xTaskCreate(
        Monitor_Task,
        "MONITOR",
        MONITOR_TASK_STACK_SIZE,
        NULL,
        MONITOR_TASK_PRIORITY,
        NULL);

    if (ret != pdPASS)
    {
        Error_Handler();
    }
}
