#include "app_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/*================ 任务配置 ================*/

#define MONITOR_TASK_STACK_SIZE 256U
#define MONITOR_TASK_PRIORITY 1U
#define MONITOR_TASK_PERIOD_MS 1000U

/* 最大可监控任务数量 */
#define MONITOR_MAX_TASKS 16U

/*================ 监控数据 ================*/

static TaskStatus_t s_task_status[MONITOR_MAX_TASKS];

/* 调试器观察变量 */
static volatile UBaseType_t s_task_count = 0;

static volatile size_t s_free_heap = 0;

static volatile size_t s_min_free_heap = 0;

static volatile uint32_t s_snapshot_fail = 0;

void APP_Monitor_TaskCreate(void);
static void Monitor_Task(void *argument)
{
    (void)argument;
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        /* 1. 获取当前剩余Heap */
        s_free_heap = xPortGetFreeHeapSize();

        /* 2. 获取历史最小剩余Heap */
        s_min_free_heap = xPortGetMinimumEverFreeHeapSize();

        /* 3. 获取所有任务的状态快照 */
        s_task_count = uxTaskGetSystemState(s_task_status, MONITOR_MAX_TASKS, NULL);

        /* 数组空间不足时可能返回0 */
        if (s_task_count == 0U)
        {
            s_snapshot_fail++;
        }

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(MONITOR_TASK_PERIOD_MS));
    }
}

/*任务创建*/
void APP_Monitor_TaskCreate(void)
{
    BaseType_t ret;

    ret = xTaskCreate(Monitor_Task, "MONITOR", MONITOR_TASK_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, NULL);

    if (ret != pdPASS)
    {
        Error_Handler();
    }
}
