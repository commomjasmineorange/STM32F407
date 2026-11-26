/**
 * @file    app_sensor.c
 * @brief   板载 ADC 传感器周期采集任务（邮箱队列模式：只保留最新快照）
 * @note    架构要点：
 *          1. Sensor_Task 每 200ms 读取一次由 DMA 双半缓冲维护的 ADC 均值快照。
 *          2. 使用长度为 1 的队列 s_sensor_queue + xQueueOverwrite() 覆盖写入，
 *             消费方通过 APP_Sensor_GetLatest()（xQueuePeek）非阻塞读取而不删除数据，
 *             适用于 LCD 显示与状态观察等“只关心最新值”的场景。
 */

#include "app_sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"

/*================ 任务配置 ================*/

/* 任务栈深度：256 Word，即1024字节 */
#define SENSOR_TASK_STACK_SIZE 256U

/* 任务优先级 */
#define SENSOR_TASK_PRIORITY 2U

/* 传感器任务运行周期 */
#define SENSOR_TASK_PERIOD_MS 200U

/* 队列长度，只保存最新一次传感器数据 */
#define SENSOR_QUEUE_LENGTH 1U

/*================ 私有变量 ================*/

/* 传感器数据队列句柄 */
static QueueHandle_t s_sensor_queue = NULL;

/*================ 函数声明 ================*/

static void Sensor_Task(void *argument);

/*================ 创建任务 ================*/

/**
 * @brief 创建传感器数据队列和传感器任务
 *
 * @note 在启动调度器之前调用一次
 */
void APP_Sensor_TaskCreate(void)
{
    BaseType_t ret;
    s_sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(BSP_ADC_Result_t));
    if (s_sensor_queue == NULL)
    {
        Error_Handler();
    }
    ret = xTaskCreate(Sensor_Task,
                      "SENSOR",
                      SENSOR_TASK_STACK_SIZE,
                      NULL,
                      SENSOR_TASK_PRIORITY,
                      NULL);
    if (ret != pdPASS)
    {
        Error_Handler();
    }
}

/*================ 传感器任务 ================*/

/**
 * @brief 周期性读取ADC数据，并发布到队列
 */
static void Sensor_Task(void *argument)
{
    (void)argument;
    BSP_ADC_Result_t result;
    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();
    for (;;)
    {

        if (BSP_ADC_GetResult(&result))
        {
            /*
             * 将整个结构体复制到队列中。
             * 如果队列已经有旧数据，直接覆盖。
             */
            xQueueOverwrite(s_sensor_queue, &result);
        }

        /*
         * 以200ms为目标周期运行。
         * 当前任务进入阻塞态，CPU执行其他任务。
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/*================ 数据读取接口 ================*/

/**
 * @brief 获取最近一次发布的ADC数据
 */
bool APP_Sensor_GetLatest(BSP_ADC_Result_t *result)
{
    if ((result == NULL) || (s_sensor_queue == NULL))
    {
        return false;
    }
    /*
     * 从队列复制数据，但不删除数据。
     *
     * 第三个参数为0：
     * 如果队列为空，立即返回，不阻塞等待。
     */
    if (xQueuePeek(s_sensor_queue, result, 0) == pdPASS)
    {
        return true;
    }
    return false;
}