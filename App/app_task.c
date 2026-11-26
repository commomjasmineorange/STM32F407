/**
 * @file    app_task.c
 * @brief   FreeRTOS 应用任务总入口：统一创建所有业务任务并启动调度器
 * @note    架构角色：
 *          1. 由 main() 在完成全部 HAL/BSP 硬件初始化后调用 APP_Task_Init()。
 *          2. 创建 LED 心跳任务、USART1 调试任务、ADC 采集任务、RS485 Modbus 主站任务、
 *             SPI Flash 存储任务及系统监控任务。
 *          3. 调用 vTaskStartScheduler() 后 CPU 控制权完全移交 FreeRTOS，不再返回。
 */

#include "app_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "BSP/LED/bsp_led.h"
#include "APP/app_uart.h"
#include "APP/app_sensor.h"
#include "App/app_monitor.h"
#include "App/app_modbus.h"
#include "App/app_storage.h"

/*================ 任务配置（栈单位：Word，F407 上 1 Word = 4 字节） ================*/

#define LED0_TASK_STACK_SIZE 128U /* 512 字节：仅翻转 GPIO，栈需求极小 */
#define LED0_TASK_PRIORITY   2U   /* 优先级 2（数值越大优先级越高） */

#define LED1_TASK_STACK_SIZE 128U /* 512 字节 */
#define LED1_TASK_PRIORITY   1U   /* 优先级 1 */

/*================ 任务句柄 ================*/

static TaskHandle_t s_led0_task_handle = NULL;
static TaskHandle_t s_led1_task_handle = NULL;

/*================ 内部函数声明 ================*/

static void LED0_Task(void *argument);
static void LED1_Task(void *argument);
static void Sensor_TestTask(void *argument);

/*================ 任务初始化 ================*/

/**
 * @brief  创建所有应用任务并启动 FreeRTOS 调度器
 * @note   若任一任务或队列因堆内存不足创建失败，将进入 Error_Handler() 停机定位。
 */
void APP_Task_Init(void)
{
    BaseType_t ret;

    /* 1. 创建红灯心跳任务（200ms 翻转一次，直观指示调度器正常运行） */
    ret = xTaskCreate(
        LED0_Task,
        "LED0",
        LED0_TASK_STACK_SIZE,
        NULL,
        LED0_TASK_PRIORITY,
        &s_led0_task_handle);
    if (ret != pdPASS)
    {
        Error_Handler();
    }

    /* 2. 创建绿灯心跳任务（500ms 翻转一次） */
    ret = xTaskCreate(
        LED1_Task,
        "LED1",
        LED1_TASK_STACK_SIZE,
        NULL,
        LED1_TASK_PRIORITY,
        &s_led1_task_handle);
    if (ret != pdPASS)
    {
        Error_Handler();
    }

    /* 3. 创建 USART1 调试串口任务（优先级 3，由 DMA+IDLE 中断通过任务通知唤醒） */
    APP_UART_TaskCreate();

    /* 4. 创建 ADC 光敏采集任务（优先级 2，每 200ms 刷新一次最新值队列） */
    APP_Sensor_TaskCreate();

    /* 5. 创建传感器消费测试任务（优先级 1，周期读取最新值并投递到存储队列） */
    if (xTaskCreate(
            Sensor_TestTask,
            "SensorTest",
            256U,
            NULL,
            1U,
            NULL) != pdPASS)
    {
        Error_Handler();
    }

    /* 6. 创建系统运行监控任务（优先级 1，每 1s 统计剩余堆与任务状态快照） */
    APP_Monitor_TaskCreate();

    /* 7. 创建 RS485 Modbus RTU 主站轮询任务（优先级 3，每 1s 轮询一次从站） */
    APP_Modbus_TaskCreate();

    /* 8. 创建 SPI Flash 掉电日志任务（优先级 2，单一写者负责异步落盘与开机恢复） */
    APP_Storage_TaskCreate();

    /* 9. 启动 FreeRTOS 调度器（开启 SysTick 与 PendSV，开始多任务抢占调度） */
    vTaskStartScheduler();

    /* 正常情况下调度器永不返回；若执行到此，通常说明 Idle/Timer 任务堆分配失败 */
    Error_Handler();
}

/**
 * @brief  LED0 红灯闪烁任务（周期 200ms）
 */
static void LED0_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        BSP_LED_Toggle(BSP_LED_RED);
        vTaskDelay(pdMS_TO_TICKS(200U));
    }
}

/**
 * @brief  LED1 绿灯闪烁任务（周期 500ms）
 */
static void LED1_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        BSP_LED_Toggle(BSP_LED_GREEN);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

/* 调试器实时观察变量（volatile 防止编译器优化掉写入操作） */
static volatile BSP_ADC_Result_t s_sensor_debug;
static volatile bool s_sensor_debug_valid = false;

/**
 * @brief  传感器消费与存储联调任务（周期 2000ms）
 * @note   1. 通过 APP_Sensor_GetLatest() 非阻塞窥视 s_sensor_queue 中的最新 ADC 快照。
 *         2. 通过 APP_Storage_Push() 将记录非阻塞投递到 s_storage_queue，
 *            由后台 Storage_Task 异步写入 SPI Flash，避免擦写耗时阻塞本任务。
 */
static void Sensor_TestTask(void *argument)
{
    BSP_ADC_Result_t result;

    (void)argument;

    for (;;)
    {
        /* 读取最新一次 ADC 采样结果（xQueuePeek 只读不删） */
        if (APP_Sensor_GetLatest(&result))
        {
            s_sensor_debug = result;
            s_sensor_debug_valid = true;

            /* 每 2000ms 向存储队列投递一条测试日志（后续可替换为 result.raw / result.voltage_mv） */
            APP_Storage_Push(LOG_SRC_ADC, 123U, 456U);
        }
        else
        {
            s_sensor_debug_valid = false;
        }

        /* 休眠 2000ms，让出 CPU 并控制 Flash 写入频率 */
        vTaskDelay(pdMS_TO_TICKS(2000U));
    }
}