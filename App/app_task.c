#include "app_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "BSP/LED/bsp_led.h"
#include "APP/app_uart.h"
#include "APP/app_sensor.h"
/*================ 任务配置 ================*/

#define LED0_TASK_STACK_SIZE 128U
#define LED0_TASK_PRIORITY 2U

#define LED1_TASK_STACK_SIZE 128U
#define LED1_TASK_PRIORITY 1U

/*================ 任务句柄 ================*/

static TaskHandle_t s_led0_task_handle = NULL;
static TaskHandle_t s_led1_task_handle = NULL;

/*================ 函数声明 ================*/

static void LED0_Task(void *argument);
static void LED1_Task(void *argument);
static void Sensor_TestTask(void *argument);

/*================ 任务初始化 ================*/

/**
 * @brief 创建所有应用任务并启动调度器
 */
void APP_Task_Init(void)
{
    BaseType_t ret;

    /* 创建 LED0 任务 */
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

    /* 创建 LED1 任务 */
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

    // 添加串口任务
    APP_UART_TaskCreate();

    // 添加传感器任务
    APP_Sensor_TaskCreate();
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
    /* 启动任务调度器 */
    vTaskStartScheduler();

    /* 调度器正常启动后不应返回 */
    Error_Handler();
}

void LED0_Task(void *argument)
{
    for (;;)
    {
        BSP_LED_Toggle(BSP_LED_RED);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void LED1_Task(void *argument)
{
    for (;;)
    {
        BSP_LED_Toggle(BSP_LED_GREEN);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* 调试器观察变量 */
static volatile BSP_ADC_Result_t s_sensor_debug;

static volatile bool s_sensor_debug_valid = false;

/**
 * @brief 临时传感器数据测试任务
 */
static void Sensor_TestTask(void *argument)
{
    BSP_ADC_Result_t result;

    (void)argument;

    for (;;)
    {
        /* 模拟GUI任务从队列读取数据 */
        if (APP_Sensor_GetLatest(&result))
        {
            /* 将结果保存到调试变量中 */
            s_sensor_debug = result;

            s_sensor_debug_valid = true;
        }
        else
        {
            s_sensor_debug_valid = false;
        }

        /* 每500ms读取一次队列 */
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}