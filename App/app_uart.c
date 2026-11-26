/**
 * @file    app_uart.c
 * @brief   USART1 调试串口接收任务（基于弱函数钩子 + FreeRTOS 任务通知）
 * @note    架构要点：
 *          1. 重写 bsp_uart.c 中的弱函数 BSP_UART_RxEventHook()，在 DMA/IDLE 中断中
 *             调用 vTaskNotifyGiveFromISR() 唤醒本任务，避免轮询浪费 CPU。
 *          2. 平时休眠在 ulTaskNotifyTake(pdTRUE, 10ms)：有数据瞬间唤醒处理，
 *             无数据时每 10ms 巡检一次 BSP_UART_Service() 以执行硬件错误自恢复。
 */

#include "app_uart.h"
#include "BSP/UART/bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/*================ 任务配置 ================*/

/* 任务栈深度：512 Word，即2048字节 */
#define UART_TASK_STACK_SIZE 512U

/* 任务优先级 */
#define UART_TASK_PRIORITY 3U

/* 单次读取数据的最大长度 */
#define UART_READ_BUF_SIZE 128U

/* 串口错误恢复检查周期 */
#define UART_SERVICE_PERIOD_MS 10U

/*================ 私有变量 ================*/

/*
 * UART任务句柄
 * 由任务启动后赋值，中断中读取。
 */
static TaskHandle_t volatile s_uart_task_handle = NULL;

/*================ 函数声明 ================*/

static void UART_Task(void *argument);

/*================ 任务创建 ================*/

void APP_UART_TaskCreate(void)
{
    BaseType_t ret;

    ret = xTaskCreate(
        UART_Task,
        "UART",
        UART_TASK_STACK_SIZE,
        NULL,
        UART_TASK_PRIORITY,
        NULL);

    if (ret != pdPASS)
    {
        Error_Handler();
    }
}

/*================ 接收事件回调 ================*/

/**
 * @brief 重写UART BSP中的弱函数
 * @note  此函数在中断上下文中执行
 */
void BSP_UART_RxEventHook(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /*
     * 防止任务尚未启动时，
     * UART中断提前进入此函数。
     */
    TaskHandle_t handle = s_uart_task_handle;

    if (handle != NULL)
    {
        /* 增加UART任务的通知值 */
        vTaskNotifyGiveFromISR(
            handle,
            &xHigherPriorityTaskWoken);

        /* 判断是否需要触发任务切换 */
        portYIELD_FROM_ISR(
            xHigherPriorityTaskWoken);
    }
}

/*================ 任务函数 ================*/

static void UART_Task(void *argument)
{
    uint8_t rx_buf[UART_READ_BUF_SIZE];
    uint16_t rx_len;

    (void)argument;

    /* 调度器启动后再开放中断通知 */
    s_uart_task_handle = xTaskGetCurrentTaskHandle();

    for (;;)
    {
        /* 执行原有UART异常检测和恢复 */
        BSP_UART_Service();

        /*
         * 读取软件环形缓冲区，
         * 直到当前缓冲区中的数据全部处理完成。
         */
        while (1)
        {
            rx_len = BSP_UART_Read(
                rx_buf,
                sizeof(rx_buf));

            if (rx_len == 0U)
            {
                break;
            }

            /* 当前阶段只进行串口回显测试 */
            BSP_UART_Send(rx_buf, rx_len);
        }

        /*
         * 等待中断发送任务通知。
         *
         * pdTRUE：获取通知后将通知值清零。
         *
         * 最大等待10ms：
         * 即使没有收到数据，也能定期检查UART错误。
         */
        ulTaskNotifyTake(
            pdTRUE,
            pdMS_TO_TICKS(UART_SERVICE_PERIOD_MS));
    }
}