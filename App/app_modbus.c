#include "app_modbus.h"
#include "BSP/RS485/bsp_rs485.h"
#include "Protocol/modbus_rtu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <stdio.h>
// #include <stdint.h>

/*================ 任务与轮询配置 ================*/
#define MODBUS_TASK_STACK_SIZE 512U /* 512 Word = 2048 字节（用 printf 栈要给够） */
#define MODBUS_TASK_PRIORITY 3U     /* 采集通信任务，优先级高于 LED/显示 */
#define MODBUS_POLL_PERIOD_MS 1000U /* 每 1 秒轮询一次 */
#define MODBUS_RESP_TIMEOUT_MS 200U /* 等从站回复的最大超时时间 */

/* 测试目标：读 1 号从站，起始地址 0，连续读 2 个保持寄存器 */
#define MODBUS_SLAVE_ID 1U
#define MODBUS_START_ADDR 0U
#define MODBUS_REG_QTY 2U

/* 保存任务句柄，供 RS485 接收中断发送任务通知 */
static TaskHandle_t volatile s_modbus_task_handle = NULL;

static void Modbus_Task(void *argument);

/*================ 创建任务 ================*/
void APP_Modbus_TaskCreate(void)
{
    BaseType_t ret = xTaskCreate(
        Modbus_Task,
        "MODBUS",
        MODBUS_TASK_STACK_SIZE,
        NULL,
        MODBUS_TASK_PRIORITY,
        NULL);
    if (ret != pdPASS)
    {
        Error_Handler();
    }
}

/*================ RS485 接收中断钩子 ================*/
/*
 * 重写 bsp_rs485.c 中的弱函数：
 * USART2 收到一包数据触发 IDLE 中断时，唤醒 Modbus_Task。
 */
void BSP_RS485_RxEventHook(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskHandle_t handle = s_modbus_task_handle;
    if (handle != NULL)
    {
        vTaskNotifyGiveFromISR(handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* 设为 1 开启上电协议自测；以后接上 485 模块后改成 0 即可关闭 */
#define MODBUS_ENABLE_SELFTEST 1

#if MODBUS_ENABLE_SELFTEST
static void Modbus_RunSelfTest(void)
{
    uint16_t regs[2] = {0};
    uint8_t ex_code = 0U;
    Modbus_Status_t st;

    printf("\r\n=== Modbus Protocol SelfTest Start ===\r\n");

    /*
     * 用例 1：合法 0x03 响应帧
     * 从站=1, 功能码=0x03, 字节数=4, reg0=0x0064(100), reg1=0x00C8(200)
     * 精确 CRC16 = 0x7ABA（低字节 0xBA 在前，高字节 0x7A 在后）
     */
    const uint8_t ok_frame[] = {0x01, 0x03, 0x04, 0x00, 0x64, 0x00, 0xC8, 0xBA, 0x7A};
    st = Modbus_ParseReadHolding(ok_frame, sizeof(ok_frame), 1U, 2U, regs, &ex_code);
    if ((st == MODBUS_OK) && (regs[0] == 100U) && (regs[1] == 200U))
    {
        printf("[Test 1] Normal Frame  : PASS (reg0=%u, reg1=%u)\r\n", regs[0], regs[1]);
    }
    else
    {
        printf("[Test 1] Normal Frame  : FAIL (st=%d, reg0=%u, reg1=%u)\r\n", (int)st, regs[0], regs[1]);
    }

    /*
     * 用例 2：故意篡改最后 1 字节（0x7A -> 0x7B），测试 CRC 校验能否拦住坏帧
     */
    const uint8_t bad_crc_frame[] = {0x01, 0x03, 0x04, 0x00, 0x64, 0x00, 0xC8, 0xBA, 0x7B};
    st = Modbus_ParseReadHolding(bad_crc_frame, sizeof(bad_crc_frame), 1U, 2U, regs, &ex_code);
    if (st == MODBUS_ERR_CRC)
    {
        printf("[Test 2] Bad CRC Frame : PASS (caught MODBUS_ERR_CRC)\r\n");
    }
    else
    {
        printf("[Test 2] Bad CRC Frame : FAIL (st=%d)\r\n", (int)st);
    }

    /*
     * 用例 3：从站异常响应帧（功能码 0x83，异常码 0x02 非法地址，CRC=0xF1C0）
     */
    const uint8_t ex_frame[] = {0x01, 0x83, 0x02, 0xC0, 0xF1};
    st = Modbus_ParseReadHolding(ex_frame, sizeof(ex_frame), 1U, 2U, regs, &ex_code);
    if ((st == MODBUS_ERR_EXCEPTION) && (ex_code == 0x02U))
    {
        printf("[Test 3] Exception     : PASS (ex_code=0x%02X)\r\n", ex_code);
    }
    else
    {
        printf("[Test 3] Exception     : FAIL (st=%d, ex=0x%02X)\r\n", (int)st, ex_code);
    }

    printf("=== Modbus Protocol SelfTest End ===\r\n\r\n");
}
#endif
/*================ Modbus 轮询任务 ================*/
static void Modbus_Task(void *argument)
{
    (void)argument;
    uint8_t tx_buf[MODBUS_READ_REQ_SIZE];
    uint8_t rx_buf[64];
    uint16_t regs[MODBUS_REG_QTY];
    uint8_t ex_code = 0U;
    uint32_t ok_cnt = 0U;
    uint32_t err_cnt = 0U;

    /* 调度器启动后再开放中断通知 */
    s_modbus_task_handle = xTaskGetCurrentTaskHandle();

    /* 注意：last_wake_time 必须在循环外初始化一次，vTaskDelayUntil 才能保证精确周期 */
    TickType_t last_wake_time = xTaskGetTickCount();

    /* 期望收到的正常响应长度：5 + 2 * 寄存器数 = 9 字节 */
    const uint16_t expect_len = (uint16_t)(5U + 2U * MODBUS_REG_QTY);
#if MODBUS_ENABLE_SELFTEST
    Modbus_RunSelfTest(); /* 上电后只跑一次，把 3 个用例结果打到 USART1 */
#endif

    for (;;)
    {
        /* 1. 检查 RS485 是否有硬件错误需要恢复 */
        BSP_RS485_Service();
        /* 2. 构造 0x03 请求帧（8 字节） */
        size_t tx_len = Modbus_BuildReadHolding(MODBUS_SLAVE_ID, MODBUS_START_ADDR, MODBUS_REG_QTY, tx_buf, sizeof(tx_buf));
        if (tx_len > 0U)
        {
            /* 3. 发请求前清空残留的旧接收数据和旧任务通知 */
            BSP_RS485_FlushRx();
            (void)ulTaskNotifyTake(pdTRUE, 0U);

            /* 4. 通过 RS485 发送请求（内部自动切发->发完->切收） */
            if (BSP_RS485_Send(tx_buf, (uint16_t)tx_len) == HAL_OK)
            {
                /* 5. 在超时时间内收集响应字节（防止拆包） */
                uint16_t rx_len = 0U;
                TickType_t start_tick = xTaskGetTickCount();
                while (rx_len < expect_len)
                {
                    TickType_t elapsed = xTaskGetTickCount() - start_tick;
                    if (elapsed >= pdMS_TO_TICKS(MODBUS_RESP_TIMEOUT_MS))
                    {
                        break; /* 等超时了 */
                    }
                    /* 挂起等待中断通知，不占 CPU */
                    TickType_t remain = pdMS_TO_TICKS(MODBUS_RESP_TIMEOUT_MS) - elapsed;
                    (void)ulTaskNotifyTake(pdTRUE, remain);

                    /* 把环形缓冲区里新到的字节读出来追加到 rx_buf */
                    rx_len += BSP_RS485_Read(&rx_buf[rx_len], (uint16_t)(sizeof(rx_buf) - rx_len));
                }
                if (rx_len == 0U)
                {
                    err_cnt++;
                    printf("[MB] Timeout! ok=%lu err=%lu\r\n", ok_cnt, err_cnt);
                }
                else
                {
                    Modbus_Status_t st = Modbus_ParseReadHolding(
                        rx_buf, rx_len,
                        MODBUS_SLAVE_ID, MODBUS_REG_QTY,
                        regs, &ex_code);
                    if (st == MODBUS_OK)
                    {
                        ok_cnt++;
                        printf("[MB] OK! reg0=%u reg1=%u (ok=%lu err=%lu)\r\n",
                               regs[0], regs[1], ok_cnt, err_cnt);
                    }
                    else if (st == MODBUS_ERR_EXCEPTION)
                    {
                        err_cnt++;
                        printf("[MB] Slave Exception=0x%02X (ok=%lu err=%lu)\r\n",
                               ex_code, ok_cnt, err_cnt);
                    }
                    else
                    {
                        err_cnt++;
                        printf("[MB] Parse Err=%d len=%u (ok=%lu err=%lu)\r\n",
                               (int)st, rx_len, ok_cnt, err_cnt);
                    }
                }
            }
        }
        /* 7. 精确 1 秒周期等待下一次轮询 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MODBUS_POLL_PERIOD_MS));
    }
}