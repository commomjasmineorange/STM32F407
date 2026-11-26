/**
 * @file    app_modbus.c
 * @brief   RS485 Modbus RTU 主站轮询任务（基于 FreeRTOS 任务通知 + DMA/IDLE 异步接收）
 * @note    工作流程：
 *          1. 重写 bsp_rs485.c 的弱函数 BSP_RS485_RxEventHook()，在 USART2 IDLE/DMA 中断中
 *             调用 vTaskNotifyGiveFromISR() 唤醒本任务，避免 CPU 轮询忙等。
 *          2. 主循环每 1000ms 构造一帧 0x03 读保持寄存器请求 -> 清空残留缓冲与通知 ->
 *             发送请求并自动切回接收 -> 休眠等待回复（超时 200ms，支持拆包拼接） ->
 *             CRC16 校验与大端序解析。
 */

#include "app_modbus.h"
#include "BSP/RS485/bsp_rs485.h"
#include "Protocol/modbus_rtu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <stdio.h>

/*================ 任务与轮询配置 ================*/

#define MODBUS_TASK_STACK_SIZE 512U  /* 512 Word = 2048 字节（调用 printf 需预留充足栈空间） */
#define MODBUS_TASK_PRIORITY   3U    /* 较高优先级：保证通信响应及时，高于存储/监控任务 */
#define MODBUS_POLL_PERIOD_MS  1000U /* 轮询周期：每 1000ms 采集一次 */
#define MODBUS_RESP_TIMEOUT_MS 200U  /* 从站应答超时上限：200ms */

/* 当前测试目标：读取 1 号从站、起始地址 0x0000 起的 2 个保持寄存器 */
#define MODBUS_SLAVE_ID        1U
#define MODBUS_START_ADDR      0U
#define MODBUS_REG_QTY         2U

/* 设为 1 开启上电协议解析自测（无硬件时验证组帧/CRC/异常帧）；正式联调可改为 0 */
#define MODBUS_ENABLE_SELFTEST 1

/* 保存本任务句柄，供 RS485 接收中断发送任务通知（volatile 防止跨上下文优化） */
static TaskHandle_t volatile s_modbus_task_handle = NULL;

static void Modbus_Task(void *argument);

/*================ 创建任务 ================*/

/**
 * @brief  创建 Modbus RTU 主站轮询任务
 */
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

/**
 * @brief  重写 bsp_rs485.c 中的弱函数钩子（运行在 USART2/DMA 中断上下文）
 * @note   BSP 层收到数据并搬运进软件环形缓冲后触发本函数；
 *         通过任务通知唤醒 Modbus_Task，若唤醒了更高优先级任务则触发 PendSV 调度。
 */
void BSP_RS485_RxEventHook(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskHandle_t handle = s_modbus_task_handle;

    /* 防止调度器启动前或任务尚未初始化句柄时提前触发通知 */
    if (handle != NULL)
    {
        vTaskNotifyGiveFromISR(handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

#if MODBUS_ENABLE_SELFTEST
/**
 * @brief  上电协议解析自测（验证正常帧、CRC 错误拦截与从站异常帧处理）
 */
static void Modbus_RunSelfTest(void)
{
    uint16_t regs[2] = {0};
    uint8_t ex_code = 0U;
    Modbus_Status_t st;

    printf("\r\n=== Modbus Protocol SelfTest Start ===\r\n");

    /* 用例 1：合法 0x03 响应帧（reg0=100, reg1=200，精确 CRC16 = 0x7ABA，低字节在前） */
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

    /* 用例 2：篡改末字节（0x7A -> 0x7B），验证 CRC16 校验能正确拦截损坏帧 */
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

    /* 用例 3：从站异常响应帧（功能码 0x83，异常码 0x02 非法地址，CRC16 = 0xF1C0） */
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

/**
 * @brief  Modbus RTU 主站周期轮询任务
 */
static void Modbus_Task(void *argument)
{
    (void)argument;

    uint8_t tx_buf[MODBUS_READ_REQ_SIZE];
    uint8_t rx_buf[64];
    uint16_t regs[MODBUS_REG_QTY];
    uint8_t ex_code = 0U;
    uint32_t ok_cnt = 0U;
    uint32_t err_cnt = 0U;

    /* 调度器启动并进入本任务后，再登记句柄以开放中断唤醒 */
    s_modbus_task_handle = xTaskGetCurrentTaskHandle();

    /* 在循环外记录初始时间基准，确保 vTaskDelayUntil 实现固定周期而不累积漂移 */
    TickType_t last_wake_time = xTaskGetTickCount();

    /* 正常 0x03 响应帧预期总长度：地址(1) + 功能码(1) + 字节数(1) + 数据(2*qty) + CRC(2) */
    const uint16_t expect_len = (uint16_t)(5U + 2U * MODBUS_REG_QTY);

#if MODBUS_ENABLE_SELFTEST
    Modbus_RunSelfTest();
#endif

    for (;;)
    {
        /* 1. 检查 USART2 是否发生过硬件错误（如 ORE 溢出），必要时自动重启 DMA 接收 */
        BSP_RS485_Service();

        /* 2. 构造 8 字节 0x03 读保持寄存器请求帧 */
        size_t tx_len = Modbus_BuildReadHolding(
            MODBUS_SLAVE_ID, MODBUS_START_ADDR, MODBUS_REG_QTY,
            tx_buf, sizeof(tx_buf));

        if (tx_len > 0U)
        {
            /*
             * 3. 发送新请求前，成对清空“软件环形缓冲区”与“旧任务通知计数”：
             *    - FlushRx() 丢弃上次超时迟到或总线干扰产生的脏字节。
             *    - ulTaskNotifyTake(pdTRUE, 0U) 非阻塞清零残留通知，防止后续等待被假唤醒。
             */
            BSP_RS485_FlushRx();
            (void)ulTaskNotifyTake(pdTRUE, 0U);

            /* 4. 半双工发送请求（BSP 内部自动完成：拉高 DE -> 阻塞发完 TC -> 拉低 DE 切收） */
            if (BSP_RS485_Send(tx_buf, (uint16_t)tx_len) == HAL_OK)
            {
                /* 5. 在 200ms 超时窗口内休眠收集应答字节，支持分包（拆包）自动拼接 */
                uint16_t rx_len = 0U;
                TickType_t start_tick = xTaskGetTickCount();

                while (rx_len < expect_len)
                {
                    TickType_t elapsed = xTaskGetTickCount() - start_tick;
                    if (elapsed >= pdMS_TO_TICKS(MODBUS_RESP_TIMEOUT_MS))
                    {
                        break; /* 等待超时，跳出循环进入超时/异常帧判定 */
                    }

                    /* 挂起等待中断通知（释放 CPU 给其他任务），最多等到剩余超时时间 */
                    TickType_t remain = pdMS_TO_TICKS(MODBUS_RESP_TIMEOUT_MS) - elapsed;
                    (void)ulTaskNotifyTake(pdTRUE, remain);

                    /* 从软件环形缓冲区取出新到达的字节，追加拼接到 rx_buf 尾部 */
                    rx_len += BSP_RS485_Read(&rx_buf[rx_len], (uint16_t)(sizeof(rx_buf) - rx_len));
                }

                /* 6. 根据接收结果执行协议解析与日志输出 */
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

        /* 7. 精确 1000ms 周期休眠，等待下一轮轮询 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MODBUS_POLL_PERIOD_MS));
    }
}