/**
 * @file    bsp_rs485.c
 * @brief   板载 RS485（USART2 + SP3485 方向脚 PG8）异步驱动实现
 * @note    核心机制：
 *          1. 接收：USART2 + DMA1_Stream5 Circular 模式 + IDLE/HT/TC 中断，
 *             在中断中将新增字节搬运至带 __DMB() 内存屏障的 SPSC 环形缓冲区，
 *             并通过弱函数 BSP_RS485_RxEventHook() 通知上层（与 RTOS 解耦）。
 *          2. 发送：拉高 RS485_RE -> HAL_UART_Transmit() 等待硬件 TC 发送完成 ->
 *             立刻拉低 RS485_RE 切回接收态，防止截断帧尾或漏收从站应答。
 *          3. 健壮性：HAL 错误回调仅置位恢复标志，由任务上下文 BSP_RS485_Service() 限频重启 DMA。
 */

#include "bsp_rs485.h"
#include "main.h"  /* 提供 RS485_RE_GPIO_Port、RS485_RE_Pin */
#include "usart.h" /* CubeMX 生成：提供 huart2 */
#include <stddef.h>

/* 编译期检查：环形缓冲区长度必须是 2 的幂，才能用 (idx + 1) & (SIZE - 1) 回绕 */
#if ((BSP_RS485_RX_RING_SIZE & (BSP_RS485_RX_RING_SIZE - 1U)) != 0U)
#error "BSP_RS485_RX_RING_SIZE must be a power of two"
#endif

/* ======================== 1. 硬件方向控制 ======================== */

/*
 * RS485 是半双工总线：同一时刻只能“发”或“收”。
 * 探索者板载 SP3485 芯片：RE/DE 引脚拉高=发送，拉低=接收。
 */
static inline void RS485_SetTxMode(void)
{
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
}

static inline void RS485_SetRxMode(void)
{
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
}

/* ======================== 2. 私有变量 ======================== */

/* DMA 硬件直接写入的循环缓冲区（必须留在普通 SRAM，不能放 CCM） */
static uint8_t s_dma_rx[BSP_RS485_DMA_RX_SIZE];

/* 软件环形缓冲区：中断往里写（更新 head），Modbus 任务往外读（更新 tail） */
static uint8_t s_rx_ring[BSP_RS485_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* 记录上一次 DMA 数据搬运到的位置，用于计算本次新增的区间 [old, pos) */
static uint16_t s_dma_old_pos = 0U;

/* 错误恢复标志位 */
static volatile uint8_t s_restart_requested = 0U;
static volatile uint8_t s_recovering = 0U;

/* 诊断计数 */
static volatile uint32_t s_rx_dropped = 0U;
static volatile uint32_t s_uart_errors = 0U;
static volatile uint32_t s_last_uart_error = 0U;

/* ======================== 3. DMA 数据搬运 ======================== */

/* 将 s_dma_rx[begin .. end-1] 的字节逐个压入软件环形缓冲区 */
static void RS485_PushDmaRange(uint16_t begin, uint16_t end)
{
    for (uint16_t i = begin; i < end; i++)
    {
        uint16_t head = s_rx_head;
        uint16_t next = (uint16_t)((head + 1U) & (BSP_RS485_RX_RING_SIZE - 1U));

        /* 队列满了：丢弃新字节并计数，不覆盖未读的旧数据 */
        if (next == s_rx_tail)
        {
            s_rx_dropped++;
            continue;
        }

        s_rx_ring[head] = s_dma_rx[i];
        __DMB(); /* 内存屏障：保证数据先写进数组，再更新 head 让任务看到 */
        s_rx_head = next;
    }
}

/* ======================== 4. 初始化与发送 ======================== */

HAL_StatusTypeDef BSP_RS485_Init(void)
{
    /* 检查 CubeMX 是否真的给 USART2 配好了 Circular RX DMA */
    if ((huart2.hdmarx == NULL) ||
        (huart2.hdmarx->Init.Mode != DMA_CIRCULAR))
    {
        return HAL_ERROR;
    }

    /* 默认处于接收状态，避免平时占着 485 总线不放 */
    RS485_SetRxMode();

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_dma_old_pos = 0U;
    s_restart_requested = 0U;
    s_recovering = 0U;
    s_rx_dropped = 0U;
    s_uart_errors = 0U;
    s_last_uart_error = 0U;

    /* 启动 DMA + IDLE 接收（发生半满 HT、全满 TC、总线空闲 IDLE 都会进回调） */
    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart2, s_dma_rx, BSP_RS485_DMA_RX_SIZE);

    if (ret != HAL_OK)
    {
        s_restart_requested = 1U;
    }

    return ret;
}

HAL_StatusTypeDef BSP_RS485_Send(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;

    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    /* 1. 拉高 DE/RE，切换为发送模式 */
    RS485_SetTxMode();

    /*
     * 2. 阻塞发送：
     * HAL_UART_Transmit 内部会等待最后一个字节的 TC（发送完成）标志置位，
     * 确保最后一个 bit 已经彻底从移位寄存器发到总线上，才会返回。
     * 所以返回后立刻切回接收是安全的，不会截断尾字节！
     */
    ret = HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100U);

    /* 3. 立刻拉低 DE/RE，切回接收模式，准备接从站的回复 */
    RS485_SetRxMode();

    return ret;
}

/* ======================== 5. 读取与清空 ======================== */

uint16_t BSP_RS485_Read(uint8_t *dst, uint16_t capacity)
{
    uint16_t n = 0U;

    if (dst == NULL)
    {
        return 0U;
    }

    while (n < capacity)
    {
        uint16_t tail = s_rx_tail;
        if (tail == s_rx_head)
        {
            break; /* 缓冲区已空 */
        }

        __DMB();
        dst[n++] = s_rx_ring[tail];
        __DMB();
        s_rx_tail = (uint16_t)((tail + 1U) & (BSP_RS485_RX_RING_SIZE - 1U));
    }

    return n;
}

void BSP_RS485_FlushRx(void)
{
    /*
     * 简单安全地丢弃所有未读字节：
     * 把消费者指针 tail 直接追平到当前的 head。
     * （单消费者场景下只改 tail，不会和中断改 head 冲突）
     */
    s_rx_tail = s_rx_head;
}

/* ======================== 6. 中断事件与错误恢复 ======================== */

/* 由全局 HAL_UARTEx_RxEventCallback 分流调用（中断上下文） */
void BSP_RS485_RxEventHandler(uint16_t size)
{
    if ((s_recovering != 0U) || (size == 0U) || (size > BSP_RS485_DMA_RX_SIZE))
    {
        return;
    }

    uint16_t pos = (size == BSP_RS485_DMA_RX_SIZE) ? 0U : size;
    uint16_t old = s_dma_old_pos;

    if (pos > old)
    {
        /* 情况 1：DMA 没绕回，新数据是连续一段 [old, pos) */
        RS485_PushDmaRange(old, pos);
    }
    else if (pos < old)
    {
        /* 情况 2：DMA 写到末尾绕回了头部，拆成尾部和头部两段搬运 */
        RS485_PushDmaRange(old, BSP_RS485_DMA_RX_SIZE);
        RS485_PushDmaRange(0U, pos);
    }

    s_dma_old_pos = pos;

    /* 通知上层（Modbus 任务）有新数据到了 */
    BSP_RS485_RxEventHook();
}

/* 由全局 HAL_UART_ErrorCallback 分流调用（中断上下文） */
void BSP_RS485_ErrorHandler(UART_HandleTypeDef *huart)
{
    s_uart_errors++;
    s_last_uart_error = huart->ErrorCode;
    s_restart_requested = 1U;
}

/* 在任务中定期调用：发现硬件错误时重启 DMA 接收 */
void BSP_RS485_Service(void)
{
    static uint32_t s_last_attempt = 0U;

    if (s_restart_requested == 0U)
    {
        return;
    }

    /* 限频：最多每 10ms 尝试恢复一次，防止总线故障时死循环重启 */
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_attempt) < 10U)
    {
        return;
    }
    s_last_attempt = now;

    s_recovering = 1U;
    if (HAL_UART_AbortReceive(&huart2) != HAL_OK)
    {
        s_recovering = 0U;
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    s_dma_old_pos = 0U;
    s_restart_requested = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_dma_rx, BSP_RS485_DMA_RX_SIZE) != HAL_OK)
    {
        s_restart_requested = 1U;
    }
    s_recovering = 0U;
}

void BSP_RS485_GetStats(BSP_RS485_Stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->rx_dropped = s_rx_dropped;
    out->uart_errors = s_uart_errors;
    out->last_uart_error = s_last_uart_error;
    __set_PRIMASK(primask);
}

/* 默认弱函数，App 层重写它来调用 vTaskNotifyGiveFromISR */
__attribute__((weak)) void BSP_RS485_RxEventHook(void)
{
}