#include "bsp_uart.h"
#include "usart.h" /* CubeMX 生成：提供 huart1 */

#include <stddef.h> /* NULL */
#include <string.h> /* strlen() */

/* 使用按位与运算实现环形回绕，要求缓冲区长度为 2 的整数次幂。 */
#if ((BSP_UART_RX_RING_SIZE & (BSP_UART_RX_RING_SIZE - 1U)) != 0U)
#error "BSP_UART_RX_RING_SIZE must be a power of two"
#endif

#if (BSP_UART_DMA_RX_SIZE < 2U) || (BSP_UART_RX_RING_SIZE < 2U)
#error "UART buffer size must be at least 2"
#endif

/* ======================== 1. 私有变量 ======================== */

/*
 * DMA 接收缓冲区（硬件写入）：
 * USART1 接收到的数据由 DMA 放入这里，写满后从头继续写。
 * STM32F407 的普通 DMA 不能访问 CCM SRAM，因此本数组应保留在
 * 默认的、DMA 可访问的 SRAM1/SRAM2 中，不要人为放进 CCM。
 */
static uint8_t s_dma_rx[BSP_UART_DMA_RX_SIZE];

/*
 * 软件环形缓冲区（CPU 写入、应用层读取）：
 * 中断回调将 DMA 中尚未处理的新数据复制到这里。
 * 只有一个接收生产者（UART/DMA 中断回调）和一个应用层消费者。
 */
static uint8_t s_rx_ring[BSP_UART_RX_RING_SIZE];

/* head：下次写入位置，中断更新；tail：下次读取位置，主循环更新。 */
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* DMA 缓冲区中“已经处理过的数据”的结束位置（下次从这里开始复制）。 */
static uint16_t s_dma_old_pos = 0U;

/*
 * 异常恢复标志：
 * restart_requested = 1：接收出现问题，主循环需要尝试恢复。
 * recovering = 1：主循环正在恢复，接收回调暂时不处理数据。
 */
static volatile uint8_t s_restart_requested = 0U;
static volatile uint8_t s_recovering = 0U;

/* 精简统计：只保留对定位接收故障有帮助的三项。 */
static volatile uint32_t s_rx_dropped = 0U;
static volatile uint32_t s_uart_errors = 0U;
static volatile uint32_t s_last_uart_error = 0U;

/* ======================== 2. DMA 数据转存 ======================== */

/**
 * @brief  把 DMA 缓冲区指定范围内的新数据放入软件环形缓冲区。
 * @param  begin 本次复制的起始位置（包含）
 * @param  end   本次复制的结束位置（不包含）
 * @note   只在 UART/DMA 接收事件回调中调用。
 *         例如 PushDmaRange(100, 150) 复制下标 100~149，共 50 字节。
 */
static void BSP_UART_PushDmaRange(uint16_t begin, uint16_t end)
{
    uint16_t i;

    for (i = begin; i < end; i++)
    {
        uint16_t head = s_rx_head;

        /*
         * 计算写入后的下一个 head：
         * 1024 为 2 的整数次幂，因此 (head + 1) & 1023
         * 等价于 (head + 1) % 1024。
         */
        uint16_t next = (uint16_t)((head + 1U) &
                                   (BSP_UART_RX_RING_SIZE - 1U));

        /*
         * next == tail：再写入就会追上尚未读取的数据。
         * 采取“丢弃新字节、保留旧数据”策略。
         */
        if (next == s_rx_tail)
        {
            s_rx_dropped++;
            continue;
        }

        /* 先把字节放入缓冲区，再移动 head，通知消费者可以读取。 */
        s_rx_ring[head] = s_dma_rx[i];
        __DMB(); /* 约束数据写入与 head 更新的先后顺序 */
        s_rx_head = next;
    }
}

/* ======================== 3. 初始化及发送 ======================== */

/**
 * @brief  初始化 BSP 串口，并启动 USART1 DMA + IDLE 接收。
 * @note   需要先调用 MX_DMA_Init() 和 MX_USART1_UART_Init()。
 *         DMA 必须配置为 Circular，且必须开启 USART1、DMA RX 的 NVIC。
 *         此函数只在系统启动时调用一次，正常接收过程中不反复启动 DMA。
 * @retval HAL_OK / HAL_ERROR / HAL_BUSY 等 HAL 状态
 */
HAL_StatusTypeDef BSP_UART_Init(void)
{
    HAL_StatusTypeDef result;

    /* 防止 CubeMX 忘记配置 DMA 或没有把 DMA 句柄链接到 UART。 */
    if ((huart1.hdmarx == NULL) ||
        (huart1.hdmarx->Init.Mode != DMA_CIRCULAR))
    {
        return HAL_ERROR;
    }

    /* 软件接收状态初始化（不会直接重新配置 USART1 的 GPIO）。 */
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_dma_old_pos = 0U;
    s_restart_requested = 0U;
    s_recovering = 0U;
    s_rx_dropped = 0U;
    s_uart_errors = 0U;
    s_last_uart_error = 0U;

    /*
     * 启动 DMA + IDLE 接收：
     * DMA 持续往 s_dma_rx 写数据。
     * 有 HT（半满）、TC（全满）、IDLE（串口空闲）事件时，
     * HAL 调用 HAL_UARTEx_RxEventCallback()。
     */
    result = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                          s_dma_rx,
                                          BSP_UART_DMA_RX_SIZE);

    if (result != HAL_OK)
    {
        /* 标记异常；之后可在主循环中通过 Service() 尝试恢复。 */
        s_restart_requested = 1U;
    }

    return result;
}

/**
 * @brief  阻塞发送指定长度的数据（支持二进制数据，不依赖 '\0'）。
 * @param  data 数据首地址
 * @param  len  发送的字节数
 * @retval HAL 发送结果
 * @note   仅供主循环使用，暂不处理多任务并发发送。
 */
HAL_StatusTypeDef BSP_UART_Send(const uint8_t *data, uint16_t len)
{
    if (len == 0U)
    {
        return HAL_OK;
    }

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000U);
}

/**
 * @brief  发送以 '\0' 结尾的 C 字符串。
 * @note   这里只负责原样发送字符串，不负责 printf 的格式化处理。
 */
HAL_StatusTypeDef BSP_UART_Print(const char *str)
{
    size_t len;

    if (str == NULL)
    {
        return HAL_ERROR;
    }

    len = strlen(str);
    if (len > UINT16_MAX)
    {
        return HAL_ERROR;
    }

    return BSP_UART_Send((const uint8_t *)str, (uint16_t)len);
}

/* ======================== 4. 软件环形缓冲区读取 ======================== */

/**
 * @brief  从软件环形缓冲区非阻塞读取 1 字节。
 * @param  data 读取结果的输出地址
 * @retval true：成功读出 1 字节；false：缓冲区为空或参数无效
 * @note   只有一个消费者可以调用读取接口。
 */
bool BSP_UART_ReadByte(uint8_t *data)
{
    uint16_t tail;

    if (data == NULL)
    {
        return false;
    }

    tail = s_rx_tail;

    /* head == tail，说明所有已接收的数据都已经被读走。 */
    if (tail == s_rx_head)
    {
        return false;
    }

    /* 先读取数据，再向前推进 tail，释放这个缓冲区位置。 */
    __DMB();
    *data = s_rx_ring[tail];
    __DMB();
    s_rx_tail = (uint16_t)((tail + 1U) &
                           (BSP_UART_RX_RING_SIZE - 1U));

    return true;
}

/**
 * @brief  最多读取 capacity 个字节，读完现有数据立即返回。
 * @param  dst      目标数组
 * @param  capacity 目标数组容量（字节）
 * @retval 实际读取的字节数（可以小于 capacity）
 */
uint16_t BSP_UART_Read(uint8_t *dst, uint16_t capacity)
{
    uint16_t n = 0U;

    if (dst == NULL)
    {
        return 0U;
    }

    while ((n < capacity) && BSP_UART_ReadByte(&dst[n]))
    {
        n++;
    }

    return n;
}

/**
 * @brief  查询软件环形缓冲区中当前尚未读取的字节数。
 * @retval 当前可读字节数（快照值；中断接收仍可能继续写入）
 */
uint16_t BSP_UART_Available(void)
{
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    return (uint16_t)((head - tail) & (BSP_UART_RX_RING_SIZE - 1U));
}

/* ======================== 5. DMA + IDLE 接收回调 ======================== */

/**
 * @brief  UART 接收事件回调：由 HAL 在 IDLE / DMA HT / DMA TC 时调用。
 * @param  huart UART 句柄
 * @param  Size  DMA 当前处理边界，不是“本次新增的字节数”！
 *
 * @note   此处只负责把 DMA 新增的数据复制到软件环形缓冲区。
 *         Circular DMA 会自动继续接收，不要在这里重复启动 DMA。
 *         USART1 IRQ 与 RX DMA IRQ 请设为相同的抢占优先级，
 *         防止两个回调相互嵌套，破坏 s_dma_old_pos 的更新顺序。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t pos;
    uint16_t old;

    if ((huart->Instance != USART1) || (s_recovering != 0U))
    {
        return;
    }

    if ((Size == 0U) || (Size > BSP_UART_DMA_RX_SIZE))
    {
        return;
    }

    /*
     * DMA TC 时 Size 可以等于 256。
     * Circular 模式写入到末尾后，会从索引 0 重新开始，故统一转为 0。
     */
    pos = (Size == BSP_UART_DMA_RX_SIZE) ? 0U : Size;
    old = s_dma_old_pos;

    if (pos > old)
    {
        /* 情况一：DMA 尚未绕回，新增数据是连续的一段 [old, pos)。 */
        BSP_UART_PushDmaRange(old, pos);
    }
    else if (pos < old)
    {
        /*
         * 情况二：DMA 已经绕回，需要分两段复制。
         * 例如 old=240, pos=15：先复制 240~255，再复制 0~14。
         */
        BSP_UART_PushDmaRange(old, BSP_UART_DMA_RX_SIZE);
        BSP_UART_PushDmaRange(0U, pos);
    }
    /*
     * pos == old：通常表示没有新的区间需要处理，例如重复的事件通知。
     * 如果 DMA 已经完整绕圈而 CPU 仍未来得及处理，则可能丢数据；
     * 本算法依赖 HT/TC/IDLE 及时响应，不能检测所有硬件层面的覆盖。
     */

    s_dma_old_pos = pos;

    /* 新数据已经写入环形缓冲区后，通知应用层 */
    BSP_UART_RxEventHook();
}

/* ======================== 6. UART 错误与恢复 ======================== */

/**
 * @brief  UART 错误回调。
 * @note   中断中只记录错误，提交恢复请求，不在这里执行复杂的停止和重启。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    s_uart_errors++;
    s_last_uart_error = huart->ErrorCode;
    s_restart_requested = 1U;
}

/**
 * @brief  主循环定期调用，发现错误请求时尝试恢复 UART 接收 DMA。
 * @note   恢复只针对 RX；恢复期间新到的数据可能丢失。
 *         若故障持续存在，最多每隔 10ms 尝试一次，避免死循环重启。
 */
void BSP_UART_Service(void)
{
    HAL_StatusTypeDef result;
    static uint32_t s_last_attempt = 0U;
    uint32_t now;

    if (s_restart_requested == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_attempt) < 10U)
    {
        return;
    }
    s_last_attempt = now;

    /* 恢复期间丢弃回调处理，避免访问正在被停止/重启的 DMA 缓冲区。 */
    s_recovering = 1U;

    /* 停止接收 DMA，不主动影响 TX；失败则留待下次 Service() 再试。 */
    result = HAL_UART_AbortReceive(&huart1);
    if (result != HAL_OK)
    {
        s_recovering = 0U;
        return;
    }

    /* F407 清除 ORE 的操作是读取 SR/DR；HAL 宏完成该动作。 */
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    /* DMA 重新从索引 0 开始写，所以“上次处理位置”也要复位。 */
    s_dma_old_pos = 0U;
    s_restart_requested = 0U;

    /* 重新启动 DMA + IDLE 接收，Normal 模式的逐帧重启不在这里使用。 */
    result = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                          s_dma_rx,
                                          BSP_UART_DMA_RX_SIZE);
    if (result != HAL_OK)
    {
        s_restart_requested = 1U;
    }

    s_recovering = 0U;
}

/* ======================== 7. 精简诊断统计 ======================== */

/**
 * @brief  获取错误及溢出计数的快照。
 * @param  out 用户提供的结构体地址
 * @note   主循环调用；临时屏蔽中断，仅为一次性读取到相互一致的统计值。
 */
void BSP_UART_GetStats(BSP_UART_Stats_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    /* 保存之前的中断屏蔽状态，完成复制后恢复原来的状态。 */
    primask = __get_PRIMASK();
    __disable_irq();

    out->rx_dropped = s_rx_dropped;
    out->uart_errors = s_uart_errors;
    out->last_uart_error = s_last_uart_error;

    __set_PRIMASK(primask);
}

/**
 * @brief 串口接收事件默认处理函数
 *
 * 应用层可以重新实现该函数，
 * BSP 层不关心具体的事件处理方式。
 */
__attribute__((weak)) void BSP_UART_RxEventHook(void)
{
}
