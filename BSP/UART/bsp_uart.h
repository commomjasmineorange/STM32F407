#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/*
 * USART1 接收缓冲区配置：
 * 1. DMA 缓冲区：硬件自动写入，Circular 模式反复使用。
 * 2. 软件环形缓冲区：保存尚未被应用层读取的数据。
 * 3. 软件环形缓冲区使用“预留一个空槽”区分空和满，实际容量为 1023 字节。
 * 4. 软件环形缓冲区长度必须是 2 的整数次幂。
 */
#define BSP_UART_DMA_RX_SIZE 256U
#define BSP_UART_RX_RING_SIZE 1024U

/*
 * 精简的运行诊断信息。
 * 注意：rx_dropped 只统计软件环形缓冲区满而丢失的数据，
 * 不包含因硬件 UART 错误、DMA 被覆盖或恢复接收期间丢失的数据。
 */
typedef struct
{
    uint32_t rx_dropped;      /* 软件环形缓冲区满：丢弃的字节总数 */
    uint32_t uart_errors;     /* HAL 串口错误回调发生的次数 */
    uint32_t last_uart_error; /* 最近一次错误码（HAL_UART_ERROR_xxx 位掩码） */
} BSP_UART_Stats_t;

/* 在 MX_DMA_Init()、MX_USART1_UART_Init() 完成后调用一次。 */
HAL_StatusTypeDef BSP_UART_Init(void);

/* 阻塞发送，供主循环使用；不要在接收中断回调中调用。 */
HAL_StatusTypeDef BSP_UART_Send(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef BSP_UART_Print(const char *str);

/* 非阻塞读取：只有一个消费者可以修改读取索引 tail。 */
bool BSP_UART_ReadByte(uint8_t *data);
uint16_t BSP_UART_Read(uint8_t *dst, uint16_t capacity);
uint16_t BSP_UART_Available(void);

/* 主循环定期调用：接收出错后尝试恢复 DMA + IDLE 接收。 */
void BSP_UART_Service(void);

/* 获取统计信息的快照；仅在主循环/任务上下文调用。 */
void BSP_UART_GetStats(BSP_UART_Stats_t *out);

/* 串口接收数据事件回调,在中断上下文中执行*/
void BSP_UART_RxEventHook(void);

#endif
/* BSP_UART_H */
