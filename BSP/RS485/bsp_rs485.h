#ifndef BSP_RS485_H
#define BSP_RS485_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
/*
 * RS485 (USART2) 缓冲区大小配置：
 * - Modbus RTU 标准单帧最大 256 字节，所以 DMA 缓冲设 256 足够。
 * - 软件环形缓冲设 512（必须是 2 的整数次幂，方便用位运算取模）。
 */
#define BSP_RS485_DMA_RX_SIZE 256U
#define BSP_RS485_RX_RING_SIZE 512U

/* RS485 诊断统计结构体，用于排查丢包和总线干扰 */
typedef struct
{
    uint32_t rx_dropped;      /* 软件环形缓冲满导致丢弃的字节数 */
    uint32_t uart_errors;     /* 串口硬件错误次数（如 ORE 溢出、噪声、帧错误） */
    uint32_t last_uart_error; /* 最近一次错误码 */
} BSP_RS485_Stats_t;

/* 初始化 RS485：切到接收态并开启 USART2 DMA Circular + IDLE 接收 */
HAL_StatusTypeDef BSP_RS485_Init(void);

/* 阻塞发送（内部自动完成：切发送 -> 发送并等发完 -> 切回接收） */
HAL_StatusTypeDef BSP_RS485_Send(const uint8_t *data, uint16_t len);

/* 从软件环形缓冲区读取数据（非阻塞，有几个读几个） */
uint16_t BSP_RS485_Read(uint8_t *dst, uint16_t capacity);

/* 清空软件环形缓冲区（每次发新的 Modbus 请求前调用，防止残留脏数据干扰） */
void BSP_RS485_FlushRx(void);

/* 错误恢复服务函数：在任务循环里定期调用 */
void BSP_RS485_Service(void);
/* 获取统计快照 */
void BSP_RS485_GetStats(BSP_RS485_Stats_t *out);
/*
 * 提供给全局 HAL 回调分流用的内部处理入口
 * （因为 HAL_UARTEx_RxEventCallback 全局只能定义一次，要在 bsp_uart.c 里按串口号分发）
 */
void BSP_RS485_RxEventHandler(uint16_t size);
void BSP_RS485_ErrorHandler(UART_HandleTypeDef *huart);
/* 弱函数钩子：收到数据时在中断中触发，供 App 层重写来唤醒 FreeRTOS 任务 */
void BSP_RS485_RxEventHook(void);

#endif