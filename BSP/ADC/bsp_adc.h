#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/*
 * 探索者 F407 板载光敏传感器：PF7 / ADC3_IN5。
 * ADC3 由 TIM2 TRGO 以 100 Hz 触发；DMA 循环接收 32 个 16 位采样值。
 * 每收到 16 个采样值，更新一次最近值和分块平均值。
 */
#define BSP_ADC_DMA_SAMPLE_COUNT 32U
#define BSP_ADC_VREF_MV 3300U /* 假定 VDDA=3.3V，仅用于估算电压 */

typedef struct
{
    uint16_t raw;          /* 最新一次 ADC 原始值，范围 0~4095 */
    uint16_t average;      /* 最近 16 个采样值的平均值，范围 0~4095 */
    uint16_t voltage_mv;   /* 平均值换算的电压，单位 mV（假定参考电压为 3.3V） */
    uint32_t sample_count; /* 从启动以来，完成处理的采样数量 */
} BSP_ADC_Result_t;

/*
 * 在 MX_DMA_Init()、MX_ADC3_Init()、MX_TIM2_Init() 后调用。
 * 启动顺序：先开启 ADC+DMA，再启动 TIM2 触发。
 */
HAL_StatusTypeDef BSP_ADC_Init(void);

/* 先停止 TIM2 触发，再停止 ADC+DMA。 */
HAL_StatusTypeDef BSP_ADC_Stop(void);

/*
 * 非阻塞读取最近一次计算结果。
 * 返回 true：已有有效的平均值；false：尚未积累 16 个点或输入指针无效。
 */
bool BSP_ADC_GetResult(BSP_ADC_Result_t *result);

/* 获取最近一次 HAL ADC 错误码；0 表示尚未记录到错误。 */
uint32_t BSP_ADC_GetLastError(void);

/*
 * 由主循环调用的显式恢复接口：停止并重新启动 ADC+DMA+TIM2。
 * 如果一直返回错误，应检查 CubeMX 设置、DMA 中断和 ADC 时钟。
 */
HAL_StatusTypeDef BSP_ADC_Restart(void);

#endif /* BSP_ADC_H */
