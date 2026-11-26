/**
 * @file    bsp_led.h
 * @brief   板载 LED 指示灯驱动接口（PF9 红灯 / PF10 绿灯，低电平点亮）
 */

#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

/* 板载 LED 逻辑编号 */
typedef enum
{
    BSP_LED_RED = 0,
    BSP_LED_GREEN,
    BSP_LED_MAX,
} BSP_LED_ID_t;

/** @brief 初始化 LED（默认全部熄灭，GPIO 模式由 CubeMX 配置） */
void BSP_LED_Init(void);

/** @brief 设置指定 LED 开关状态（true=点亮，false=熄灭） */
void BSP_LED_Set(BSP_LED_ID_t led, bool on);

/** @brief 翻转指定 LED 当前状态 */
void BSP_LED_Toggle(BSP_LED_ID_t led);

/** @brief 查询指定 LED 当前是否点亮（true=亮，false=灭） */
bool BSP_LED_GetState(BSP_LED_ID_t led);

#endif /* BSP_LED_H */
