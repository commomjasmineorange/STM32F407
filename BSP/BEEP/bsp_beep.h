
#ifndef BSP_BEEP_H
#define BSP_BEEP_H

#include <stdbool.h>

/**
 * @brief  蜂鸣器初始化
 * @note   GPIO 由 CubeMX 配置，本函数仅关闭蜂鸣器
 */
void BSP_BEEP_Init(void);

/**
 * @brief  蜂鸣器开关控制
 * @param  on true：开启
 *            false：关闭
 */
void BSP_BEEP_Set(bool on);

#endif /* BSP_BEEP_H */
