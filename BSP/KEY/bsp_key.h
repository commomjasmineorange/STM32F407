
#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdint.h>

/* 按键事件定义：使用位掩码 */
typedef enum
{
    BSP_KEY_NONE = 0,

    BSP_KEY0 = (1U << 0),
    BSP_KEY1 = (1U << 1),
    BSP_KEY2 = (1U << 2),
    BSP_KEY_WKUP = (1U << 3)

} BSP_KeyEvent_t;

/**
 * @brief 初始化按键软件状态
 * @note  必须在 MX_GPIO_Init() 后调用
 */
void BSP_KEY_Init(void);

/**
 * @brief  按键扫描及软件消抖
 * @note   每隔约 10ms 调用一次
 * @retval 按键按下事件位掩码
 *         无新事件时返回 BSP_KEY_NONE
 */
uint8_t BSP_KEY_Scan10ms(void);

#endif /* BSP_KEY_H */
