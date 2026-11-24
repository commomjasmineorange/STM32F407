#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_LED_RED = 0,
    BSP_LED_GREEN,
    BSP_LED_MAX,
} BSP_LED_ID_t;

/* LED 初始化 */
void BSP_LED_Init(void);

/* LED 开关控制 */
void BSP_LED_Set(BSP_LED_ID_t led, bool on);

/* LED 状态翻转 */
void BSP_LED_Toggle(BSP_LED_ID_t led);

bool BSP_LED_GetState(BSP_LED_ID_t led);

#endif
