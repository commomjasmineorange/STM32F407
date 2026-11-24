
#include "bsp_beep.h"
#include "stm32f4xx_hal.h"

/* 蜂鸣器硬件映射：仅在当前文件内部使用 */
#define BEEP_GPIO_PORT GPIOF
#define BEEP_GPIO_PIN GPIO_PIN_8

/**
 * @brief  初始化蜂鸣器
 * @note   必须在 MX_GPIO_Init() 之后调用
 */
void BSP_BEEP_Init(void)
{
    /* 上电后默认关闭蜂鸣器 */
    BSP_BEEP_Set(false);
}

/**
 * @brief  控制蜂鸣器
 * @param  on true：开启
 *            false：关闭
 */
void BSP_BEEP_Set(bool on)
{
    HAL_GPIO_WritePin(
        BEEP_GPIO_PORT,
        BEEP_GPIO_PIN,
        on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
