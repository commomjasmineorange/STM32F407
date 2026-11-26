#include "bsp_led.h"
#include "stm32f4xx_hal.h"

/* LED 逻辑编号到硬件 GPIO 端口/引脚的映射表 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} BSP_LED_Hardware_t;

static const BSP_LED_Hardware_t s_led_map[BSP_LED_MAX] =
    {
        [BSP_LED_RED] = {GPIOF, GPIO_PIN_9},
        [BSP_LED_GREEN] = {GPIOF, GPIO_PIN_10}};

/**
 * @brief 初始化 LED
 * @note  GPIO 模式已经由 CubeMX 配置
 */
void BSP_LED_Init(void)
{
    for (uint8_t i = 0; i < BSP_LED_MAX; i++)
    {
        HAL_GPIO_WritePin(s_led_map[i].port, s_led_map[i].pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 控制 LED 开关
 * @param led LED 设备编号
 * @param on  true: 点亮  false: 熄灭
 */
void BSP_LED_Set(BSP_LED_ID_t led, bool on)
{
    if ((uint32_t)led >= BSP_LED_MAX)
    {
        return;
    }

    /* LED 为低电平点亮 */
    HAL_GPIO_WritePin(
        s_led_map[led].port,
        s_led_map[led].pin,
        on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 翻转 LED 状态
 * @param led LED 设备编号
 */
void BSP_LED_Toggle(BSP_LED_ID_t led)
{
    if ((uint32_t)led >= BSP_LED_MAX)
    {
        return;
    }

    HAL_GPIO_TogglePin(
        s_led_map[led].port,
        s_led_map[led].pin);
}

/**
 * @brief 读取LED状态
 * @param led LED设备编号
 * @return 1:ON 0:OFF
 */
bool BSP_LED_GetState(BSP_LED_ID_t led)
{
    if ((uint32_t)led >= BSP_LED_MAX)
    {
        return false;
    }

    GPIO_PinState state = HAL_GPIO_ReadPin(
        s_led_map[led].port,
        s_led_map[led].pin);

    return (state == GPIO_PIN_RESET);
}