
#include "bsp_key.h"
#include "gpio.h"

/* 按键数量 */
#define BSP_KEY_COUNT 4U

/* 连续两次采样相同才确认状态 */
#define KEY_DEBOUNCE_COUNT 2U

/* 按键状态结构体 */
typedef struct
{
    uint8_t last_sample;     /* 上一次采样状态 */
    uint8_t current_sample;  /* 当前采样状态 */
    uint16_t debounce_count; /* 连续相同采样次数 */
    uint8_t stable_state;    /* 消抖确认后的稳定状态 */

} keyState;

/* 每个按键独立保存一份状态 */
static keyState s_key_states[BSP_KEY_COUNT];

/**
 * @brief  读取指定按键的原始逻辑状态
 * @param  index 按键索引 0~3
 * @retval 1：按下
 *         0：松开
 *
 * @note   KEY0/1/2 低电平有效
 *         WK_UP 高电平有效
 */
static uint8_t BSP_KEY_ReadRaw(uint8_t index)
{
    switch (index)
    {
    case 0: /* KEY0 -> PE4 */
        return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET);

    case 1: /* KEY1 -> PE3 */
        return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET);

    case 2: /* KEY2 -> PE2 */
        return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET);

    case 3: /* WK_UP -> PA0 */
        return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);

    default:
        return 0U;
    }
}

/**
 * @brief  初始化按键状态
 * @note   GPIO 已经由 CubeMX 完成初始化
 */
void BSP_KEY_Init(void)
{
    for (uint8_t i = 0; i < BSP_KEY_COUNT; i++)
    {
        /* 获取初始化时的按键状态 */
        uint8_t state = BSP_KEY_ReadRaw(i);

        /* 初始化采样状态 */
        s_key_states[i].last_sample = state;
        s_key_states[i].current_sample = state;

        /* 初始化消抖计数 */
        s_key_states[i].debounce_count = 0U;

        /* 初始化已确认的状态 */
        s_key_states[i].stable_state = state;
    }
}

/**
 * @brief  按键扫描及消抖
 * @note   上层每隔约 10ms 调用一次
 *         函数内部不使用阻塞延时
 *
 * @retval 按键按下事件位掩码
 */
uint8_t BSP_KEY_Scan10ms(void)
{
    /* 本次扫描产生的按键事件 */
    uint8_t events = BSP_KEY_NONE;

    /* 遍历四个按键 */
    for (uint8_t i = 0; i < BSP_KEY_COUNT; i++)
    {
        /* 第一步：读取当前按键状态 */
        s_key_states[i].current_sample =
            BSP_KEY_ReadRaw(i);

        /* 第二步：判断采样是否连续相同 */
        if (s_key_states[i].current_sample !=
            s_key_states[i].last_sample)
        {
            /* 电平发生变化，重新开始计数 */
            s_key_states[i].debounce_count = 1U;
        }
        else
        {
            /* 采样结果连续相同，计数加一 */
            if (s_key_states[i].debounce_count <
                KEY_DEBOUNCE_COUNT)
            {
                s_key_states[i].debounce_count++;
            }
        }

        /* 第三步：更新上一次采样状态 */
        s_key_states[i].last_sample =
            s_key_states[i].current_sample;

        /* 第四步：判断是否通过消抖 */
        if (s_key_states[i].debounce_count >=
            KEY_DEBOUNCE_COUNT)
        {
            /* 检查稳定状态是否发生改变 */
            if (s_key_states[i].current_sample !=
                s_key_states[i].stable_state)
            {
                /* 更新稳定状态 */
                s_key_states[i].stable_state =
                    s_key_states[i].current_sample;

                /* 只在确认按下时产生事件 */
                if (s_key_states[i].stable_state == 1U)
                {
                    /* 根据按键编号设置事件标志 */
                    events |= (uint8_t)(1U << i);
                }
            }
        }
    }

    return events;
}
