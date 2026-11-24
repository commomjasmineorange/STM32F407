#include "bsp_adc.h"
#include "adc.h"    /* CubeMX 生成：提供 hadc3 */
#include "tim.h"    /* CubeMX 生成：提供 htim2 */
#include <stddef.h> /* NULL */

#if (BSP_ADC_DMA_SAMPLE_COUNT < 2U) || ((BSP_ADC_DMA_SAMPLE_COUNT % 2U) != 0U)
#error "BSP_ADC_DMA_SAMPLE_COUNT must be an even number >= 2"
#endif

/* DMA 缓冲区前 16 个点由半传输中断处理，后 16 个点由传输完成中断处理。 */
#define BSP_ADC_HALF_COUNT (BSP_ADC_DMA_SAMPLE_COUNT / 2U)

/*
 * 重要：ADC 结果为 12 位，但 DMA 的存储宽度需要配置为 Half Word（16 位）。
 * 本缓冲区必须位于 DMA 可访问的 SRAM1/SRAM2，不可放在 F407 的 CCM SRAM。
 * 不需要将本数组声明为 volatile；DMA 写入的半缓冲在对应回调中已经完成。
 */
static uint16_t s_adc_dma_buffer[BSP_ADC_DMA_SAMPLE_COUNT];

/* 以下结果由 DMA 中断更新，主循环读取。 */
static volatile uint16_t s_last_raw = 0U;
static volatile uint16_t s_last_average = 0U;
static volatile uint32_t s_sample_count = 0U;
static volatile uint8_t s_data_valid = 0U;
static volatile uint8_t s_running = 0U;
static volatile uint32_t s_last_error = HAL_ADC_ERROR_NONE;

/**
 * @brief 处理 DMA 缓冲区中一个已经接收完成的半区。
 * @param begin 半区起始索引，半传输事件传入 0，完成事件传入 16。
 * @note  本函数在 DMA 中断回调上下文执行，只进行 16 次求和，不打印、不延时。
 *        100 Hz 采样下，每个半区约 160 ms 更新一次。
 */
static void BSP_ADC_ProcessHalf(uint16_t begin)
{
    uint32_t sum = 0U;
    uint16_t i;

    /* 只读取已经完成 DMA 写入的那一半，不碰 DMA 正在写入的另一半。 */
    for (i = 0U; i < BSP_ADC_HALF_COUNT; i++)
    {
        sum += s_adc_dma_buffer[begin + i];
    }

    /* 平均值四舍五入，最新原始值取本半区的最后一个样本。 */
    s_last_average = (uint16_t)((sum + BSP_ADC_HALF_COUNT / 2U) /
                                BSP_ADC_HALF_COUNT);
    s_last_raw = s_adc_dma_buffer[begin + BSP_ADC_HALF_COUNT - 1U];
    s_sample_count += BSP_ADC_HALF_COUNT;

    /* 确保结果先更新，再将其标记为可读取。 */
    __DMB();
    s_data_valid = 1U;
}

/**
 * @brief 启动板载光敏 ADC 采集。
 * @note  GPIO、ADC3、DMA2 和 TIM2 的寄存器配置统一由 CubeMX 生成代码负责。
 *        此处仅检查必要的 DMA 设置、重置软件状态并启动外设。
 */
HAL_StatusTypeDef BSP_ADC_Init(void)
{
    HAL_StatusTypeDef status;

    if (s_running != 0U)
    {
        return HAL_BUSY;
    }

    /* DMA 必须与 ADC3 关联，且必须使用 Circular + Half Word 模式。 */
    if ((hadc3.DMA_Handle == NULL) ||
        (hadc3.DMA_Handle->Init.Mode != DMA_CIRCULAR) ||
        (hadc3.DMA_Handle->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD) ||
        (hadc3.DMA_Handle->Init.MemDataAlignment != DMA_MDATAALIGN_HALFWORD))
    {
        return HAL_ERROR;
    }

    s_last_raw = 0U;
    s_last_average = 0U;
    s_sample_count = 0U;
    s_data_valid = 0U;
    s_last_error = HAL_ADC_ERROR_NONE;

    /*
     * HAL 的参数类型是 uint32_t*，这是 STM32 HAL 的通用接口设计。
     * 实际 DMA 搬运宽度取决于 CubeMX 的 Half Word 配置，每次写入 2 字节。
     * Length 的单位是 DMA 数据项个数，因此这里传 32，而不是 64 字节。
     */
    status = HAL_ADC_Start_DMA(&hadc3,
                               (uint32_t *)s_adc_dma_buffer,
                               BSP_ADC_DMA_SAMPLE_COUNT);
    if (status != HAL_OK)
    {
        return status;
    }

    /* ADC+DMA 已经准备好，最后开启 TIM2，开始每 10 ms 产生触发。 */
    status = HAL_TIM_Base_Start(&htim2);
    if (status != HAL_OK)
    {
        (void)HAL_ADC_Stop_DMA(&hadc3);
        return status;
    }

    s_running = 1U;
    return HAL_OK;
}

/**
 * @brief 停止采样。
 * @note  优先停止 TIM2，避免停止 ADC 的同时仍有新的定时触发进入。
 */
HAL_StatusTypeDef BSP_ADC_Stop(void)
{
    HAL_StatusTypeDef tim_result;
    HAL_StatusTypeDef adc_result;

    if (s_running == 0U)
    {
        return HAL_OK;
    }

    tim_result = HAL_TIM_Base_Stop(&htim2);
    adc_result = HAL_ADC_Stop_DMA(&hadc3);

    s_running = 0U;
    s_data_valid = 0U;

    return ((tim_result == HAL_OK) && (adc_result == HAL_OK)) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 从中断更新的状态中获取一份一致的数据快照。
 * @param result 输出数据的结构体指针。
 * @note  仅在主循环或任务上下文调用。临时屏蔽中断是为了避免读取
 *        raw、average、sample_count 时恰好被 DMA 回调打断。
 */
bool BSP_ADC_GetResult(BSP_ADC_Result_t *result)
{
    uint32_t primask;

    if (result == NULL)
    {
        return false;
    }

    /* 记录此前是否已经处于中断屏蔽状态，结束后恢复原状态。 */
    primask = __get_PRIMASK();
    __disable_irq();

    if ((s_data_valid == 0U) || (s_last_error != HAL_ADC_ERROR_NONE))
    {
        __set_PRIMASK(primask);
        return false;
    }

    result->raw = s_last_raw;
    result->average = s_last_average;
    result->sample_count = s_sample_count;

    __set_PRIMASK(primask);

    /*
     * ADC 12 位量化范围：0~4095。
     * 乘以 3300 / 4095 可得到近似电压（mV），仅供调试。
     * 光敏电阻不是照度计，不能直接把本电压称为 lux。
     */
    result->voltage_mv = (uint16_t)(((uint32_t)result->average *
                                         BSP_ADC_VREF_MV +
                                     2047U) /
                                    4095U);

    return true;
}

/** @brief 获取最近一次 ADC 错误码，0 表示未记录错误。 */
uint32_t BSP_ADC_GetLastError(void)
{
    return s_last_error;
}

/**
 * @brief 显式重新启动采样链路。
 * @note  仅在主循环中调用，不要放在中断回调内。
 *        如果错误已经导致 HAL 停止 DMA，Stop_DMA 可能返回错误，
 *        这里仍尝试重新启动；启动失败的错误码会直接返回给应用层。
 */
HAL_StatusTypeDef BSP_ADC_Restart(void)
{
    (void)HAL_TIM_Base_Stop(&htim2);
    (void)HAL_ADC_Stop_DMA(&hadc3);
    s_running = 0U;
    s_data_valid = 0U;

    return BSP_ADC_Init();
}

/**
 * @brief DMA 半传输完成回调：前 16 个样本已经接收完成。
 * @note  下一阶段 DMA 正在写后 16 个位置，所以前半区可以安全求平均。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC3) && (s_running != 0U) &&
        (s_last_error == HAL_ADC_ERROR_NONE))
    {
        BSP_ADC_ProcessHalf(0U);
    }
}

/**
 * @brief DMA 传输完成回调：后 16 个样本已经接收完成。
 * @note  Circular 模式会自动回到索引 0，无须再次调用 HAL_ADC_Start_DMA()。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC3) && (s_running != 0U) &&
        (s_last_error == HAL_ADC_ERROR_NONE))
    {
        BSP_ADC_ProcessHalf(BSP_ADC_HALF_COUNT);
    }
}

/**
 * @brief ADC 错误回调。
 * @note  记录 HAL 错误位掩码，不在中断里调用 printf 或重启 DMA。
 *        应用层检测到错误后，可以调用 BSP_ADC_Restart()。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC3)
    {
        s_last_error = HAL_ADC_GetError(hadc);
        s_data_valid = 0U;
    }
}
