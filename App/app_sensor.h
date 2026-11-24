#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdbool.h>
#include "BSP/ADC/bsp_adc.h"

/**
 * @brief 创建传感器任务及数据队列
 */
void APP_Sensor_TaskCreate(void);

/**
 * @brief 获取最近一次发布的传感器数据
 *
 * @param result 用于保存传感器数据的结构体指针
 * @return true  获取成功
 * @return false 暂时没有有效数据
 *
 * @note 非阻塞接口，不会取走队列中的数据
 */
bool APP_Sensor_GetLatest(BSP_ADC_Result_t *result);

#endif