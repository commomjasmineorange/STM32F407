/**
 * @file    app_monitor.h
 * @brief   FreeRTOS 系统运行健康监控任务对外接口
 */

#ifndef APP_MONITOR_H
#define APP_MONITOR_H

/**
 * @brief  创建系统监控任务（周期采集剩余堆内存与全部任务状态快照）
 * @note   需在启动调度器前由 APP_Task_Init() 调用一次。
 */
void APP_Monitor_TaskCreate(void);

#endif /* APP_MONITOR_H */