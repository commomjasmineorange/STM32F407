#ifndef APP_STORAGE_H
#define APP_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

/* 数据来源编号：用来标记这条数据是谁采的 */
#define LOG_SRC_MODBUS 1U
#define LOG_SRC_ADC 2U

/* 创建存储队列和存储任务（在 APP_Task_Init 里调用一次） */
void APP_Storage_TaskCreate(void);

/* 往存储队列里塞一条数据（给采集任务调用，非阻塞，不会卡住采集任务） */
bool APP_Storage_Push(uint16_t source, uint16_t val0, uint16_t val1);

/* 打印最近 max_count 条历史记录到 USART1（方便验证掉电恢复，也给后面命令行用） */
void APP_Storage_DumpRecent(uint16_t max_count);

#endif /* APP_STORAGE_H */