/**
 * @file    bsp_flash.h
 * @brief   板载 SPI NOR Flash（W25Q128，16MiB，SPI1 + PB14 片选）底层驱动接口
 * @note    NOR Flash 物理约束：
 *          - 擦除最小单位为 4KiB 扇区，地址必须 4096 字节对齐。
 *          - 写入仅能将 bit 从 1 置为 0，覆写旧数据前必须先擦除目标扇区。
 *          - BSP_FLASH_Write() 内部已处理 256 字节跨页拆分，但不自动执行擦除。
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 预留给板级读写自测使用的扇区与地址（避开 0x00100000 起的日志分区） */
#define FLASH_TEST_SECTOR 0x00FF0000UL
#define FLASH_TEST_ADDR   0x00FF00F0UL

/**
 * @brief  读取 SPI Flash 3 字节 JEDEC ID（W25Q128 正常为 0xEF4018）
 */
HAL_StatusTypeDef BSP_FLASH_ReadJEDECID(uint32_t *id);

/**
 * @brief  读取状态寄存器 SR1（bit0 = BUSY 忙标志，bit1 = WEL 写使能锁存）
 */
HAL_StatusTypeDef BSP_FLASH_ReadStatusReg1(uint8_t *reg_value);

/**
 * @brief  发送写使能指令（0x06），擦除或页编程前必须调用
 */
HAL_StatusTypeDef BSP_FLASH_WriteEnable(void);

/**
 * @brief  轮询等待 Flash 内部擦除/编程操作完成（SR1.BUSY 清零）
 * @param  timeout 最大等待超时时间（单位：ms）
 */
HAL_StatusTypeDef BSP_FLASH_WaitBusy(uint32_t timeout);

/**
 * @brief  从指定 24 位物理地址连续读取 size 字节到 buffer
 */
HAL_StatusTypeDef BSP_FLASH_Read(
    uint32_t address,
    uint8_t *buffer,
    uint32_t size);

/**
 * @brief  擦除指定 4KiB 扇区（address 必须 4096 字节对齐）
 */
HAL_StatusTypeDef BSP_FLASH_EraseSector(uint32_t address);

/**
 * @brief  向指定地址写入数据（自动按 256 字节页边界拆分，不自动擦除）
 */
HAL_StatusTypeDef BSP_FLASH_Write(
    uint32_t address,
    uint8_t *buffer,
    uint32_t size);

#endif /* BSP_FLASH_H */