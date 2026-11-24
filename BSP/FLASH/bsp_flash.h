#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Test addresses: reserve this region before erase/write tests. */
#define FLASH_TEST_SECTOR 0x00FF0000UL
#define FLASH_TEST_ADDR 0x00FF00F0UL
/* 读取 Flash 的 3 字节 JEDEC ID */
HAL_StatusTypeDef BSP_FLASH_ReadJEDECID(uint32_t *id);
/* Read SR1: bit 0 = BUSY, bit 1 = WEL. */
HAL_StatusTypeDef BSP_FLASH_ReadStatusReg1(uint8_t *reg_value);
/* Set write enable before erase or page program. */
HAL_StatusTypeDef BSP_FLASH_WriteEnable(void);
/* Poll BUSY until idle; timeout is in milliseconds. */
HAL_StatusTypeDef BSP_FLASH_WaitBusy(uint32_t timeout);
/* Read size bytes starting at address into buffer. */
HAL_StatusTypeDef BSP_FLASH_Read(
    uint32_t address,
    uint8_t *buffer,
    uint32_t size);
/* Erase one 4 KiB sector; address must be 4096-byte aligned. */
HAL_StatusTypeDef BSP_FLASH_EraseSector(uint32_t address);
/* Split writes at page boundaries; does not automatically erase. */
HAL_StatusTypeDef BSP_FLASH_Write(
    uint32_t address,
    uint8_t *buffer,
    uint32_t size);

#endif