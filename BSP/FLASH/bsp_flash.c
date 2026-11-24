#include "bsp_flash.h"
#include "spi.h"
#include <string.h>

/* ======================== 硬件连接 ======================== */
#define FLASH_CS_PORT GPIOB
#define FLASH_CS_PIN GPIO_PIN_14

/* ======================== 存储容量与组织 ======================== */
#define FLASH_TOTAL_SIZE 0x01000000UL /* 16 MiB */
#define FLASH_SECTOR_SIZE 4096U       /* 4 KiB */
#define FLASH_PAGE_SIZE 256U          /* 256 bytes */

/* ======================== SPI 指令 ======================== */
#define FLASH_CMD_READ_SR1 0x05U     /* Read status register 1 */
#define FLASH_CMD_WRITE_ENABLE 0x06U /* Write enable */
#define FLASH_CMD_READ_DATA 0x03U    /* Read data + 24-bit address */
#define FLASH_CMD_SECTOR_ERASE 0x20U /* Erase 4 KiB sector */
#define FLASH_CMD_PAGE_PROGRAM 0x02U /* Page program */

/* ======================== SR1 状态位 ======================== */
#define FLASH_SR1_BUSY_MASK 0x01U /* bit 0: BUSY */
#define FLASH_SR1_WEL_MASK 0x02U  /* bit 1: write enable latch */

/* ======================== 超时时间（ms） ======================== */
#define FLASH_SPI_TIMEOUT 1000U     /* HAL SPI transfer timeout */
#define FLASH_DEFAULT_TIMEOUT 1000U /* Read/erase busy timeout */
#define FLASH_PROGRAM_TIMEOUT 100U  /* Program busy timeout */

/* ======================== 传输缓冲区 ======================== */
#define FLASH_SPI_DUMMY_SIZE 32U /* Dummy TX bytes per receive block */

/* 选中 Flash */
/* CS is active low; keep it asserted for the entire command transaction. */
static void FLASH_CS_Select(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

/* 取消选中 Flash */
static void FLASH_CS_Diselect(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

/**
 * @brief SPI 发送数据
 * @param data 发送数据缓冲区
 * @param size 发送数据长度
 */
static HAL_StatusTypeDef FLASH_SPI_Transmit(
    uint8_t *data,
    uint16_t size)
{
    if ((data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_SPI_Transmit(
        &hspi1,
        data,
        size,
        FLASH_SPI_TIMEOUT);
}

/**
 * @brief SPI 同时发送和接收数据
 */
static HAL_StatusTypeDef FLASH_SPI_TransmitReceive(
    uint8_t *tx_data,
    uint8_t *rx_data,
    uint16_t size)
{
    if ((tx_data == NULL) ||
        (rx_data == NULL) ||
        (size == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_SPI_TransmitReceive(
        &hspi1,
        tx_data,
        rx_data,
        size,
        FLASH_SPI_TIMEOUT);
}

/**
 * @brief SPI接收数据
 * @param rx_data 接收数据缓冲
 * @param size 要接收的大小
 * @return
 */
/* Send dummy bytes to generate receive clocks; caller controls CS. */
static HAL_StatusTypeDef FLASH_SPI_Receive(
    uint8_t *rx_data,
    uint32_t size)
{
    uint8_t dummy[FLASH_SPI_DUMMY_SIZE];

    /* 用 0xFF 填充占位数据 */
    memset(dummy, 0xFF, sizeof(dummy));

    if ((rx_data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    // 分块处理数据
    while (size > 0U)
    {
        // 确保每次处理size个数据（未满）或者处理FLASH_SPI_DUMMY_SIZE（满）
        uint16_t len = (size > FLASH_SPI_DUMMY_SIZE)
                           ? FLASH_SPI_DUMMY_SIZE
                           : (uint16_t)size;

        HAL_StatusTypeDef ret =
            FLASH_SPI_TransmitReceive(
                dummy,
                rx_data,
                len);

        if (ret != HAL_OK)
        {
            return ret;
        }

        rx_data += len;
        size -= len;
    }

    return HAL_OK;
}

/**
 * @brief 读取 Flash JEDEC ID
 * @param id 用于保存读取结果的指针
 * @retval HAL_OK   读取成功
 *         HAL_ERROR 参数错误或返回明显无效的 ID
 *         其他       SPI 通信错误
 */
HAL_StatusTypeDef BSP_FLASH_ReadJEDECID(uint32_t *id)
{
    /* 第一个字节为读 ID 命令
       后三个字节用于产生时钟 */
    uint8_t tx_buf[4] = {0x9F, 0xFF, 0xFF, 0xFF};

    /* 接收 Flash 返回的数据 */
    uint8_t rx_buf[4] = {0};

    HAL_StatusTypeDef status;

    if (id == NULL)
    {
        return HAL_ERROR;
    }

    *id = 0U;

    /* 1. 选中 Flash */
    FLASH_CS_Select();

    /* 2. 发送命令并接收 ID */
    status = FLASH_SPI_TransmitReceive(
        tx_buf,
        rx_buf,
        4);

    /* 3. 结束本次通信
       即使通信失败，也要释放片选 */
    FLASH_CS_Diselect();

    if (status != HAL_OK)
    {
        return status;
    }

    /* 4. 拼接三个 ID 字节
       rx_buf[0] 是发送命令期间收到的无效数据 */
    *id = ((uint32_t)rx_buf[1] << 16) |
          ((uint32_t)rx_buf[2] << 8) |
          (uint32_t)rx_buf[3];

    /* 全 0 或全 1 通常意味着通信异常 */
    if ((*id == 0x000000U) || (*id == 0xFFFFFFU))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief 读取 Flash 状态寄存器 SR1
 * @param reg_value 用于保存状态寄存器的指针
 * @retval HAL_OK      读取成功
 *         HAL_ERROR   参数错误
 *         其他        SPI 通信错误
 */
HAL_StatusTypeDef BSP_FLASH_ReadStatusReg1(uint8_t *reg_value)
{

    uint8_t tx_buf[2] = {FLASH_CMD_READ_SR1, 0xFF};

    /* 接收 Flash 返回的数据 */
    uint8_t rx_buf[2] = {0};

    HAL_StatusTypeDef status;

    /* 1. 选中 Flash */
    FLASH_CS_Select();
    status = FLASH_SPI_TransmitReceive(
        tx_buf,
        rx_buf,
        2);

    FLASH_CS_Diselect();
    if (status != HAL_OK)
    {
        return status;
    }

    *reg_value = rx_buf[1];

    return HAL_OK;
}

/**
 * @brief Flash 写使能
 * @retval HAL_OK      指令发送成功
 *         其他        SPI 通信错误
 */
HAL_StatusTypeDef BSP_FLASH_WriteEnable(void)
{
    /* 准备写使能指令 */
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;

    HAL_StatusTypeDef status;

    /* 1. 拉低 CS，选中 Flash */
    FLASH_CS_Select();
    /* 2. 通过 SPI 发送一个字节 */
    status = FLASH_SPI_Transmit(
        &cmd,
        1);
    /* 3. 拉高 CS，结束通信 */
    FLASH_CS_Diselect();

    /* 4. 返回 SPI 通信状态 */

    return status;
}

/**
 * @brief 等待 Flash 内部操作完成
 * @param timeout 最大等待时间，单位 ms
 * @retval HAL_OK      Flash 空闲
 *         HAL_TIMEOUT 等待超时
 *         其他        SPI 通信错误
 */
HAL_StatusTypeDef BSP_FLASH_WaitBusy(uint32_t timeout)
{
    uint8_t status_reg = 0U;
    uint32_t start_tick = HAL_GetTick();

    while (1)
    {
        /* 1. 读取状态寄存器 */
        HAL_StatusTypeDef ret =
            BSP_FLASH_ReadStatusReg1(&status_reg);

        /* 2. 检查 SPI 通信是否成功 */
        if (ret != HAL_OK)
        {
            return ret;
        }

        /* 3. 检查 BUSY 是否清零 */
        if ((status_reg & FLASH_SR1_BUSY_MASK) == 0U)
        {
            return HAL_OK;
        }

        /* 4. 检查是否超时 */
        if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout)
        {
            return HAL_TIMEOUT;
        }

        /* 5. 短暂等待，避免一直占用 CPU */
        HAL_Delay(1U);
    }
}

/**
 * @brief 从 Flash 指定地址读取数据
 *
 * @param address Flash 起始地址
 * @param buffer  数据接收缓冲区
 * @param size    读取字节数
 *
 * @retval HAL_OK      读取成功
 *         HAL_ERROR   参数错误
 *         其他        SPI 通信错误或等待超时
 */
HAL_StatusTypeDef BSP_FLASH_Read(
    uint32_t address,
    uint8_t *buffer,
    uint32_t size)
{
    uint8_t cmd_addr[4];

    HAL_StatusTypeDef ret;

    /* 1. 检查参数 */
    if ((buffer == NULL) ||
        (size == 0U) ||
        (address >= FLASH_TOTAL_SIZE) ||
        (size > FLASH_TOTAL_SIZE - address))
    {
        return HAL_ERROR;
    }

    /* 2. 确保 Flash 当前处于空闲状态 */
    ret = BSP_FLASH_WaitBusy(FLASH_DEFAULT_TIMEOUT);

    if (ret != HAL_OK)
    {
        return ret;
    }

    /* 3. 准备读取命令和 24 位地址 */
    cmd_addr[0] = FLASH_CMD_READ_DATA;

    cmd_addr[1] = (uint8_t)(address >> 16);
    cmd_addr[2] = (uint8_t)(address >> 8);
    cmd_addr[3] = (uint8_t)address;

    /* 4. 选中 Flash */
    FLASH_CS_Select();

    /* 5. 发送读取命令和地址 */
    ret = FLASH_SPI_Transmit(
        cmd_addr,
        sizeof(cmd_addr));

    /* 6. 命令发送成功后，开始接收数据 */
    if (ret == HAL_OK)
    {
        ret = FLASH_SPI_Receive(buffer, size);
    }

    /* 7. 结束通信 */
    FLASH_CS_Diselect();

    return ret;
}

/**
 * @brief 擦除指定的 4KB 扇区
 *
 * @param address 扇区起始地址，必须 4KB 对齐
 *
 * @retval HAL_OK      擦除成功
 *         HAL_ERROR   参数错误或写使能失败
 *         HAL_TIMEOUT 擦除超时
 *         其他        SPI 通信错误
 */
HAL_StatusTypeDef BSP_FLASH_EraseSector(uint32_t address)
{
    uint8_t cmd_addr[4];
    uint8_t sr1 = 0U;

    HAL_StatusTypeDef ret;

    /* 1. 检查地址是否合法 */
    if ((address >= FLASH_TOTAL_SIZE) ||
        ((address % FLASH_SECTOR_SIZE) != 0U))
    {
        return HAL_ERROR;
    }

    /* 2. 等待 Flash 空闲 */
    ret = BSP_FLASH_WaitBusy(FLASH_DEFAULT_TIMEOUT);

    if (ret != HAL_OK)
    {
        return ret;
    }

    /* 3. 执行写使能 */
    ret = BSP_FLASH_WriteEnable();

    if (ret != HAL_OK)
    {
        return ret;
    }

    /* 4. 读取 SR1，检查 WEL 是否置位 */
    ret = BSP_FLASH_ReadStatusReg1(&sr1);

    if (ret != HAL_OK)
    {
        return ret;
    }

    if ((sr1 & FLASH_SR1_WEL_MASK) == 0U)
    {
        return HAL_ERROR;
    }

    /* 5. 准备擦除命令和 24 位地址 */
    cmd_addr[0] = FLASH_CMD_SECTOR_ERASE;

    cmd_addr[1] = (uint8_t)(address >> 16);
    cmd_addr[2] = (uint8_t)(address >> 8);
    cmd_addr[3] = (uint8_t)address;

    /* 6. 选中 Flash */
    FLASH_CS_Select();

    /* 7. 发送擦除命令和地址 */
    ret = FLASH_SPI_Transmit(
        cmd_addr,
        sizeof(cmd_addr));

    /* 8. 结束通信，启动 Flash 内部擦除 */
    FLASH_CS_Diselect();

    if (ret != HAL_OK)
    {
        return ret;
    }

    /* 9. 等待 Flash 擦除完成 */
    ret = BSP_FLASH_WaitBusy(FLASH_DEFAULT_TIMEOUT);

    return ret;
}

/**
 * @brief 向 Flash 写入数据，支持跨页写入
 *
 * @param address Flash 起始地址
 * @param buffer  待写入数据缓冲区
 * @param size    写入数据长度
 *
 * @retval HAL_OK      写入成功
 *         HAL_ERROR   参数错误或写使能失败
 *         HAL_TIMEOUT Flash 操作超时
 *         其他        SPI 通信错误
 */
/* Caller must erase the target region when needed; this function only programs. */
HAL_StatusTypeDef BSP_FLASH_Write(
    uint32_t address,
    uint8_t *buffer,
    uint32_t size)
{
    uint8_t cmd_addr[4];
    uint8_t sr1;

    uint32_t page_offset;
    uint32_t page_remain;
    uint32_t write_size;

    HAL_StatusTypeDef ret;

    /* 1. 检查参数是否合法 */
    if ((buffer == NULL) ||
        (size == 0U) ||
        (address >= FLASH_TOTAL_SIZE) ||
        (size > FLASH_TOTAL_SIZE - address))
    {
        return HAL_ERROR;
    }

    /* 2. 确保 Flash 当前处于空闲状态 */
    ret = BSP_FLASH_WaitBusy(FLASH_PROGRAM_TIMEOUT);

    if (ret != HAL_OK)
    {
        return ret;
    }

    /* 3. 循环处理，直到全部数据写入完成 */
    while (size > 0U)
    {
        /* 计算当前地址所在 Page 的偏移 */
        page_offset = address % FLASH_PAGE_SIZE;

        /* 计算当前 Page 剩余空间 */
        page_remain = FLASH_PAGE_SIZE - page_offset;

        /* 确定本次写入长度 */
        write_size = (size < page_remain) ? size : page_remain;

        /* 4. 写使能 */
        ret = BSP_FLASH_WriteEnable();

        if (ret != HAL_OK)
        {
            return ret;
        }

        /* 5. 检查 WEL 是否置位 */
        ret = BSP_FLASH_ReadStatusReg1(&sr1);

        if (ret != HAL_OK)
        {
            return ret;
        }

        if ((sr1 & FLASH_SR1_WEL_MASK) == 0U)
        {
            return HAL_ERROR;
        }

        /* 6. 准备页编程命令和 24 位地址 */
        cmd_addr[0] = FLASH_CMD_PAGE_PROGRAM;

        cmd_addr[1] = (uint8_t)(address >> 16);
        cmd_addr[2] = (uint8_t)(address >> 8);
        cmd_addr[3] = (uint8_t)address;

        /* 7. 选中 Flash */
        FLASH_CS_Select();

        /* 8. 发送页编程命令和地址 */
        ret = FLASH_SPI_Transmit(
            cmd_addr,
            sizeof(cmd_addr));

        /* 9. 发送实际数据 */
        if (ret == HAL_OK)
        {
            ret = FLASH_SPI_Transmit(
                buffer,
                (uint16_t)write_size);
        }

        /* 10. 结束通信，启动内部编程 */
        FLASH_CS_Diselect();

        if (ret != HAL_OK)
        {
            return ret;
        }

        /* 11. 等待当前 Page 编程完成 */
        ret = BSP_FLASH_WaitBusy(FLASH_PROGRAM_TIMEOUT);

        if (ret != HAL_OK)
        {
            return ret;
        }

        /* 12. 更新地址、数据指针和剩余长度 */
        address += write_size;
        buffer += write_size;
        size -= write_size;
    }

    return HAL_OK;
}