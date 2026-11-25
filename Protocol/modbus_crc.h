#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief 计算 Modbus RTU 使用的 CRC-16 校验值。
 * @param data 待校验数据的首地址；len > 0 时必须指向至少 len 字节的有效内存。
 * @param len 参与计算的字节数，不包含帧尾已有的两个 CRC 字节。
 * @return 16 位 CRC 数值。写入 RTU 帧时，应先放低字节，再放高字节。
 * @note 初始值为 0xFFFF，使用反射多项式 0xA001，结束时不再取反。
 *       本函数不检查空指针；len 为 0 时不访问 data，返回初始值 0xFFFF。
 */
uint16_t Modbus_CRC16(
    const uint8_t *data,
    size_t len);

#endif
