#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stddef.h>

/* 0x03：读取保持寄存器；每个寄存器为 16 位。 */
#define MODBUS_FC_READ_HOLDING 0x03U
/* 读请求固定为 8 字节：从站地址 1 + 功能码 1 + 起始地址 2 + 数量 2 + CRC 2。 */
#define MODBUS_READ_REQ_SIZE 8U
/* RTU ADU（完整协议帧）的最大长度，包含从站地址和 CRC；当前组帧不需要这么大。 */
#define MODBUS_MAX_ADU_SIZE 256U

/**
 * @brief 构造一帧读取保持寄存器的 Modbus RTU 请求，不执行串口发送。
 * @param slave 从站地址，范围 1～247；读请求不接受广播地址 0。
 * @param start_addr 协议中的零基起始地址，范围 0x0000～0xFFFF。
 *                   若设备文档以 40001 表示首个保持寄存器，通常应传入 0；
 *                   具体映射仍须以该设备的寄存器表为准。
 * @param quantity 连续读取的寄存器数量，范围 1～125，单位不是字节。
 * @param out 调用方提供的输出缓冲区，不能为空。
 * @param capacity out 可写入的字节数，至少为 MODBUS_READ_REQ_SIZE。
 * @return 成功返回 8（可发送的字节数）；参数非法返回 0，且不修改 out。
 * @note 最后一个寄存器地址不能超过 0xFFFF。
 *       地址和数量按高字节在前编码，CRC 按低字节在前编码。
 *       函数不保存缓冲区指针，也不负责响应解析、超时或 RTU 帧间隔控制。
 */
size_t Modbus_BuildReadHolding(
    uint8_t slave,
    uint16_t start_addr,
    uint16_t quantity,
    uint8_t *out,
    size_t capacity);

#endif
