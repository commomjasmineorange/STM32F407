#include "modbus_rtu.h"
#include "modbus_crc.h"

/*
 * 0x03 请求帧布局（下标对应 out）：
 *   [0]       [1]     [2] [3]       [4] [5]       [6] [7]
 *   从站地址  功能码  起始地址H/L   寄存器数量H/L  CRC低/高
 *
 * 此处只生成请求帧。调用方应在返回值非零时，将对应长度的数据交给串口发送。
 * 注意：请求中的 quantity 是寄存器数；正常响应的数据区字节数为 quantity * 2，
 * 响应格式与请求不同，不能直接用本函数的布局解释响应。
 */
size_t Modbus_BuildReadHolding(
    uint8_t slave,
    uint16_t start_addr,
    uint16_t quantity,
    uint8_t *out,
    size_t capacity)
{
    /*
     * 在写缓冲区之前统一检查参数，失败时保持原缓冲区不变。
     * 地址 0 用于广播，读操作需要从站应答，因此不能使用广播地址。
     * 0x03 单次最多读取 125 个寄存器，即 250 字节的寄存器数据。
     */
    if ((out == NULL) ||
        (capacity < MODBUS_READ_REQ_SIZE) ||
        (slave == 0U) ||
        (slave > 247U) ||
        (quantity == 0U) ||
        (quantity > 125U))
    {
        return 0U;
    }

    /*
     * 最后一个地址为 start_addr + quantity - 1，必须 <= 65535。
     * 等价地，start_addr + quantity 必须 <= 65536；恰好等于 65536 是合法的。
     * 先转为 uint32_t 再相加，确保检查过程中不会发生 16 位截断。
     */
    if (((uint32_t)start_addr + quantity) > 65536UL)
    {
        return 0U;
    }

    /* 字节 0：指定接收本次请求的从站。 */
    out[0] = slave;

    /* 字节 1：0x03，读取保持寄存器。 */
    out[1] = MODBUS_FC_READ_HOLDING;

    /* 字节 2～3：起始地址，高字节在前。例如 0x1234 编码为 0x12、0x34。 */
    out[2] = (uint8_t)(start_addr >> 8);
    out[3] = (uint8_t)start_addr;

    /* 字节 4～5：寄存器数量，同样使用高字节在前的顺序。 */
    out[4] = (uint8_t)(quantity >> 8);
    out[5] = (uint8_t)quantity;

    /* 校验覆盖地址、功能码和请求参数共 6 字节，不包含 CRC 字段自身。 */
    uint16_t crc = Modbus_CRC16(out, 6U);

    /* 字节 6～7：CRC 低字节在前，与上述 16 位协议字段的字节顺序不同。 */
    out[6] = (uint8_t)crc;
    out[7] = (uint8_t)(crc >> 8);

    /* 返回有效帧长度，而非缓冲区容量；发送时使用这个长度。 */
    return MODBUS_READ_REQ_SIZE;
}

/*
 * 0x03 正常响应帧布局：
 *   [0]       [1]     [2]          [3 .. 3+2*qty-1]      [最后2字节]
 *   从站地址   0x03    字节数(2*qty) 寄存器数据(高字节在前)  CRC低 / CRC高
 *   总长度 = 5 + 2 * quantity
 *
 * 0x03 异常响应帧布局（固定 5 字节）：
 *   [0]       [1]     [2]      [3]     [4]
 *   从站地址   0x83    异常码    CRC低   CRC高
 */
Modbus_Status_t Modbus_ParseReadHolding(
    const uint8_t *rx,
    size_t rx_len,
    uint8_t expect_slave,
    uint16_t expect_qty,
    uint16_t *out_regs,
    uint8_t *out_ex_code)
{
    /* 1. 检查入参 */
    if ((rx == NULL) || (out_regs == NULL) || (expect_qty == 0U) || (expect_qty > 125U))
    {
        return MODBUS_ERR_PARAM;
    }
    /* 任何合法 Modbus 响应帧（哪怕是异常帧）最少也有 5 字节 */
    if (rx_len < 5U)
    {
        return MODBUS_ERR_FRAME_LEN;
    }
    /* 2. 先校验 CRC16（覆盖除最后 2 字节外的所有数据） */
    uint16_t calc_crc = Modbus_CRC16(rx, rx_len - 2);
    /* Modbus RTU 的 CRC 是小端序：低字节在前，高字节在后 */
    uint16_t recv_crc = (uint16_t)rx[rx_len - 2] | (uint16_t)(rx[rx_len - 1] << 8);

    /*判断校验位*/
    if (calc_crc != recv_crc)
        return MODBUS_ERR_CRC;
    /*3 核对从站地址*/
    if (rx[0] != expect_slave)
    {
        return MODBUS_ERR_SLAVE_MISMATCH;
    }

    /* 4. 判断是不是从站返回的异常响应（0x03 | 0x80 = 0x83） */
    if (rx[1] == (MODBUS_FC_READ_HOLDING | 0x80U))
    {
        if (out_ex_code != NULL)
        {
            *out_ex_code = rx[2];
        }
        return MODBUS_ERR_EXCEPTION;
    }

    /* 5. 核对功能码是否为 0x03 */
    if (rx[1] != MODBUS_FC_READ_HOLDING)
    {
        return MODBUS_ERR_FUNC_MISMATCH;
    }
    /* 6. 核对字节数和总帧长 */
    uint8_t expect_bytes = (uint8_t)(expect_qty * 2U);
    size_t expect_len = 5U + (size_t)expect_bytes;
    if ((rx[2] != expect_bytes) || (rx_len != expect_len))
    {
        return MODBUS_ERR_FRAME_LEN;
    }
    /* 7. 按大端序（高字节在前）拼出每个 16 位寄存器值 */
    for (uint16_t i = 0U; i < expect_qty; i++)
    {
        size_t offset = 3U + (size_t)i * 2U;
        out_regs[i] = ((uint16_t)rx[offset] << 8) | (uint16_t)rx[offset + 1U];
    }
    return MODBUS_OK;
}
