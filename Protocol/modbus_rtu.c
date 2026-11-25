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
