#include "modbus_crc.h"

/*
 * 逐位计算 CRC，适合不希望额外存放查表数组的场合。
 * 多项式的常规表示为 0x8005；这里从最低位开始处理，使用反射表示 0xA001。
 * 返回值是数值意义上的 CRC，线上字节顺序由组帧函数负责安排。
 */
uint16_t Modbus_CRC16(const uint8_t *data, size_t len)
{
    /* 每一帧均从相同的初值开始，不能沿用上一帧的计算结果。 */
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        /* 将当前数据字节异或到 CRC 的低 8 位，再逐位处理该字节。 */
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            /* 判断移位前的最低位；移出 1 时需要异或生成多项式。 */
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                /* 移出 0 时只需右移，不进行多项式异或。 */
                crc = crc >> 1;
            }
        }
    }
    /* Modbus CRC 无最终异或步骤。 */
    return crc;
}
