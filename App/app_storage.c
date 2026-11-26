/**
 * @file    app_storage.c
 * @brief   SPI Flash 掉电环形日志存储任务（FreeRTOS 单一写者模式）
 * @note    设计要点：
 *          1. 并发解耦：外部任务（如 ADC/Modbus 采集任务）仅通过 APP_Storage_Push()
 *             非阻塞向 s_storage_queue 投递消息；全系统只有 Storage_Task 独占访问 SPI Flash，
 *             避免擦写扇区（40~100ms）阻塞采集任务或多任务竞争 SPI1 总线。
 *          2. 定长对齐：每条记录 LogRecord_t 严格 32 字节（整除 256B 页与 4KB 扇区），
 *             永不跨页/跨扇区写入；共占用 4 个扇区（16KB，512 个循环槽位）。
 *          3. 掉电恢复：开机执行 Storage_RecoverOnBoot() 扫描全部 512 槽位，
 *             以“魔数 + CRC16”过滤未写区或断电残帧，按最大 seq 恢复写入进度。
 */

#include "app_storage.h"
#include "BSP/FLASH/bsp_flash.h"
#include "Protocol/modbus_crc.h" /* 复用纯 C 协议层的 CRC-16 校验算法 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/*================ 1. 任务与队列配置 ================*/

#define STORAGE_TASK_STACK 512U /* 512 Word = 2048 字节（调用 printf 需充足栈空间） */
#define STORAGE_TASK_PRIO  2U   /* 优先级低于 UART/Modbus(3)，不抢占实时通信 */
#define STORAGE_QUEUE_LEN  16U  /* 待落盘消息队列深度：最多缓存 16 条采集记录 */

/* 采集任务 -> 存储任务之间通过 Queue 传递的轻量消息 */
typedef struct
{
    uint32_t tick;   /* 采集时刻的系统节拍（ms） */
    uint16_t source; /* 数据来源（LOG_SRC_MODBUS / LOG_SRC_ADC） */
    uint16_t val0;   /* 业务数值 0 */
    uint16_t val1;   /* 业务数值 1 */
} StorageMsg_t;

/*================ 2. SPI Flash 环形日志分区配置 ================*/

#define LOG_BASE_ADDR     0x00100000UL                             /* 日志区起始地址：1MB 处 */
#define LOG_SECTOR_SIZE   4096U                                    /* W25Q128 单扇区擦除粒度：4KB */
#define LOG_SECTOR_COUNT  4U                                       /* 日志区占用 4 个扇区（共 16KB） */
#define LOG_RECORD_SIZE   32U                                      /* 单条持久化记录固定 32 字节 */
#define LOG_SLOTS_PER_SEC (LOG_SECTOR_SIZE / LOG_RECORD_SIZE)      /* 每扇区 128 个槽位 */
#define LOG_TOTAL_SLOTS   (LOG_SLOTS_PER_SEC * LOG_SECTOR_COUNT)   /* 环形区总槽位数：512 */
#define LOG_MAGIC         0xA55A1234UL                             /* 有效记录魔数，用于区分 0xFF 空白区 */

/* 真正落盘到 SPI Flash 的 32 字节定长记录布局（自然对齐，无隐藏填充字节） */
typedef struct
{
    uint32_t magic;       /* 偏移 0..3  ：魔数 LOG_MAGIC */
    uint32_t seq;         /* 偏移 4..7  ：跨重启严格递增序号（从 1 开始） */
    uint32_t tick;        /* 偏移 8..11 ：采集时间戳（FreeRTOS tick） */
    uint16_t source;      /* 偏移 12..13：数据来源标记 */
    uint16_t val0;        /* 偏移 14..15：采集值 0 */
    uint16_t val1;        /* 偏移 16..17：采集值 1 */
    uint8_t reserved[12]; /* 偏移 18..29：保留扩展区（写入前清零） */
    uint16_t crc16;       /* 偏移 30..31：前 30 字节的 CRC-16 校验码 */
} LogRecord_t;

/* C11 编译期断言：锁死结构体大小为 32 字节，防止后续修改破坏对齐约定 */
_Static_assert(sizeof(LogRecord_t) == LOG_RECORD_SIZE, "LogRecord_t must be 32 bytes");

/*================ 3. 私有状态变量 ================*/

static uint32_t      s_write_slot    = 0U;   /* 下一次写入的目标槽位（0 ~ 511） */
static uint32_t      s_next_seq      = 1U;   /* 下一条记录的递增序号（从 1 开始） */
static QueueHandle_t s_storage_queue = NULL; /* 生产者 -> 存储任务的 FIFO 消息队列 */

static void Storage_Task(void *argument);

/*================ 4. 内部 Flash 日志操作函数 ================*/

/**
 * @brief  将环形槽位索引（0 ~ 511）换算为 SPI Flash 物理地址
 */
static inline uint32_t SlotToAddr(uint32_t slot)
{
    return LOG_BASE_ADDR + (slot * LOG_RECORD_SIZE);
}

/**
 * @brief  将一条消息打包为 32 字节持久化记录并追加写入 SPI Flash
 * @param  msg 来自存储队列的采集消息指针
 * @retval true  写入成功
 * @retval false 擦除或写入失败
 */
static bool Storage_AppendRecord(const StorageMsg_t *msg)
{
    uint32_t addr = SlotToAddr(s_write_slot);

    /*
     * NOR Flash 物理特性：只能将 bit 从 1 写为 0，不能直接从 0 改回 1。
     * 每当写指针落到某个 4KB 扇区的第 0 槽位（0/128/256/384）时，先整扇区擦除为 0xFF。
     */
    if ((s_write_slot % LOG_SLOTS_PER_SEC) == 0U)
    {
        if (BSP_FLASH_EraseSector(addr) != HAL_OK)
        {
            printf("[STORAGE] Erase FAIL at 0x%08lX\r\n", addr);
            return false;
        }
    }

    /* 1. 填充 32 字节记录并计算前 30 字节的 CRC-16 */
    LogRecord_t rec;
    memset(&rec, 0, sizeof(rec)); /* 先清零，确保 reserved 字段内容确定 */
    rec.magic  = LOG_MAGIC;
    rec.seq    = s_next_seq;
    rec.tick   = msg->tick;
    rec.source = msg->source;
    rec.val0   = msg->val0;
    rec.val1   = msg->val1;
    rec.crc16  = Modbus_CRC16((const uint8_t *)&rec, sizeof(LogRecord_t) - 2U);

    /* 2. 写入目标槽位（单条 32 字节严格落在单页内，不会跨页） */
    if (BSP_FLASH_Write(addr, (uint8_t *)&rec, sizeof(rec)) != HAL_OK)
    {
        printf("[STORAGE] Write FAIL at 0x%08lX\r\n", addr);
        return false;
    }

    /* 3. 打印确认日志，并推进序号与环形写指针 */
    printf("[STORAGE] Saved seq=%lu slot=%lu val0=%u val1=%u\r\n",
           rec.seq, s_write_slot, rec.val0, rec.val1);

    s_next_seq++;
    s_write_slot = (s_write_slot + 1U) % LOG_TOTAL_SLOTS;
    return true;
}

/**
 * @brief  校验一条从 Flash 读出的记录是否完整合法（魔数匹配 + CRC16 一致）
 */
static bool RecordIsValid(const LogRecord_t *rec)
{
    if (rec->magic != LOG_MAGIC)
    {
        return false;
    }

    uint16_t calc = Modbus_CRC16((const uint8_t *)rec, sizeof(LogRecord_t) - 2U);
    return (calc == rec->crc16);
}

/**
 * @brief  开机掉电恢复：全量扫描 512 个槽位，定位断电前最新一条合法记录
 * @note   若某条记录在写入中途断电导致残缺，其 CRC16 必然失败，会被自动跳过；
 *         系统从最大合法 seq 的下一槽位继续追加，避免覆盖历史有效数据。
 */
static void Storage_RecoverOnBoot(void)
{
    LogRecord_t rec;
    uint32_t max_seq   = 0U;
    uint32_t max_slot  = 0U;
    uint32_t valid_cnt = 0U;

    for (uint32_t slot = 0U; slot < LOG_TOTAL_SLOTS; slot++)
    {
        if (BSP_FLASH_Read(SlotToAddr(slot), (uint8_t *)&rec, sizeof(rec)) != HAL_OK)
        {
            continue;
        }

        if (RecordIsValid(&rec))
        {
            valid_cnt++;
            if (rec.seq > max_seq)
            {
                max_seq  = rec.seq;
                max_slot = slot;
            }
        }
    }

    if (max_seq > 0U)
    {
        s_next_seq   = max_seq + 1U;
        s_write_slot = (max_slot + 1U) % LOG_TOTAL_SLOTS;
    }
    else
    {
        s_next_seq   = 1U;
        s_write_slot = 0U;
    }

    printf("[STORAGE] Boot recover: valid=%lu next_seq=%lu next_slot=%lu\r\n",
           valid_cnt, s_next_seq, s_write_slot);
}

/*================ 5. 对外接口与任务主体 ================*/

/**
 * @brief  创建存储消息队列与 Storage_Task
 */
void APP_Storage_TaskCreate(void)
{
    s_storage_queue = xQueueCreate(STORAGE_QUEUE_LEN, sizeof(StorageMsg_t));
    if (s_storage_queue == NULL)
    {
        Error_Handler();
    }

    BaseType_t ret = xTaskCreate(
        Storage_Task,
        "STORAGE",
        STORAGE_TASK_STACK,
        NULL,
        STORAGE_TASK_PRIO,
        NULL);

    if (ret != pdPASS)
    {
        Error_Handler();
    }
}

/**
 * @brief  非阻塞向存储队列投递一条待持久化记录
 * @param  source 数据来源编号（LOG_SRC_MODBUS / LOG_SRC_ADC）
 * @param  val0   数据字段 0
 * @param  val1   数据字段 1
 * @retval true   入队成功
 * @retval false  队列未初始化或已满（绝不阻塞调用方任务）
 */
bool APP_Storage_Push(uint16_t source, uint16_t val0, uint16_t val1)
{
    if (s_storage_queue == NULL)
    {
        return false;
    }

    StorageMsg_t msg;
    msg.tick   = xTaskGetTickCount();
    msg.source = source;
    msg.val0   = val0;
    msg.val1   = val1;

    if (xQueueSend(s_storage_queue, &msg, 0U) != pdPASS)
    {
        return false;
    }
    return true;
}

/**
 * @brief  从最新记录开始倒序打印最近 max_count 条合法历史日志到 USART1
 */
void APP_Storage_DumpRecent(uint16_t max_count)
{
    LogRecord_t rec;
    uint16_t printed = 0U;

    printf("--- Recent %u Logs ---\r\n", max_count);

    for (uint32_t step = 1U; (step <= LOG_TOTAL_SLOTS) && (printed < max_count); step++)
    {
        /* 环形倒推下标：先加总容量再减 step 后取模，避免无符号整数下溢 */
        uint32_t slot = (s_write_slot + LOG_TOTAL_SLOTS - step) % LOG_TOTAL_SLOTS;

        if (BSP_FLASH_Read(SlotToAddr(slot), (uint8_t *)&rec, sizeof(rec)) != HAL_OK)
        {
            break;
        }

        if (RecordIsValid(&rec))
        {
            printf("  [slot=%3lu] seq=%lu tick=%lu src=%u val0=%u val1=%u\r\n",
                   slot, rec.seq, rec.tick, rec.source, rec.val0, rec.val1);
            printed++;
        }
    }

    if (printed == 0U)
    {
        printf("  (empty)\r\n");
    }
}

/**
 * @brief  存储任务主循环：开机硬件自检与日志恢复，随后休眠等待队列消息落盘
 */
static void Storage_Task(void *argument)
{
    (void)argument;

    StorageMsg_t msg;
    uint32_t jedec_id = 0U;

    /* 1. 开机读取 SPI Flash JEDEC ID（华邦 W25Q128 正常为 0xEF4018） */
    if (BSP_FLASH_ReadJEDECID(&jedec_id) == HAL_OK)
    {
        printf("[STORAGE] Flash OK, ID=0x%06lX\r\n", jedec_id);
    }
    else
    {
        printf("[STORAGE] Flash Read ID FAILED!\r\n");
    }

    /* 2. 扫描恢复断电前写入进度，并打印最近 5 条历史记录作为验证 */
    Storage_RecoverOnBoot();
    APP_Storage_DumpRecent(5U);

    /* 3. 主循环：无数据时永久阻塞让出 CPU，收到消息立即顺序落盘 */
    for (;;)
    {
        if (xQueueReceive(s_storage_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            Storage_AppendRecord(&msg);
        }
    }
}
