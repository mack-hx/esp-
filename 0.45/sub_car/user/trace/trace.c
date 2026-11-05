/**
 * @file trace.c
 * @brief 控制回路跟踪: 环形缓冲采集 + 独立任务回灌串口
 *
 * @see trace.h (接口说明)
 *
 * @author ESP32 平衡轮腿机器人
 */

#include "trace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_heap_caps.h"
#include "esp_partition.h"

#include <stdio.h>
#include <string.h>

/* ========== Flash 记录 (脱机留底) ==========
 * PSRAM 里留最近 FLOG_N 条 (40 秒), 离开平衡态时整块写进 "storage" 分区,
 * 断电不丢, 之后用 scripts/read_flash_log.py 读出。
 *
 * 分区布局: [0, 4KB) 头 (flog_hdr_t), [4KB, ...) 连续的 trace_sample_t 数组。
 * 头最后写: 写到一半断电 → 头无效 → 读出端判定"没有记录"。 */
#define FLOG_N            4000
#define FLOG_SECTOR       4096
#define FLOG_MAGIC        0x474F4C54u     /* "TLOG" */
#define FLOG_PART_NAME    "storage"
/* trace_sample_t.st == 平衡中 (bal_state_t 的 BAL_STATE_BALANCING) */
#define FLOG_ST_BALANCING 2

typedef struct {
    uint32_t magic;       /* FLOG_MAGIC, 写完整块后才有效 */
    uint32_t count;       /* 记录条数 */
    uint32_t rec_size;    /* sizeof(trace_sample_t), 读出端校验用 */
    uint32_t reserved;
} flog_hdr_t;

_Static_assert(sizeof(trace_sample_t) == 56, "读出脚本按 56 字节解析, 改结构体要同步改脚本");

static trace_sample_t *s_flog = NULL;           /* PSRAM 环形缓冲, FLOG_N 条 */
static volatile uint32_t s_flog_total = 0;      /* 累计写入条数 */
static volatile int8_t   s_last_st = 0;         /* 最近一条采样的状态 */

/* ========== 环形缓冲 ==========
 * 600 条 × 10ms = 6 秒历史, 每条 48 字节 → 约 29KB 常驻 RAM */
#define TRACE_RING_N      600

/* 输出任务: 堆 4KB, 优先级 2, 钉在 core 0 (控制任务在 core 1) */
#define TRACE_TASK_STACK  6144
#define TRACE_TASK_PRIO   2
#define TRACE_TASK_CORE   0

/* 实时打印的节奏 (ms): 与控制周期一致 → 100Hz */
#define TRACE_LIVE_PERIOD_MS  10

/* 一轮实时打印最多补多少条 (串口慢时别一次刷一大堆, 免得越堵越堵) */
#define TRACE_LIVE_MAX_BURST  100

/* 冻结采集后等这么久再读, 确保生产者没有"在途"的写
 * (生产者写一条只要几百纳秒, 30ms 绰绰有余)。 */
#define TRACE_FREEZE_WAIT_MS  30

static trace_sample_t s_ring[TRACE_RING_N];

/* s_total  = 累计写入条数 (单调递增, 只增不减)
 * s_count  = 当前有效条数 (最多 TRACE_RING_N)
 * s_printed= 已经实时打印出去的条数 (s_total 的某个前缀)
 * 三者用单调计数表示, 就不会出现"环形绕回来覆盖了还没打印的记录"
 * 那种指针歧义。 */
static volatile uint32_t s_total   = 0;
static volatile uint32_t s_count   = 0;
static volatile uint32_t s_printed = 0;
static volatile bool     s_freeze  = false; /* 回灌期间冻结采集 */

/* 回灌请求信号量 (二值): 控制任务 give, 输出任务 take */
static SemaphoreHandle_t s_dump_sem = NULL;

static SemaphoreHandle_t s_push_lock = NULL;

/**
 * @brief 把一条采样打印成统一的 TRACE 行
 * @param r 采样记录
 * @note  先拼进一个整行再一次性输出 —— 免得 printf 分多次写串口,
 *        跟别的任务的日志交错, 把一行 TRACE 拦腰截断。
 */
static void trace_print_one(const trace_sample_t *r)
{
    char line[160];
    int n = snprintf(line, sizeof(line),
                     "TRACE t=%.3f pitch=%.2f gyro=%.1f out=%.1f tgt=%.2f "
                     "spd=%.1f yaw=%.1f pos=%.1f pos_err=%.2f pos_out=%.1f "
                     "pos_vel=%.2f "
                     "kd=%.2f dt=%.1f st=%d\n",
                     (double)r->t,
                     (double)r->pitch, (double)r->gyro, (double)r->out,
                     (double)r->tgt,  (double)r->spd,   (double)r->yaw,
                     (double)r->pos,  (double)r->pos_err,
                     (double)r->pos_out, (double)r->pos_vel,
                     (double)r->kd,   (double)r->dt,    (int)r->st);
    if (n > 0) {
        fwrite(line, 1, (size_t)n, stdout);
    }
}

/**
 * @brief 取 flash 记录数据区 [off, off+len) 的字节 (按"最老→最新"的记录顺序)
 * @param dst   输出缓冲 (len 字节)
 * @param off   数据区内偏移 (字节)
 * @param len   取多少字节
 * @param first 最老一条的全局序号
 * @param count 记录总条数; 超出数据区的部分填 0xFF
 */
static void flog_fill(uint8_t *dst, uint32_t off, uint32_t len,
                      uint32_t first, uint32_t count)
{
    const uint32_t rec_sz = sizeof(trace_sample_t);
    const uint32_t data_len = count * rec_sz;
    uint32_t k = 0;
    while (k < len) {
        uint32_t pos = off + k;
        if (pos >= data_len) {
            memset(dst + k, 0xFF, len - k);
            return;
        }
        uint32_t in_rec = pos % rec_sz;
        uint32_t n = rec_sz - in_rec;
        if (n > len - k) n = len - k;
        const uint8_t *src =
            (const uint8_t *)&s_flog[(first + pos / rec_sz) % FLOG_N];
        memcpy(dst + k, src + in_rec, n);
        k += n;
    }
}

/**
 * @brief 把 PSRAM 记录缓冲整块写进 storage 分区
 * @return true 写完且头有效; false 没有数据 / 分区不够 / 写入中途重新起立 (已作废)
 * @note  调用时采集已冻结。擦写期间 flash 缓存会暂停, 控制任务会被卡几十毫秒,
 *        所以每个扇区写前检查一次状态, 发现重新进入平衡就放弃。
 */
static bool flog_save(void)
{
    if (s_flog == NULL) return false;

    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, FLOG_PART_NAME);
    if (p == NULL) return false;

    uint32_t total = s_flog_total;
    uint32_t count = total < FLOG_N ? total : FLOG_N;
    if (count == 0) return false;
    uint32_t first = total - count;

    uint32_t data_bytes = count * (uint32_t)sizeof(trace_sample_t);
    uint32_t data_sectors = (data_bytes + FLOG_SECTOR - 1) / FLOG_SECTOR;
    if ((1 + data_sectors) * FLOG_SECTOR > p->size) return false;

    uint8_t *buf = heap_caps_malloc(FLOG_SECTOR, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (buf == NULL) return false;

    bool ok = (esp_partition_erase_range(p, 0, FLOG_SECTOR) == ESP_OK);   /* 先作废旧头 */
    for (uint32_t s = 0; ok && s < data_sectors; s++) {
        if (s_last_st == FLOG_ST_BALANCING) { ok = false; break; }
        uint32_t addr = (1 + s) * FLOG_SECTOR;
        flog_fill(buf, s * FLOG_SECTOR, FLOG_SECTOR, first, count);
        ok = (esp_partition_erase_range(p, addr, FLOG_SECTOR) == ESP_OK) &&
             (esp_partition_write(p, addr, buf, FLOG_SECTOR) == ESP_OK);
        vTaskDelay(1);
    }
    if (ok) {
        flog_hdr_t hdr = { FLOG_MAGIC, count, (uint32_t)sizeof(trace_sample_t), 0 };
        ok = (esp_partition_write(p, 0, &hdr, sizeof(hdr)) == ESP_OK);
    }
    free(buf);
    return ok;
}

/**
 * @brief 输出任务: 实时打印新采样 + 响应回灌请求
 * @param arg 未用
 */
static void trace_task(void *arg)
{
    (void)arg;
    TickType_t last_live = xTaskGetTickCount();

    for (;;) {
        /* --- 1) 实时打印还没打印过的采样 --- */
        TickType_t now = xTaskGetTickCount();
        if (now - last_live >= pdMS_TO_TICKS(TRACE_LIVE_PERIOD_MS)) {
            last_live = now;

            /* 串口慢导致打印落后太多时, 落后部分已经被环形缓冲覆盖了,
             * 直接跳过 (那些数据回灌时会从环形缓冲里完整取到)。 */
            uint32_t total = s_total;
            if (total - s_printed > TRACE_RING_N) {
                s_printed = total - TRACE_RING_N;
            }
            uint32_t burst = 0;
            while (s_printed < s_total && burst < TRACE_LIVE_MAX_BURST) {
                trace_print_one(&s_ring[s_printed % TRACE_RING_N]);
                s_printed++;
                burst++;
            }
            fflush(stdout);
        }

        /* --- 2) 等回灌请求 (最多等一个实时周期) --- */
        if (xSemaphoreTake(s_dump_sem, pdMS_TO_TICKS(TRACE_LIVE_PERIOD_MS))
            != pdTRUE) {
            continue;
        }

        /* 冻结采集, 并等生产者把在途的写做完 —— 之后环形缓冲是稳定的,
         * 打印出来的内容不会有"半条记录"这种撕裂。
         * (冻结标记的置位在采集锁里做, 跟生产者的写互斥) */
        if (s_push_lock != NULL) {
            xSemaphoreTake(s_push_lock, portMAX_DELAY);
            s_freeze = true;
            xSemaphoreGive(s_push_lock);
        } else {
            s_freeze = true;
        }
        vTaskDelay(pdMS_TO_TICKS(TRACE_FREEZE_WAIT_MS));

        /* 先落 flash (权威备份), 再打串口 */
        bool saved = flog_save();
        printf("FLOG %s (%u 条)\n", saved ? "saved" : "skipped",
               (unsigned)(s_flog_total < FLOG_N ? s_flog_total : FLOG_N));

        uint32_t total = s_total;
        uint32_t count = s_count;
        uint32_t first = total - count;   /* 最老一条的全局序号 */

        printf("=== TRACE DUMP BEGIN (%u 条) ===\n", (unsigned)count);
        for (uint32_t i = first; i < total; i++) {
            trace_print_one(&s_ring[i % TRACE_RING_N]);
        }
        printf("=== TRACE DUMP END ===\n");
        fflush(stdout);

        /* 回灌里的记录已经打过, 别再实时打一遍 */
        if (s_printed < total) s_printed = total;
        if (s_push_lock != NULL) {
            xSemaphoreTake(s_push_lock, portMAX_DELAY);
            s_freeze = false;
            xSemaphoreGive(s_push_lock);
        } else {
            s_freeze = false;
        }
    }
}

/* ================= 对外 API ================= */

void trace_init(void)
{
    if (s_dump_sem != NULL) return;   /* 已经初始化过了 */

    /* stdout 默认是全缓冲的, 光靠 fwrite 不一定马上发出去。
     * 靠 ESP_LOG 组件每次都 flush 的副作用带着走 —— 我们自己的 printf
     * 后面也显式 fflush, 不依赖这个副作用。 */
    s_dump_sem = xSemaphoreCreateBinary();
    if (s_dump_sem == NULL) return;
    s_push_lock = xSemaphoreCreateBinary();
    if (s_push_lock == NULL) return;
    xSemaphoreGive(s_push_lock);   /* 二值信号量初始为空 = 满 */

    s_total   = 0;
    s_count   = 0;
    s_printed = 0;
    s_freeze  = false;

    /* PSRAM 申请失败就只关掉 flash 记录, 串口跟踪照常 */
    s_flog = heap_caps_malloc(FLOG_N * sizeof(trace_sample_t), MALLOC_CAP_SPIRAM);
    s_flog_total = 0;

    xTaskCreatePinnedToCore(trace_task, "trace", TRACE_TASK_STACK,
                            NULL, TRACE_TASK_PRIO, NULL, TRACE_TASK_CORE);
}

void trace_push(const trace_sample_t *s)
{
    if (s == NULL) return;
    s_last_st = s->st;      /* 冻结期间也要更新, flog_save 靠它判断是否重新起立 */
    /* 唯一生产者 = 控制任务, 所以环形缓冲本身不需要加锁。
     * 但"检查冻结标记"和"写数据"之间不能被打断 (见 s_push_lock 的注释):
     * 回灌读的正是生产者下一步要覆盖的那一槽, 撕裂了就白抓了。 */
    if (s_push_lock != NULL) {
        if (xSemaphoreTake(s_push_lock, portMAX_DELAY) != pdTRUE) return;
    }
    if (s_freeze) {
        if (s_push_lock != NULL) xSemaphoreGive(s_push_lock);
        return;
    }

    uint32_t idx = s_total % TRACE_RING_N;
    s_ring[idx] = *s;
    s_total++;
    if (s_count < TRACE_RING_N) s_count++;
    if (s_flog != NULL) {
        s_flog[s_flog_total % FLOG_N] = *s;
        s_flog_total++;
    }

    if (s_push_lock != NULL) xSemaphoreGive(s_push_lock);
}

void trace_request_dump(void)
{
    if (s_dump_sem == NULL) return;
    /* 二值信号量: 已经有未处理的请求时这次 give 不会加计数,
     * 所以连续触发不会堆积。 */
    xSemaphoreGive(s_dump_sem);
}
