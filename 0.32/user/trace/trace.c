/**
 * @file trace.c
 * @brief 控制回路跟踪: 环形缓冲采集 + 独立任务回灌串口
 *
 * @see trace.h (接口说明 + 为什么要有这个组件)
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#include "trace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <string.h>

/* ========== 环形缓冲 ==========
 * 600 条 × 10ms = **6 秒**历史。
 * 12Hz 量级的振荡在 6 秒里有 72 个周期, 频谱足够分辨。
 * 每条 48 字节 → 约 29KB 常驻 RAM。 */
#define TRACE_RING_N      600

/* 输出任务: 堆 4KB, 优先级压到最低 (2),
 * 钉在 core 0 —— 控制任务钉在 core 1, 串口阻塞不跟控制环抢 CPU。 */
#define TRACE_TASK_STACK  4096
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

/* 采集锁 (二值信号量): 保证"冻结检查 + 写环形缓冲"是原子的。
 * ⚠ 为什么需要: 环形缓冲的写槽位 s_total % N **恰好就是最老那条所在的槽位**
 *   (因为 N 是整数倍关系), 也就是说回灌正在读的那条, 正是生产者下一步要覆盖的那条。
 *   如果只在"检查 s_freeze"和"写数据"之间被打断一次, 回灌就会读到一条
 *   写了一半的记录 (撕裂)。
 *   用信号量把"检查 + 写"包成临界区就彻底没有这个窗口了。
 *   代价: 控制任务每 10ms 抢一次信号量 (无竞争时 ~100ns), 可以忽略。 */
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
                     "spd=%.1f yaw=%.1f pos=%.1f pos_out=%.1f kd=%.2f dt=%.1f st=%d\n",
                     (double)r->t,
                     (double)r->pitch, (double)r->gyro, (double)r->out,
                     (double)r->tgt,  (double)r->spd,   (double)r->yaw,
                     (double)r->pos,  (double)r->pos_out,
                     (double)r->kd,   (double)r->dt,    (int)r->st);
    if (n > 0) {
        fwrite(line, 1, (size_t)n, stdout);
    }
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

    xTaskCreatePinnedToCore(trace_task, "trace", TRACE_TASK_STACK,
                            NULL, TRACE_TASK_PRIO, NULL, TRACE_TASK_CORE);
}

void trace_push(const trace_sample_t *s)
{
    if (s == NULL) return;
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

    if (s_push_lock != NULL) xSemaphoreGive(s_push_lock);
}

void trace_request_dump(void)
{
    if (s_dump_sem == NULL) return;
    /* 二值信号量: 已经有未处理的请求时这次 give 不会加计数,
     * 所以连续触发不会堆积。 */
    xSemaphoreGive(s_dump_sem);
}
