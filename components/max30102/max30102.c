/**
 * @file    max30102.c
 * @brief   MAX30102 心率/血氧驱动（ESP-IDF 硬件 I2C + GPIO 中断）
 */

#include "max30102.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"

#include "bsp_i2c.h"

static const char *TAG = "max30102";

/* ============================== Kconfig 兜底 ==============================
 * 万一组件 Kconfig 没被 sdkconfig 收进去，宏就是空的，这里给默认值保证能编译。
 */
#ifndef CONFIG_MAX30102_SAMPLE_TASK_STACK
#define CONFIG_MAX30102_SAMPLE_TASK_STACK       3072
#endif
#ifndef CONFIG_MAX30102_SAMPLE_TASK_PRIO
#define CONFIG_MAX30102_SAMPLE_TASK_PRIO        15
#endif
#ifndef CONFIG_MAX30102_SAMPLE_TASK_CORE
#define CONFIG_MAX30102_SAMPLE_TASK_CORE        1
#endif
#ifndef CONFIG_MAX30102_HR_TASK_STACK
#define CONFIG_MAX30102_HR_TASK_STACK           6144
#endif
#ifndef CONFIG_MAX30102_HR_TASK_PRIO
#define CONFIG_MAX30102_HR_TASK_PRIO            13
#endif
#ifndef CONFIG_MAX30102_HR_TASK_CORE
#define CONFIG_MAX30102_HR_TASK_CORE            1
#endif


SemaphoreHandle_t xMax30102Semaphore  = NULL;   /* INT 触发 → 采样任务 */
SemaphoreHandle_t xSemaphore_HR_Ready = NULL;   /* 窗口满 → 算法任务 */

/* 双缓冲：采样任务写 s_bank，写满后切成另一块，算法任务读刚写满的那块。
 * 原工程只有一组 200 点缓冲，采满就 xSemaphoreGive 然后立刻从 0 开始覆盖，
 * 算法任务那边还在读 → 会被踩。ESP32-S3 有 512KB SRAM，多 3.2KB 换掉这个竞态很划算。 */
static uint32_t s_red_buf[2][MAX30102_WINDOW];
static uint32_t s_ir_buf[2][MAX30102_WINDOW];

static volatile int s_bank       = 0;   /* 正在写的 bank */
static volatile int s_ready_bank = 0;   /* 刚写满、等算法取的 bank */
static volatile int s_cnt        = 0;   /* 当前 bank 里的样本数 */
static volatile uint32_t s_last_red = 0;  /* 诊断用：最近读到的红光/红外原始值 */
static volatile uint32_t s_last_ir  = 0;

static volatile uint8_t s_heart      = 0;
static volatile uint8_t s_spo2       = 0;
static volatile uint8_t s_hr_valid   = 0;
static volatile uint8_t s_spo2_valid = 0;

static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static void (*s_hb_cb)(void) = NULL;

/* 器件在线标志，与 MPU6050 同理：线松了任务还在转，只有这个标志能区分
 * "任务卡死"和"器件掉了"。掉线后继续给看门狗打卡（掉线是可恢复故障，
 * 复位治不好，只会无限重启），由上层降级显示与上报。 */
static volatile bool s_online = true;

bool max30102_is_online(void)
{
    return s_online;
}

static void set_online(bool on, const char *why)
{
    if (s_online == on) {
        return;
    }
    s_online = on;
    if (on) {
        ESP_LOGI(TAG, "[MAX] 恢复在线");
    } else {
        ESP_LOGE(TAG, "[MAX] 离线：%s（检查 VCC/GND/SDA/SCL 与 INT 线）",
                 why != NULL ? why : "无响应");
    }
}
static bool s_isr_added  = false;
static bool s_tasks_up   = false;
static bool s_inited     = false;

/* 算法任务的工作副本（放静态区，不占栈） */
static uint32_t s_hr_ir[MAX30102_WINDOW];
static uint32_t s_hr_red[MAX30102_WINDOW];

/* ============================== 寄存器读写 ==============================
 * 原工程这里有三层：max30102_Bus_Write → 位翻转；现在只剩一行。
 * 返回值语义统一成 esp_err_t，但兼容层保留 1/0 的旧语义。
 */

esp_err_t max30102_write_reg(uint8_t reg, uint8_t val)
{
    return bsp_i2c_write_reg(MAX30102_I2C_ADDR, reg, val);
}

esp_err_t max30102_read_reg(uint8_t reg, uint8_t *val)
{
    if (val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return bsp_i2c_read_reg(MAX30102_I2C_ADDR, reg, val, 1);
}

esp_err_t max30102_read_part_id(uint8_t *id)
{
    return max30102_read_reg(REG_PART_ID, id);
}


/**
 * @brief GPIO 中断服务程序。，
 * 这里只做"给信号量"一件事：ISR 里不做 I2C（会阻塞）、不打日志。
 */
static void IRAM_ATTR max30102_int_isr(void *arg)
{
    (void)arg;
    BaseType_t hp = pdFALSE;

    if (xMax30102Semaphore != NULL) {
        xSemaphoreGiveFromISR(xMax30102Semaphore, &hp);
    }
    if (hp == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief 把 INT 脚挂到 GPIO 中断服务上。
 *
 * 和 STM32 的差别：STM32 是 HAL_NVIC_EnableIRQ(EXTI1_IRQn) 直接开；
 * ESP-IDF 需要先 gpio_install_isr_service() 装一个全局分发器（整个工程只能装一次），
 * 再 gpio_isr_handler_add() 挂自己的处理函数。
 */
static esp_err_t int_pin_install(void)
{
    if (s_isr_added) {
        return ESP_OK;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << MAX30102_INT_GPIO_IO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,   /* MAX30102 INT 低有效，靠上拉保证空闲为高 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config 失败");

    /* 其它组件（按键、编码器…）可能已经装过分发器 → ESP_ERR_INVALID_STATE 视为正常 */
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(MAX30102_INT_GPIO_IO, max30102_int_isr, NULL),
                        TAG, "gpio_isr_handler_add 失败");

    s_isr_added = true;
    return ESP_OK;
}

/** 读两个中断状态寄存器把 INT 拉回高电平（不读的话中断只会来一次） */
static esp_err_t int_clear(void)
{
    uint8_t dummy;
    esp_err_t err = max30102_read_reg(REG_INTR_STATUS_1, &dummy);
    if (err == ESP_OK) {
        err = max30102_read_reg(REG_INTR_STATUS_2, &dummy);
    }
    return err;
}


/** 读 FIFO 里的一个样本（3 字节红光 + 3 字节红外，各 18 位） */
static esp_err_t fifo_pop_one(uint32_t *red, uint32_t *ir)
{
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(MAX30102_I2C_ADDR, REG_FIFO_DATA, b, sizeof(b)),
                        TAG, "读 FIFO 失败");

    *red = ((uint32_t)(b[0] & 0x03) << 16) | ((uint32_t)b[1] << 8) | b[2];
    *ir  = ((uint32_t)(b[3] & 0x03) << 16) | ((uint32_t)b[4] << 8) | b[5];

    s_last_red = *red;
    s_last_ir  = *ir;
    return ESP_OK;
}

/* 轮询周期与每轮样本数。芯片固定 100 sps，所以"每 N ms 取 N/10 个样本"正好 1:1 对齐。
 * 10ms → 1 个（旧行为，100 次事务/秒）；20ms → 2 个（事务数减半）。
 * 两种周期下 FIFO 都不会溢出：20ms 只攒 2 个，深度 32 有 320ms 余量。 */
#define MAX30102_POLL_MS          (CONFIG_MAX30102_POLL_MS)
#define MAX30102_SAMPLES_PER_POLL ((MAX30102_POLL_MS) / 10)

static void window_push(uint32_t red, uint32_t ir);

/** 一次事务连读 k 个样本。
 *
 * 批量读的由来：芯片固定 100 sps，而每次读一个样本都要走一遍
 * "START + 器件地址 + 寄存器地址 + Sr + 器件地址 + 6 字节 + STOP"，
 * 100 次/秒就是 100 次总线事务、100 次抢互斥量。改成每 20ms 一次读 2 个样本，
 * 事务数直接砍一半，FIFO 深度 32 也完全撑得住（32 个样本 = 320ms 余量）。
 *
 * @param want 想读几个（1~4）
 * @param got  实际有效样本数（全 0 的填充值会被丢弃，不计入）
 */
static esp_err_t fifo_pop_batch(int want, int *got)
{
    uint8_t b[MAX30102_FIFO_DEPTH * 6 > 24 ? 24 : MAX30102_FIFO_DEPTH * 6];
    int k = (want < 1) ? 1 : (want > 4 ? 4 : want);

    *got = 0;

    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(MAX30102_I2C_ADDR, REG_FIFO_DATA, b, (size_t)(k * 6)),
                        TAG, "批量读 FIFO 失败");

    for (int i = 0; i < k; i++) {
        const uint8_t *p = &b[i * 6];
        uint32_t red = ((uint32_t)(p[0] & 0x03) << 16) | ((uint32_t)p[1] << 8) | p[2];
        uint32_t ir  = ((uint32_t)(p[3] & 0x03) << 16) | ((uint32_t)p[4] << 8) | p[5];

        /* FIFO 空的时候芯片回填充值，基本是全 0，丢掉 */
        if (red == 0 && ir == 0) {
            continue;
        }
        s_last_red = red;
        s_last_ir  = ir;
        window_push(red, ir);
        (*got)++;
    }
    return ESP_OK;
}

/** 把一个样本压进当前 bank；写满则切换 bank 并通知算法任务 */
static void window_push(uint32_t red, uint32_t ir)
{
    int i = s_cnt;
    if (i >= MAX30102_WINDOW) {
        return;
    }

    s_red_buf[s_bank][i] = red;
    s_ir_buf[s_bank][i]  = ir;
    s_cnt = i + 1;

    if (s_cnt >= MAX30102_WINDOW) {
        /* 顺序很关键：先切 bank 再给信号量。
         * 反过来写的话，算法任务可能在本任务切换之前就抢到信号量，
         * 读到 s_bank^1 指向的、马上要被覆盖的那块。 */
        s_ready_bank = s_bank;
        s_bank       = s_bank ^ 1;
        s_cnt        = 0;
        if (xSemaphore_HR_Ready != NULL) {
            xSemaphoreGive(xSemaphore_HR_Ready);
        }
    }
}

/**
 * @brief 把 FIFO 抽空。
 *
 * 逐样本中断读的问题：
 * 原工程每次中断只读 1 个样本，而 FIFO_CONFIG=0x0F 意味着 FIFO 攒到 17 个才触发一次中断，
 * 剩下的样本堆在 FIFO 里，攒满 32 个后就被丢弃（OVF）。改成按读写指针抽空，
 * 100Hz 的采样率才真正守住。
 */
/* 指针不可用的芯片走盲读模式（实测这颗模块 WR/RD 恒读 0，但 OVF 会涨），根据你自己的芯片特性调整 */
static bool     s_blind_mode    = false;
static uint32_t s_total_samples = 0;

static esp_err_t fifo_drain(void)
{
    uint8_t wr = 0, rd = 0, ovf = 0;
    esp_err_t err;

    err = max30102_read_reg(REG_FIFO_WR_PTR, &wr);
    if (err != ESP_OK) {
        return err;
    }
    err = max30102_read_reg(REG_FIFO_RD_PTR, &rd);
    if (err != ESP_OK) {
        return err;
    }
    (void)max30102_read_reg(REG_OVF_COUNTER, &ovf);

    wr &= 0x1F;
    rd &= 0x1F;

    /* 实测结论，2026-10-28：这颗模块的 WR_PTR(0x04) / RD_PTR(0x06) 恒读回 0，
     * 但 OVF_COUNTER(0x05) 会一路涨到 31。溢出只可能由"芯片写入了样本"造成 ——
     * 说明数据其实一直在产生，只是指针寄存器不更新（兼容片常见毛病）。
     * 按指针判断就会永远认为 FIFO 是空的，一个样本都读不出来。
     * 于是：一旦"溢出过 + 至今没读到任何样本"成立，退化成盲读 ——
     * 每次轮询读一批样本，与芯片 100Hz 产出保持 1:1（见 MAX30102_POLL_MS）。 */
    if (!s_blind_mode && ovf != 0 && s_total_samples == 0) {
        s_blind_mode = true;
        ESP_LOGW(TAG, "[MAX] WR/RD 恒为 0 但溢出计数=%u → 切盲读模式（每 %d ms 读 %d 个样本）",
                 ovf, MAX30102_POLL_MS, MAX30102_SAMPLES_PER_POLL);
    }

    int n = 0;
    if (wr != rd) {
        while (rd != wr && n < MAX30102_FIFO_DEPTH) {
            uint32_t red, ir;
            err = fifo_pop_one(&red, &ir);
            if (err != ESP_OK) {
                return err;
            }
            window_push(red, ir);
            rd = (rd + 1) & 0x1F;
            n++;
        }
        /* 指针本身能正常抽出样本，说明这颗芯片是好的 → 关掉盲读，回到标准路径 */
        if (n > 0 && s_blind_mode) {
            s_blind_mode = false;
            ESP_LOGI(TAG, "[MAX] 指针已可正常使用，退出盲读模式");
        }
    } else if (s_blind_mode) {
        /* 盲读：一次事务读 SAMPLES_PER_POLL 个样本，与轮询周期对齐。
         * 这样即使指针不可用，也不至于每 10ms 抢一次总线。 */
        int got = 0;
        err = fifo_pop_batch(MAX30102_SAMPLES_PER_POLL, &got);
        if (err != ESP_OK) {
            return err;
        }
        n = got;
    }
    s_total_samples += (uint32_t)n;

    if (ovf != 0) {
        ESP_LOGD(TAG, "FIFO 溢出计数=%u（CPU 忙于其它事务，未丢进算法窗口）", ovf);
    }
    return ESP_OK;
}

/** 清 FIFO：把三个指针归零 */
static esp_err_t fifo_clear(void)
{
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_FIFO_WR_PTR, 0x00), TAG, "清 WR_PTR 失败");
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_OVF_COUNTER, 0x00), TAG, "清 OVF 失败");
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_FIFO_RD_PTR, 0x00), TAG, "清 RD_PTR 失败");
    return ESP_OK;
}


esp_err_t max30102_reset(void)
{
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_MODE_CONFIG, MODE_RESET_BIT),
                        TAG, "写 RESET 位失败");

    /* 原工程：Delay_us(20000) 死等 20ms。
     * ESP-IDF 里必须让出 CPU，否则同核的 WiFi/MQTT 任务会被饿 20ms。 */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 轮询 RESET 位自清，最多等 100ms（数据手册典型 10ms 以内） */
    for (int i = 0; i < 10; i++) {
        uint8_t v = 0;
        if (max30102_read_reg(REG_MODE_CONFIG, &v) == ESP_OK && (v & MODE_RESET_BIT) == 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "RESET 位 100ms 内未自清（器件可能存在，但响应慢）");
    return ESP_OK;   /* 不当作致命错误：后面的 ID 校验才是判据 */
}

esp_err_t max30102_init(void)
{
    uint8_t id = 0;

    /* 1) 总线先起来（幂等，重复调用无副作用） */
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "I2C 总线初始化失败");

    /* 2) 软复位 */
    ESP_RETURN_ON_ERROR(max30102_reset(), TAG, "软复位失败");

    /* 3) 校验器件在线：PART_ID 固定 0x15 */
    if (max30102_read_part_id(&id) == ESP_OK && id != MAX30102_PART_ID_VAL) {
        ESP_LOGE(TAG, "PART_ID=0x%02X，期望 0x15（检查接线/地址 0x57/供电）", id);
        return ESP_ERR_NOT_FOUND;
    }

    /* 4) 配置（数值与原工程完全一致，勿随意改：改 SPO2_CONFIG 会同时改变采样率与脉宽） */
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_INTR_ENABLE_1, 0x40), TAG, "配置 INTR_EN_1 失败");
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_INTR_ENABLE_2, 0x00), TAG, "配置 INTR_EN_2 失败");
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_FIFO_CONFIG,  0x0F), TAG, "配置 FIFO 失败");
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_SPO2_CONFIG,  0x27), TAG, "配置 SPO2 失败"); // 先配
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_MODE_CONFIG,  MODE_SPO2), TAG, "配置 MODE 失败"); // 后启
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_LED1_PA,      0x24), TAG, "配置 LED1 失败");
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_LED2_PA,      0x24), TAG, "配置 LED2 失败");
    ESP_RETURN_ON_ERROR(max30102_write_reg(REG_PILOT_PA,     0x7F), TAG, "配置 PILOT 失败");

    /* 5) 清 FIFO + 清中断标志 */
    ESP_RETURN_ON_ERROR(fifo_clear(), TAG, "清 FIFO 失败");
    (void)int_clear();

    /* 6) 信号量必须先于中断创建，否则第一次中断会落入 NULL 判断 */
    if (xMax30102Semaphore == NULL) {
        xMax30102Semaphore = xSemaphoreCreateBinary();
    }
    if (xSemaphore_HR_Ready == NULL) {
        xSemaphore_HR_Ready = xSemaphoreCreateBinary();
    }
    if (xMax30102Semaphore == NULL || xSemaphore_HR_Ready == NULL) {
        ESP_LOGE(TAG, "信号量创建失败（堆不足）");
        return ESP_ERR_NO_MEM;
    }

    /* 7) 挂 INT 中断 */
    ESP_RETURN_ON_ERROR(int_pin_install(), TAG, "INT 引脚初始化失败");

    s_cnt        = 0;
    s_bank       = 0;
    s_ready_bank = 0;
    s_inited     = true;

    ESP_LOGI(TAG, "初始化完成：PART_ID=0x%02X，SpO2 模式，窗口 %d 点",
             id, MAX30102_WINDOW);
    return ESP_OK;
}


esp_err_t max30102_recover_bus(void)
{
    uint8_t id = 0;

    ESP_LOGW(TAG, "开始恢复 MAX30102");

    /* 第一层：纯总线级恢复（9 个脉冲 + STOP + 重装驱动）。
     * 原工程这段是自己敲 GPIO；现在 bsp_i2c_bus_reset() 已经把三个 I2C 器件共用，
     * 所以恢复一次对 OLED/MPU6050 也是有效的，这正是一条总线的额外好处。 */
    ESP_RETURN_ON_ERROR(bsp_i2c_bus_reset(), TAG, "总线恢复失败");

    /* 第二层：总线通了但器件还在复位态 → 重配寄存器 */
    if (max30102_read_part_id(&id) == ESP_OK && id == MAX30102_PART_ID_VAL) {
        (void)int_clear();
        s_cnt = 0;
        s_bank = 0;
        s_ready_bank = 0;
        ESP_LOGI(TAG, "恢复成功（ID=0x%02X）", id);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "器件无应答（ID=0x%02X），尝试软复位重配", id);
    esp_err_t err = max30102_init();
    if (err == ESP_OK && max30102_read_part_id(&id) == ESP_OK && id == MAX30102_PART_ID_VAL) {
        ESP_LOGI(TAG, "重新初始化后恢复成功");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "恢复失败，可能已死锁，需断电重启");
    return ESP_FAIL;
}


void max30102_get_heart(uint8_t *heart, uint8_t *spo2,
                        uint8_t *hr_valid, uint8_t *spo2_valid)
{
    /* 原工程用 taskENTER_CRITICAL()；ESP-IDF 里这个宏要么不存在要么绑全局自旋锁，
     * 正确写法是自带一个 portMUX_TYPE，作用域只有这 4 个字节的拷贝。 */
    portENTER_CRITICAL(&s_state_mux);
    if (heart)      { *heart      = s_heart; }
    if (spo2)       { *spo2       = s_spo2; }
    if (hr_valid)   { *hr_valid   = s_hr_valid; }
    if (spo2_valid) { *spo2_valid = s_spo2_valid; }
    portEXIT_CRITICAL(&s_state_mux);
}

void max30102_set_heartbeat_cb(void (*cb)(void))
{
    s_hb_cb = cb;
}


void vMAX30102_Task(void *arg)
{
    (void)arg;

    uint8_t id = 0;
    if (max30102_read_part_id(&id) == ESP_OK && id == MAX30102_PART_ID_VAL) {
        ESP_LOGI(TAG, "[MAX] 启动自检通过 ID=0x%02X", id);
    } else {
        ESP_LOGE(TAG, "[MAX] 启动自检失败 ID=0x%02X", id);
    }

    int      io_fail  = 0;
    uint32_t last_rec = 0;

    /* 体检统计：用于定位"没有数"的原因 */
    uint32_t poll_cnt   = 0;
    uint32_t int_cnt    = 0;
    uint32_t last_diag  = 0;
    uint8_t  last_wr    = 0xFF;
    bool     wr_moving  = false;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        bool io_ok = true;

        poll_cnt++;
        /* 只做计数、不阻塞等待：采样节奏由循环末尾的 vTaskDelayUntil 固定成 10ms，
         * 与芯片 100Hz 的产出速率对齐。盲读模式下必须如此，靠中断驱动
         * 会导致读得比写得慢，FIFO 持续溢出。 */
        if (xSemaphoreTake(xMax30102Semaphore, 0) == pdTRUE) {
            int_cnt++;
        }
        /* 注意：这里不再 continue。超时也照样往下读 ——
         * INT 线没接、或者中断边沿被错过时，靠 FIFO 写/读指针轮询一样能出数，
         * 只是实时性略差。原来"没中断就跳过"的做法会让整条链路静默死掉。 */

        /* 先读中断状态，否则 INT 脚一直是低的 */
        if (int_clear() != ESP_OK) {
            io_fail++;
            io_ok = false;
        }

        if (fifo_drain() != ESP_OK) {
            io_fail++;
            io_ok = false;
            if (io_fail >= 5) {
                set_online(false, "连续 5 次 I2C 读失败");
                if ((uint32_t)(esp_log_timestamp() - last_rec) > 5000) {
                    last_rec = esp_log_timestamp();
                    io_fail  = 0;
                    ESP_LOGW(TAG, "[MAX] 连续 5 次 I2C 失败，触发恢复");
                    (void)max30102_recover_bus();
                }
            }
        } else {
            io_fail = 0;
            set_online(true, NULL);
        }

        /* 每 5 秒体检一次，把"没数"的原因直接说清楚： */
        uint32_t now_ms = esp_log_timestamp();
        if (now_ms - last_diag >= 5000) {
            last_diag = now_ms;

            uint8_t wr = 0, rd = 0, ovf = 0, mode = 0, spo2cfg = 0;
            (void)max30102_read_reg(REG_FIFO_WR_PTR,  &wr);
            (void)max30102_read_reg(REG_FIFO_RD_PTR,  &rd);
            (void)max30102_read_reg(REG_OVF_COUNTER,  &ovf);
            (void)max30102_read_reg(REG_MODE_CONFIG,  &mode);
            (void)max30102_read_reg(REG_SPO2_CONFIG,  &spo2cfg);

            if (wr != last_wr) {
                if (last_wr != 0xFF) {   /* 第一次只记录，不算"在动" */
                    wr_moving = true;
                }
                last_wr = wr;
            }

            /* 下面这一整段是"体检报告"，只在调这颗芯片时才需要看。
             * 默认关掉（menuconfig → Max30102 → 打印周期诊断），
             * 否则每 5 秒刷 3 行，把 OTA / MQTT / 看门狗的日志全盖住。
             * 注意：判"在线/离线"的逻辑不在这里，关掉不影响掉线检测。 */
#if CONFIG_MAX30102_DEBUG_DIAG
            ESP_LOGI(TAG, "[MAX] 5s: 中断=%lu | WR=%u RD=%u OVF=%u | MODE=0x%02X SPO2=0x%02X "
                          "| 窗口=%d/%d | 末样本 red=%lu ir=%lu",
                     (unsigned long)int_cnt, wr, rd, ovf, mode, spo2cfg,
                     s_cnt, MAX30102_WINDOW,
                     (unsigned long)s_last_red, (unsigned long)s_last_ir);

            /* 温度自检：温度转换不经过 LED 通路，是判断"芯片到底在不在运转"的干净探针。
             * 温度能出数 ⇒ 芯片活着，问题在 LED/光路/供电电压；温度也不动 ⇒ 芯片没在转换。 */
            uint8_t tint = 0, tfrac = 0;
            (void)max30102_write_reg(REG_TEMP_CONFIG, 0x01);
            vTaskDelay(pdMS_TO_TICKS(35));
            (void)max30102_read_reg(REG_TEMP_INTR, &tint);
            (void)max30102_read_reg(REG_TEMP_FRAC, &tfrac);
            float tc = (float)((int8_t)tint) + (float)(tfrac & 0x0F) * 0.0625f;
            ESP_LOGI(TAG, "[MAX] 片内温度=%.4f℃（0.0000 说明芯片没在转换）", tc);

            /* 整块寄存器快照：只连读 0x00~0x06。
             * 千万不要读到 0x07(FIFO_DATA)，连读会顺手把 FIFO 里的样本吃掉，
             * 每 5 秒偷走一个样本，还会让后面的 CONFIG/MODE 显示成数据字节。 */
            uint8_t regs[7] = {0};
            if (bsp_i2c_read_reg(MAX30102_I2C_ADDR, 0x00, regs, sizeof(regs)) == ESP_OK) {
                ESP_LOGI(TAG, "[MAX] 寄存器 00~06: "
                              "INTR1=%02X INTR2=%02X EN1=%02X EN2=%02X WR=%02X OVF=%02X RD=%02X",
                         regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6]);
            } else {
                ESP_LOGW(TAG, "[MAX] 整块读寄存器 00~06 失败");
            }
#endif

            /* 写指针不动 ≠ 没数据。
             * 这颗兼容片的 WR/RD 会长期读 0（见盲读模式），但样本其实一直在流。
             * 判据改成"这 5 秒里样本总数有没有涨"，涨了就闭嘴。 */
            static uint32_t last_total = 0;
            const bool samples_flowing = (s_total_samples != last_total);
            last_total = s_total_samples;

            /* 直接给出结论，省得对着数字猜 */
            if (!wr_moving && int_cnt == 0 && !s_blind_mode && !samples_flowing) {
                ESP_LOGE(TAG, "[MAX] 芯片没在采样且没有中断：查供电(3V3)、I2C 上拉、MODE 是否被复位");
                set_online(false, "FIFO 不涨且无中断，芯片没在采样");
            } else if (!wr_moving && !samples_flowing) {
                ESP_LOGE(TAG, "[MAX] FIFO 写指针不动：MODE=0x%02X（应为 0x03）。"
                              "亮度/模式可能没写进去，或芯片处于 SHDN", mode);
            } else if (!wr_moving) {
                /* 指针是死的、数据是活的，兼容片常见，不必报警 */
                ESP_LOGD(TAG, "[MAX] 写指针恒为 0，但样本在正常流入（兼容片特性）");
            } else if (int_cnt == 0) {
                ESP_LOGW(TAG, "[MAX] FIFO 在走但没有中断：INT 线没接到 GPIO%d（不影响出数）",
                         MAX30102_INT_GPIO_IO);
            } else if (s_cnt == 0 && s_last_ir == 0) {
                ESP_LOGE(TAG, "[MAX] 读出全是 0：LED 没亮，查 LED1_PA/LED2_PA 与供电");
            }

            poll_cnt = 0;
            int_cnt  = 0;
        }

        /* 打卡规则同 MPU6050：通了就打卡；判了离线也打卡（任务活着、器件没了）；
         * 只有"连败但还没判离线"那几拍不打卡，让看门狗有机会发现真正的阻塞。 */
        if (io_ok || !s_online) {
            if (s_hb_cb) {
                s_hb_cb();
            }
        }

        /* 固定 10ms 节奏（= 100Hz），与芯片采样率一一对应 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MAX30102_POLL_MS));
    }
}

void vHR_Algorithm_Task(void *arg)
{
    (void)arg;

    int32_t spo2 = 0, hr = 0;
    int8_t  spo2_valid = 0, hr_valid = 0;

    for (;;) {
        if (s_hb_cb) {
            s_hb_cb();
        }

        if (xSemaphoreTake(xSemaphore_HR_Ready, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 拷快照：拷完这两个数组，采样任务再去写另一块 bank，两边彻底解耦 */
        int b = s_ready_bank;
        memcpy(s_hr_ir,  s_ir_buf[b],  sizeof(s_hr_ir));
        memcpy(s_hr_red, s_red_buf[b], sizeof(s_hr_red));

        maxim_heart_rate_and_oxygen_saturation(s_hr_ir, MAX30102_WINDOW, s_hr_red,
                                               &spo2, &spo2_valid, &hr, &hr_valid);

        if (hr_valid && spo2_valid && hr < 200 && spo2 < 200) {
            portENTER_CRITICAL(&s_state_mux);
            s_heart      = (hr < 0) ? 0 : (uint8_t)hr;
            s_spo2       = (uint8_t)(spo2 - 1);
            s_hr_valid   = 1;
            s_spo2_valid = 1;
            portEXIT_CRITICAL(&s_state_mux);

            ESP_LOGI(TAG, "[MAX] HR=%d bpm  SpO2=%d%%", (int)hr, (int)spo2);
        } else {
            /* 窗口无效是常态（手指没放好、动了、信号太弱），不该用 INFO 刷屏。
             * 改成 D 级：默认日志级别下不打印，需要时把 log level 调到 Debug 就能看到。 */
            ESP_LOGD(TAG, "[MAX] 本窗口无效 hr_valid=%d spo2_valid=%d", hr_valid, spo2_valid);
        }
    }
}

esp_err_t max30102_start_tasks(void)
{
    if (s_tasks_up) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(vMAX30102_Task, "max30102",
                                             CONFIG_MAX30102_SAMPLE_TASK_STACK,
                                             NULL, CONFIG_MAX30102_SAMPLE_TASK_PRIO,
                                             NULL, CONFIG_MAX30102_SAMPLE_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "采样任务创建失败");
        return ESP_ERR_NO_MEM;
    }

    ret = xTaskCreatePinnedToCore(vHR_Algorithm_Task, "hr_algo",
                                  CONFIG_MAX30102_HR_TASK_STACK,
                                  NULL, CONFIG_MAX30102_HR_TASK_PRIO,
                                  NULL, CONFIG_MAX30102_HR_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "算法任务创建失败");
        return ESP_ERR_NO_MEM;
    }

    s_tasks_up = true;
    return ESP_OK;
}

/* ============================== 算法实现由 hr_algo 组件提供 ==============================
 * 之前这里放了一个 __attribute__((weak)) 的桩，只为了让工程在算法还没移植时能链接通过。
 * 现在 components/hr_algo/ 已经有了强符号实现，桩已删除，这是有意的：
 * 弱符号 + 归档库的组合有个坑，如果 hr_algo 的成员没被 ld 拉进来，
 * 弱桩会悄悄顶替，表现就是"心率永远出不来"却没有任何报错。
 * 删掉桩之后，如果哪天 hr_algo 被移除，链接期就会直接报 undefined reference，
 * 比运行期静默失效好得多。
 */


void MAX30102_Init(void)
{
    (void)max30102_init();
}

int8_t MAX30102_RecoverBus(void)
{
    return (max30102_recover_bus() == ESP_OK) ? 0 : 1;
}

void MAX30102_GetResult(uint8_t *heart, uint8_t *spo2)
{
    max30102_get_heart(heart, spo2, NULL, NULL);
}

void MAX30102_GetData(uint8_t *heart, uint8_t *spo2)
{
    MAX30102_GetResult(heart, spo2);
}

uint8_t max30102_Bus_Write(uint8_t reg, uint8_t data)
{
    return (max30102_write_reg(reg, data) == ESP_OK) ? 1 : 0;
}

uint8_t max30102_Bus_Read(uint8_t reg)
{
    uint8_t v = 0;
    (void)max30102_read_reg(reg, &v);
    return v;
}

void max30102_FIFO_ReadBytes(uint8_t reg, uint8_t *data)
{
    if (data == NULL) {
        return;
    }
    (void)bsp_i2c_read_reg(MAX30102_I2C_ADDR, reg, data, 6);
}

void max30102_FIFO_ReadWords(uint8_t reg, uint16_t data[][2], uint8_t count)
{
    if (data == NULL) {
        return;
    }
    for (uint8_t i = 0; i < count; i++) {
        uint32_t red = 0, ir = 0;
        if (fifo_pop_one(&red, &ir) != ESP_OK) {
            return;
        }
        data[i][0] = (uint16_t)red;
        data[i][1] = (uint16_t)ir;
    }
    (void)reg;
}

void maxim_max30102_write_reg(uint8_t addr, uint8_t data)
{
    (void)max30102_write_reg(addr, data);
}

void maxim_max30102_read_reg(uint8_t addr, uint8_t *data)
{
    if (data) {
        (void)max30102_read_reg(addr, data);
    }
}

void maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
    uint32_t red = 0, ir = 0;
    if (fifo_pop_one(&red, &ir) == ESP_OK) {
        if (pun_red_led) { *pun_red_led = red; }
        if (pun_ir_led)  { *pun_ir_led  = ir;  }
    }
}

void MAX30102_CreateTask(void)
{
    (void)max30102_start_tasks();
}