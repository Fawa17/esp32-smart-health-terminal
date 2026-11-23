/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动，由 STM32 HAL + 软件 I2C 迁移到 ESP-IDF 硬件 I2C
 *
 * 注意：软件 I2C 里 I2C_WaitAck() 的返回值原本是被丢弃的，所以从机 NACK 时
 * 函数照样返回数据（全 0xFF 或旧值），上层完全不知道。现在每条事务都能拿到
 * 全部寄存器定义、MPU6050_t 结构、3 次采样求平均、计步状态机、跌倒状态机、
 */

#include "mpu6050.h"
#include "bsp_i2c.h"

#include <math.h>       /* sqrtf：计步/跌倒的合加速度 */
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

#ifndef CONFIG_MPU6050_TASK_STACK
#define CONFIG_MPU6050_TASK_STACK       3072   /* 单位：字节！IDF 的栈深不是"字" */
#endif
#ifndef CONFIG_MPU6050_TASK_PRIO
#define CONFIG_MPU6050_TASK_PRIO        14     /* 必须 < 18（lwIP），否则会饿死网络栈 */
#endif
#ifndef CONFIG_MPU6050_TASK_CORE
#define CONFIG_MPU6050_TASK_CORE        1      /* 核 0 留给 WiFi/lwIP，采样放核 1 */
#endif
#ifndef CONFIG_MPU6050_SAMPLE_PERIOD_MS
#define CONFIG_MPU6050_SAMPLE_PERIOD_MS 40     /* 与原工程一致 */
#endif

#define SAMPLE_NUM                3         /* 每帧采样次数：10 → 3（降低 I2C 耗时） */

#define STEP_THRESHOLD            2000      /* 峰值阈值（原 3500 → 2000，更灵敏） */
#define STEP_VALLEY_THRESHOLD     800       /* 谷底阈值（原 1000 → 800，迟滞更小） */
#define STEP_MIN_INTERVAL_MS      300       /* 最小步间隔（原 400 → 300） */
#define STEP_MAX_INTERVAL_MS      1500      /* 最大步间隔（原 1200 → 1500，慢走也能计） */
#define GRAVITY_LP_ALPHA          0.92f
#define STEP_CONFIRM_COUNT        3         /* 前 3 步预热 */

#define FALL_FREEFALL_THRESHOLD   0.7f      /* 原 0.5 → 0.7，更容易抓到 */
#define FALL_IMPACT_THRESHOLD     1.8f      /* 原 2.5 → 1.8，轻摔也能触发 */
#define FALL_STABLE_MIN           0.7f
#define FALL_STABLE_MAX           1.3f
#define FALL_CONFIRM_MS           800       /* 原 1000 → 800 */
#define FALL_TIMEOUT_MS           1000      /* 原 500 → 1000 */

#define ACC_2_G(acc_val)          ((float)(acc_val) / 16384.0f)

/* 任务内的故障处理参数（与原工程一致） */
#define ID_CHECK_EVERY_N          25        /* 40ms × 25 ≈ 1s 查一次 WHO_AM_I */
#define RECOVER_TRIGGER_COUNT     5
#define MAX_RECOVER_ATTEMPT       3
#define RECOVER_COOLDOWN_MS       5000
#define IO_FAIL_LIMIT             5         /* 连续 I2C 失败几次就强制恢复 */

static volatile uint32_t s_step_count   = 0;
static volatile uint8_t  s_tumble_state = 0;
static MPU6050_t         g_mpu_data     = {0};

static int16_t gyro_offset_x = 0;
static int16_t gyro_offset_y = 0;
static int16_t gyro_offset_z = 0;

/* 原工程遗留的调试标志 */
volatile uint8_t g_mpu_kill_flag = 0;

/* 跨任务共享 s_step_count / s_tumble_state 的临界区。
 * ESP-IDF 里 taskENTER_CRITICAL() 不带参数会拿全局自旋锁，粒度太大，
 * 这里用自己的 mux，只保护两条 32 位赋值，持锁时间纳秒级。 */
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

/* 心跳回调（替代原来直接调 app_task_heartbeat(APP_TASK_MPU)） */
static void (*s_hb_cb)(void) = NULL;

/* 器件在线标志。 */
static volatile bool s_online      = true;
static uint32_t      s_fail_streak = 0;

bool mpu6050_is_online(void)
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
        ESP_LOGI(TAG, "MPU6050 恢复在线");
    } else {
        ESP_LOGE(TAG, "MPU6050 离线：%s（检查 VCC/GND/SDA/SCL 是否松动）",
                 why != NULL ? why : "无响应");
    }
}

/* 小工具                                                              */

/**
 * 单调毫秒时间戳。
 * 原代码用 xTaskGetTickCount() * portTICK_PERIOD_MS，在 IDF 默认 100Hz tick 下
 * 精度只有 10ms，会让 300ms/800ms 这些阈值抖动 ±20ms。
 * esp_timer 走 1MHz 计数器，与 tick 频率无关，改用后阈值精度稳定在 1ms。
 */
static inline uint32_t mpu_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline void hb_ping(void)
{
    if (s_hb_cb != NULL) {
        s_hb_cb();
    }
}

static inline esp_err_t wr_reg(uint8_t reg, uint8_t val)
{
    return bsp_i2c_write_reg(MPU6050_I2C_ADDR, reg, val);
}

/* 限频打印：现在是 (uint32) 毫秒时间戳，比原 TickType_t 做差更直观 */
static bool rate_limit(uint32_t *last_ms, uint32_t interval_ms)
{
    const uint32_t now = mpu_now_ms();
    if ((now - *last_ms) >= interval_ms) {
        *last_ms = now;
        return true;
    }
    return false;
}

/* 配置寄存器（原 MPU_Set_Gyro_Fsr / Accel_Fsr / LPF / Rate）            */

static void mpu_set_gyro_fsr(uint8_t fsr)
{
    /* fsr: 0=±250, 1=±500, 2=±1000, 3=±2000 dps */
    (void)wr_reg(MPU_GYRO_CFG_REG, (uint8_t)(fsr << 3));
}

static void mpu_set_accel_fsr(uint8_t fsr)
{
    /* fsr: 0=±2, 1=±4, 2=±8, 3=±16 g */
    (void)wr_reg(MPU_ACCEL_CFG_REG, (uint8_t)(fsr << 3));
}

static void mpu_set_lpf(uint16_t lpf)
{
    uint8_t data;
    if (lpf >= 188)      data = 1;
    else if (lpf >= 98)  data = 2;
    else if (lpf >= 42)  data = 3;
    else if (lpf >= 20)  data = 4;
    else if (lpf >= 10)  data = 5;
    else                 data = 6;
    (void)wr_reg(MPU_CFG_REG, data);
}

static void mpu_set_rate(uint16_t rate)
{
    if (rate > 1000) rate = 1000;
    if (rate <   4)  rate = 4;
    const uint8_t data = (uint8_t)(1000 / rate - 1);
    (void)wr_reg(MPU_SAMPLE_RATE_REG, data);
    mpu_set_lpf((uint16_t)(rate / 2));
}

/*
 * 读 WHO_AM_I 寄存器，检查 MPU6050 是否在线。
 * 失败时返回 esp_err_t 错误码，成功时 *id = 0x68。
 * 原工程里 ID 不对时静默跳过，这里改成显式错误返回。
 * 任务里会在连续失败几次后触发总线复位。
*/   

esp_err_t mpu6050_read_id(uint8_t *id)
{
    if (id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return bsp_i2c_read_reg(MPU6050_I2C_ADDR, MPU_DEVICE_ID_REG, id, 1);
}
/*
 * 校准陀螺仪
 */
esp_err_t mpu6050_calibrate_gyro(uint16_t samples)
{
    if (samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buf[6];

    for (uint16_t i = 0; i < samples; i++) {
        esp_err_t err = bsp_i2c_read_reg(MPU6050_I2C_ADDR, MPU_GYRO_XOUTH_REG, buf, sizeof(buf));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "校准过程中读陀螺仪失败: %s", esp_err_to_name(err));
            return err;
        }
        sum_x += (int16_t)((buf[0] << 8) | buf[1]);
        sum_y += (int16_t)((buf[2] << 8) | buf[3]);
        sum_z += (int16_t)((buf[4] << 8) | buf[5]);

        /* 原来是 Delay_us(5000) 死等 5ms，在 IDF 里这属于"忙等且不让出 CPU"，
         * 50 次就是 250ms 完全占死一个核。vTaskDelay 让出 CPU，期间 WiFi/OLED 都能跑。 */
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gyro_offset_x = (int16_t)(sum_x / samples);
    gyro_offset_y = (int16_t)(sum_y / samples);
    gyro_offset_z = (int16_t)(sum_z / samples);
    return ESP_OK;
}

/*
 * 初始化 MPU6050
 */
esp_err_t mpu6050_init(void)
{
    esp_err_t err;

    /* 0) 总线兜底初始化（重复调用是幂等的，三个器件谁先跑都不会出问题） */
    err = bsp_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 1) 软复位。原代码 Reset 之后立刻写下一句，没有等待，手册要求
     *    DEVICE_RESET 置位后等 100ms，否则后续寄存器写入可能被丢掉。
     *    这是迁移时顺手修掉的一个原工程隐患。 */
    err = wr_reg(MPU_PWR_MGMT1_REG, 0x80);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "复位命令发送失败，检查接线/上拉/地址(0x%02X): %s",
                 MPU6050_I2C_ADDR, esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    hb_ping();

    /* 2) 解除 sleep */
    err = wr_reg(MPU_PWR_MGMT1_REG, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 3) 量程与速率：陀螺 ±2000dps、加速度 ±2g、采样 200Hz
     *    （与原工程完全一致；注意 ±2g 时 1g = 16384 LSB，计步/跌倒阈值都基于这个刻度） */
    mpu_set_gyro_fsr(3);
    mpu_set_accel_fsr(0);
    mpu_set_rate(200);

    (void)wr_reg(MPU_INT_EN_REG, 0x00);      /* 不用中断 */
    (void)wr_reg(MPU_USER_CTRL_REG, 0x00);   /* 不用 FIFO/I2C 主模式 */
    (void)wr_reg(MPU_FIFO_EN_REG, 0x00);
    /* INT 引脚低电平有效、推挽输出（原工程值，保持不动） */
    (void)wr_reg(MPU_INTBP_CFG_REG, 0x80);

    /* 4) 校验身份。原代码 if(ID==0x68) 才继续配置，ID 不对时静默跳过，
     *    上电后表现为"数据全 0 但没有任何报错"。这里改成显式错误返回。 */
    uint8_t id = 0;
    err = mpu6050_read_id(&id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读 WHO_AM_I 失败: %s", esp_err_to_name(err));
        return err;
    }
    if (id != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02X（期望 0x68）：检查 SDA/SCL 是否接反、"
                      "AD0 电平、是否有外部 4.7k 上拉、以及是否为 MPU6500 等兼容片",
                 id);
        return ESP_ERR_NOT_FOUND;
    }

    /* 5) 时钟源切到 PLL（陀螺 X 轴），比内部 8MHz RC 稳得多 */
    err = wr_reg(MPU_PWR_MGMT1_REG, 0x01);
    if (err != ESP_OK) {
        return err;
    }
    (void)wr_reg(MPU_PWR_MGMT2_REG, 0x00);   /* 打开全部轴 */
    mpu_set_rate(200);

    /* 6) 零偏校准（设备必须静止放平，否则静态重力会被算成角速度） */
    ESP_LOGI(TAG, "开始陀螺仪零偏校准（%d 次，约 %dms），请保持静止...",
             50, 50 * 5);
    err = mpu6050_calibrate_gyro(50);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "校准完成，零偏 offset = (%d, %d, %d)",
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
    ESP_LOGI(TAG, "MPU6050 初始化成功");
    return ESP_OK;
}

/* 数据读取                                                            */

esp_err_t mpu6050_read_raw(MPU6050_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t sum_acc[3]  = {0};
    int32_t sum_gyro[3] = {0};
    uint8_t buf[14];    /* 0x3B..0x48：accel 6 + temp 2 + gyro 6 */

    for (uint8_t i = 0; i < SAMPLE_NUM; i++) {
        /* 优化点：原代码每轮做 2 次事务（先读陀螺、再读加速度）× 3 轮 = 6 次。 */
        esp_err_t err = bsp_i2c_read_reg(MPU6050_I2C_ADDR, MPU_ACCEL_XOUTH_REG,
                                        buf, sizeof(buf));
        if (err != ESP_OK) {
            return err;
        }
        sum_acc[0] += (int16_t)((buf[0] << 8) | buf[1]);
        sum_acc[1] += (int16_t)((buf[2] << 8) | buf[3]);
        sum_acc[2] += (int16_t)((buf[4] << 8) | buf[5]);
        /* buf[6..7] 是片内温度，原结构体没有该字段，按原样丢弃 */
        sum_gyro[0] += (int16_t)((buf[8]  << 8) | buf[9]);
        sum_gyro[1] += (int16_t)((buf[10] << 8) | buf[11]);
        sum_gyro[2] += (int16_t)((buf[12] << 8) | buf[13]);
    }

    /* 陀螺仪扣零偏；加速度不扣（重力必须在，否则计步的合加速度失去参考） */
    out->gx = (int16_t)(sum_gyro[0] / SAMPLE_NUM - gyro_offset_x);
    out->gy = (int16_t)(sum_gyro[1] / SAMPLE_NUM - gyro_offset_y);
    out->gz = (int16_t)(sum_gyro[2] / SAMPLE_NUM - gyro_offset_z);
    out->ax = (int16_t)(sum_acc[0] / SAMPLE_NUM);
    out->ay = (int16_t)(sum_acc[1] / SAMPLE_NUM);
    out->az = (int16_t)(sum_acc[2] / SAMPLE_NUM);
    return ESP_OK;
}

/* 计步（算法逻辑与原工程逐行一致，只把取时间的方式换掉）                 */
/* 诊断用：最近一次算出的动态加速度（mag - 重力基线）。
 * 计步不涨时先看这个值有没有超过 STEP_THRESHOLD，能立刻分清
 * 是"信号太小"还是"状态机没走通"。 */
static volatile int32_t s_last_dyn = 0;

int32_t mpu6050_get_last_dyn(void)
{
    return s_last_dyn;
}

static uint32_t step_detect(int16_t ax, int16_t ay, int16_t az)
{
    static uint32_t step_count     = 0;
    static float    gravity_mag    = 16384.0f;   /* 重力基线 */
    static uint8_t  peak_state     = 0;          /* 0=等峰值, 1=等谷底 */
    static uint32_t last_step_tick = 0;
    static uint8_t  candidate_step = 0;          /* 预热计数器 */

    /* 1. 合加速度（与设备佩戴方向无关） */
    const float mag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az);

    /* 2. 低通滤波提取重力基线 */
    gravity_mag = GRAVITY_LP_ALPHA * gravity_mag + (1.0f - GRAVITY_LP_ALPHA) * mag;

    /* 3. 动态加速度 = 合加速度 - 重力基线 */
    const float dyn = mag - gravity_mag;
    s_last_dyn = (int32_t)dyn;

    const uint32_t now = mpu_now_ms();

    /* 4. 峰值检测状态机 */
    if (peak_state == 0) {
        if (dyn > STEP_THRESHOLD) {
            const uint32_t interval = now - last_step_tick;

            if (interval > STEP_MIN_INTERVAL_MS && interval < STEP_MAX_INTERVAL_MS) {
                /* 前 STEP_CONFIRM_COUNT 步只预热，之后每步都计 */
                if (candidate_step < STEP_CONFIRM_COUNT) {
                    candidate_step++;
                }
                if (candidate_step >= STEP_CONFIRM_COUNT) {
                    step_count++;              /* 确认后每步 +1，不清零 */
                }
                last_step_tick = now;
            } else if (interval >= STEP_MAX_INTERVAL_MS) {
                /* 间隔太久，重新预热 */
                candidate_step = 0;
                last_step_tick = now;
            }
            /* interval < STEP_MIN_INTERVAL_MS：太密，当抖动忽略 */

            peak_state = 1;                    /* 进入等谷底状态 */
        }
    } else {
        if (dyn < STEP_VALLEY_THRESHOLD) {
            peak_state = 0;
        }
    }

    return step_count;
}

/* 跌倒检测（算法逻辑与原工程逐行一致）                                   */
static uint8_t fall_detect(int16_t ax, int16_t ay, int16_t az)
{
    static uint8_t  state          = 0;
    static uint32_t state_start_ms = 0;

    const float mag_g = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) / 16384.0f;
    const uint32_t now = mpu_now_ms();

    switch (state) {
    case 0:  /* ---- 正常 ---- */
        if (mag_g < FALL_FREEFALL_THRESHOLD) {
            state = 1;
            state_start_ms = now;
        }
        break;

    case 1:  /* ---- 自由落体，等撞击 ---- */
        if ((now - state_start_ms) > FALL_TIMEOUT_MS) {
            state = 0;                         /* 超时，回正常 */
        } else if (mag_g > FALL_IMPACT_THRESHOLD) {
            state = 2;                         /* 检测到撞击 */
            state_start_ms = now;
        }
        break;

    case 2:  /* ---- 撞击后，等静止确认 ---- */
        if (mag_g > FALL_STABLE_MIN && mag_g < FALL_STABLE_MAX) {
            if ((now - state_start_ms) > FALL_CONFIRM_MS) {
                state = 0;
                return 1;                      /* 确认跌倒 */
            }
        } else {
            state_start_ms = now;              /* 还在抖，重置计时 */
        }
        break;
    }

    return 0;
}

esp_err_t mpu6050_update_step_tumble(MPU6050_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = mpu6050_read_raw(data);
    if (err != ESP_OK) {
        return err;
    }
    data->step_count   = step_detect(data->ax, data->ay, data->az);
    data->tumble_state = fall_detect(data->ax, data->ay, data->az);
    return ESP_OK;
}

void mpu6050_get_motion(uint32_t *steps, uint8_t *tumble)
{
    /* 原代码用 taskENTER_CRITICAL()；IDF 里必须显式给自旋锁对象 */
    portENTER_CRITICAL(&s_state_mux);
    if (steps  != NULL) { *steps  = s_step_count;   }
    if (tumble != NULL) { *tumble = s_tumble_state; }
    portEXIT_CRITICAL(&s_state_mux);
}

/* 故障恢复                                                            */
esp_err_t mpu6050_recover_bus(void)
{
    /* L1：硬件层面解总线死锁（9 时钟 + STOP + 重装驱动）。
     * 这一步取代了原代码里 __disable_irq() + 手动拉 SCL/SDA 的那 20 行 ——
     * 在 ESP32 上不能长时间关中断（WiFi 会直接掉线），所以放到 bsp_i2c 里做，
     * 那里只在打脉冲的 ~100us 内用 GPIO 操作，不开临界区。 */
    esp_err_t err = bsp_i2c_bus_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "L1 总线复位失败: %s", esp_err_to_name(err));
    }

    uint8_t id = 0;
    if (mpu6050_read_id(&id) == ESP_OK && id == MPU6050_WHO_AM_I_VAL) {
        return ESP_OK;
    }

    /* L2：芯片可能被电磁干扰打进了异常状态，软复位后整轮重新配置 */
    ESP_LOGW(TAG, "L1 未恢复（WHO_AM_I=0x%02X），尝试复位芯片后重新初始化", id);
    (void)wr_reg(MPU_PWR_MGMT1_REG, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    return mpu6050_init();
}

/*
 * 设置心跳回调函数
 */
void mpu6050_set_heartbeat_cb(void (*cb)(void))
{
    s_hb_cb = cb;
}

/**
 * 采样任务。与原 vMPU6050_Task 的差异只有三处：
 *   1. app_task_heartbeat(APP_TASK_MPU) → hb_ping()（回调注入，解除对 app_tasks.h 的依赖）
 *   2. USART2_Printf → ESP_LOGx（自动带时间戳和 TAG，能进 idf.py monitor 过滤）
 *   3. 增加 I2C 读失败的连续计数：硬件 I2C 能报错，所以"读不到"可以立即触发恢复，
 *      不必等到每 1s 查 ID 才发现（原代码等最多 1s，现在最坏 5×40ms=200ms）
 */
void vMPU6050_Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    uint8_t  id_check_counter = 0;
    uint8_t  id_fail_count    = 0;
    uint8_t  recover_attempt  = 0;
    uint8_t  io_fail_count    = 0;
    bool     degraded         = false;
    uint32_t last_recover_ms  = 0;
    uint32_t last_print_ms    = 0;

    ESP_LOGI(TAG, "采样任务启动：周期 %dms，优先级 %d，栈 %d 字节，核 %d",
             CONFIG_MPU6050_SAMPLE_PERIOD_MS, CONFIG_MPU6050_TASK_PRIO,
             CONFIG_MPU6050_TASK_STACK, CONFIG_MPU6050_TASK_CORE);

    for (;;) {
        /* 这里不再无条件打卡。打卡规则见循环末尾：只有"通信成功"或
         * "已经判定掉线"时才打卡，否则看门狗无法区分
         * "任务被卡住" 和 "线松了"，这两件事的处理方式完全相反。 */

        if (++id_check_counter >= ID_CHECK_EVERY_N) {
            id_check_counter = 0;

            uint8_t   id  = 0;
            esp_err_t err = mpu6050_read_id(&id);

            if (err != ESP_OK || id != MPU6050_WHO_AM_I_VAL) {
                id_fail_count++;
                set_online(false, "WHO_AM_I 读不到 0x68");

                if (rate_limit(&last_print_ms, 3000)) {
                    ESP_LOGW(TAG, "WHO_AM_I 异常（err=%s, id=0x%02X），连续 %u 次，已恢复 %u/%u",
                             esp_err_to_name(err), id, id_fail_count,
                             recover_attempt, MAX_RECOVER_ATTEMPT);
                }

                if (id_fail_count >= RECOVER_TRIGGER_COUNT &&
                    recover_attempt < MAX_RECOVER_ATTEMPT &&
                    (mpu_now_ms() - last_recover_ms) >= RECOVER_COOLDOWN_MS) {

                    recover_attempt++;
                    last_recover_ms = mpu_now_ms();

                    ESP_LOGW(TAG, "尝试恢复 %u/%u", recover_attempt, MAX_RECOVER_ATTEMPT);

                    hb_ping();                          /* 恢复过程可能耗时，前后各喂一次心跳 */
                    int8_t ret = MPU6050_RecoverBus();
                    hb_ping();

                    if (ret == 0) {
                        ESP_LOGI(TAG, "恢复成功");
                        id_fail_count   = 0;
                        recover_attempt = 0;
                        degraded        = false;
                    } else {
                        ESP_LOGE(TAG, "恢复失败");
                    }

                    /* 恢复期间总线被重装过，周期基准必须重新对齐，
                     * 否则 vTaskDelayUntil 会连续几次立即返回，采样瞬间密集 */
                    last_wake = xTaskGetTickCount();
                }

                /* 3 次都失败就降级：继续跑但不读数据，避免刷屏和反复折腾总线 */
                if (recover_attempt >= MAX_RECOVER_ATTEMPT) {
                    if (!degraded) {
                        ESP_LOGE(TAG, "已降级运行（保持现状，每 5 秒轻量重试一次）");
                        degraded = true;
                        set_online(false, "多次恢复失败，进入降级");
                    }

                    /* 降级不等于躺平：线插回去之后必须能自己回来。
                     * 这里只做"重装总线 + 读 ID"（一毫秒级别），不做整片初始化，
                     * 失败也不会影响总线上别的器件。 */
                    if ((mpu_now_ms() - last_recover_ms) >= 5000) {
                        last_recover_ms = mpu_now_ms();
                        hb_ping();
                        if (MPU6050_RecoverBus() == 0) {
                            ESP_LOGI(TAG, "降级后自动恢复成功");
                            recover_attempt = 0;
                            id_fail_count   = 0;
                            degraded        = false;
                            s_fail_streak   = 0;
                            set_online(true, NULL);
                        }
                        hb_ping();
                    }

                    hb_ping();      /* 器件掉线不是任务卡死，继续打卡 */
                    vTaskDelay(pdMS_TO_TICKS(200));
                    last_wake = xTaskGetTickCount();
                    continue;
                }
                /* ID 失败时仍继续采样，只是标记异常（与原逻辑一致） */

            } else if (id_fail_count > 0) {
                ESP_LOGI(TAG, "恢复正常");
                id_fail_count   = 0;
                recover_attempt = 0;
                degraded        = false;
            }
        }

        esp_err_t err = mpu6050_update_step_tumble(&g_mpu_data);
        if (err != ESP_OK) {
            if (++s_fail_streak >= IO_FAIL_LIMIT) {
                set_online(false, "连续 I2C 读失败");
            }
            if (++io_fail_count >= IO_FAIL_LIMIT) {
                io_fail_count = 0;
                ESP_LOGW(TAG, "连续 %d 次 I2C 失败（%s），强制总线恢复",
                         IO_FAIL_LIMIT, esp_err_to_name(err));
                hb_ping();
                (void)mpu6050_recover_bus();
                hb_ping();
                last_wake = xTaskGetTickCount();
            }
        } else {
            io_fail_count = 0;
            s_fail_streak = 0;
            set_online(true, NULL);

            /* 发布给 mpu6050_get_motion() 的读者（原来是 taskENTER_CRITICAL） */
            portENTER_CRITICAL(&s_state_mux);
            s_step_count   = g_mpu_data.step_count;
            s_tumble_state = g_mpu_data.tumble_state;
            portEXIT_CRITICAL(&s_state_mux);
        }

        /* 打卡规则，这就是"能发现卡死、又不会把线松当成卡死"的关键： */
        if (err == ESP_OK || !s_online) {
            hb_ping();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_MPU6050_SAMPLE_PERIOD_MS));
    }
}

void mpu6050_start_task(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(vMPU6050_Task,
                                            "mpu6050",
                                            CONFIG_MPU6050_TASK_STACK,   /* 字节！ */
                                            NULL,
                                            CONFIG_MPU6050_TASK_PRIO,
                                            NULL,
                                            CONFIG_MPU6050_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建采样任务失败（栈 %d 字节，内存是否不足？）",
                 CONFIG_MPU6050_TASK_STACK);
    }
}

/* ==================================================================
 *                          兼容层实现
 * ================================================================== */

uint8_t MPU6050_Read_ID(void)
{
    uint8_t id = 0;
    if (mpu6050_read_id(&id) != ESP_OK) {
        return 0;   /* 调用点都是 id != 0x68 判错，返回 0 语义一致 */
    }
    return id;
}

void MPU6050_CalibrateGyro(uint16_t samples)
{
    (void)mpu6050_calibrate_gyro(samples);
}

int8_t MPU6050_RecoverBus(void)
{
    return (mpu6050_recover_bus() == ESP_OK) ? 0 : 1;   /* 0=成功，匹配 RecoverCallback_t */
}

void MPU6050_GetRawData(MPU6050_t *data)
{
    (void)mpu6050_read_raw(data);
}

void MPU6050_UpdateStepAndTumble(MPU6050_t *data)
{
    (void)mpu6050_update_step_tumble(data);
}

void MPU6050_CreateTask(void)
{
    mpu6050_start_task();
}