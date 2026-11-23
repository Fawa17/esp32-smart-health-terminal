/* ============================================================================
 * ds1302.c — DS1302 三线 RTC 驱动（ESP-IDF）
 *
 * 位操作时序（DS1302 手册 Figure 2/3）：
 *   写：RST 拉高 → 命令字节(LSB 先出，上升沿锁存) → 数据字节 → RST 拉低
 *   读：RST 拉高 → 命令字节 → 释放 IO → 芯片在 SCLK 下降沿输出本位 → 主机采样
 *
 * 一次读写 = 一条事务，RST 全程保持高。DS1302 对时钟频率**没有下限**，
 * 所以事务被高优先级任务或 WiFi 中断打断只是把脉冲拉宽，不会丢数据。
 * 真正需要防的是"两个任务同时操作总线"，用互斥量解决，不需要关中断。
 * ==========================================================================*/
#include "ds1302.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "ds1302";

#define PIN_CLK   CONFIG_DS1302_CLK_GPIO
#define PIN_DAT   CONFIG_DS1302_DAT_GPIO
#define PIN_RST   CONFIG_DS1302_RST_GPIO

#ifndef CONFIG_DS1302_BIT_DELAY_US
#define CONFIG_DS1302_BIT_DELAY_US 2
#endif

/* 半个时钟周期的延时。DS1302 手册要求 SCLK 高/低各 ≥ 250ns，
 * 取 2us 已经留了 8 倍余量；原工程用 30us 是因为当年 HAL_GPIO_WritePin 太慢。 */
#define T_HALF   CONFIG_DS1302_BIT_DELAY_US
/* 采样前的数据保持时间。原 STM32 版在 SCLK 下降沿后固定等 30us 再读
 * （BSP_Drivers/DS1302.c:96 的 Delay_us(30)），这里保持一致。
 * 8 位 × 30us ≈ 240us 一个字节，8 字节突发读约 2ms，完全可以接受。 */
#define T_SAMPLE 30
/* 发送/接收方向切换后的稳定时间，对应原版的 Delay_us(2) */
#define T_SETTLE 2

static SemaphoreHandle_t s_lock;   /* 递归互斥量：一次事务可能嵌套调用 */
static bool              s_ready;

static inline uint8_t bcd2bin(uint8_t v) { return (uint8_t)(((v >> 4) * 10u) + (v & 0x0Fu)); }
static inline uint8_t bin2bcd(uint8_t v) { return (uint8_t)(((v / 10u) << 4) | (v % 10u)); }

/* 合法的 BCD 字节：两个半字节都 ≤ 9。电池没电时寄存器会返回 0x8A 这类垃圾，
 * 不校验就会算出"90 月"这种值。 */
static inline bool bcd_ok(uint8_t v)
{
    return ((v >> 4) <= 9) && ((v & 0x0F) <= 9);
}

static inline void clk_l(void) { gpio_set_level(PIN_CLK, 0); }
static inline void clk_h(void) { gpio_set_level(PIN_CLK, 1); }
static inline void dat_l(void) { gpio_set_level(PIN_DAT, 0); }
static inline void dat_h(void) { gpio_set_level(PIN_DAT, 1); }
static inline void rst_l(void) { gpio_set_level(PIN_RST, 0); }
static inline void rst_h(void) { gpio_set_level(PIN_RST, 1); }

/* 推挽输出（仍可回读）*/
static inline void dat_dir_tx(void) { gpio_set_direction(PIN_DAT, GPIO_MODE_INPUT_OUTPUT); }
/* 释放总线，交给 DS1302 驱动。注意不能用开漏 + 无上拉，见头文件说明 */
static inline void dat_dir_rx(void) { gpio_set_direction(PIN_DAT, GPIO_MODE_INPUT); }

static inline void lock(void)
{
    if (s_lock) {
        xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    }
}

static inline void unlock(void)
{
    if (s_lock) {
        xSemaphoreGiveRecursive(s_lock);
    }
}

/* 位序：真正按 datasheet 的片子是 LSB 先发（原 STM32 版也是 `data & 0x01` 后右移），
 * 但实测有兼容片会反，所以做成可切换，由 probe_comm() 用 RAM 回环自校准。
 * 采样相位不再做成开关，已按原版固定为"下降沿后采样"，见 r_byte()。 */
static bool s_lsb_first = true;

/* 写一个字节：默认 LSB 先出，数据在 SCLK 上升沿被芯片锁存 */
static void w_byte(uint8_t b)
{
    dat_dir_tx();
    for (int i = 0; i < 8; i++) {
        clk_l();
        esp_rom_delay_us(T_HALF);
        const int bit = s_lsb_first ? i : (7 - i);
        if (b & (uint8_t)(1u << bit)) {
            dat_h();
        } else {
            dat_l();
        }
        esp_rom_delay_us(T_HALF);   /* 数据建立时间：原工程这里没有，靠 GPIO 写入延迟"碰巧"满足 */
        clk_h();
        esp_rom_delay_us(T_HALF);
    }
    clk_l();
}

/* 读一个字节。时序严格照抄 STM32 原版（BSP_Drivers/DS1302.c:91-100）：
 *
 *     SCLK 高 -> 2us -> SCLK 低（下降沿，芯片把本位送上线）-> 等 30us -> 采样
 *
 * 原实现写成了"先采样、再补时钟"，等于**少等一个下降沿**：
 * 采到的是上一位（甚至还没送出的悬空电平），整字节右移一位 ——
 * 正好对应实测的"写 0x96 读回 0x4B（= 0x96>>1）"。
 * 另外原版在下降沿后等 30us 才采样（tACC 保守值），原实现只等了 2us，也不够。
 */
static uint8_t r_byte(void)
{
    uint8_t b = 0;

    dat_dir_rx();
    esp_rom_delay_us(T_SETTLE);      /* 换向需要时间，否则第一位会被刚才的输出吃掉 */

    for (int i = 0; i < 8; i++) {
        const int bit = s_lsb_first ? i : (7 - i);

        clk_h();
        esp_rom_delay_us(T_HALF);
        clk_l();                      /* 下降沿：芯片把本位送上 I/O */
        esp_rom_delay_us(T_SAMPLE);   /* 数据稳定后再采（原版 30us，保守但可靠） */

        if (gpio_get_level(PIN_DAT)) {
            b |= (uint8_t)(1u << bit);
        }
    }
    return b;
}

static void xfer_start(void)
{
    dat_dir_tx();
    clk_l();
    dat_l();        /* RST 拉高前的准备：CLK 低、IO 低，避免半截脉冲被当成时钟 */
    rst_l();
    esp_rom_delay_us(T_HALF);
    rst_h();
    esp_rom_delay_us(T_HALF);
}

static void xfer_stop(void)
{
    rst_l();
    clk_l();
    dat_l();
    dat_dir_tx();
    esp_rom_delay_us(T_HALF);
}

/* 寄存器读写（调用者必须已持锁） */
static void reg_w(uint8_t addr, uint8_t val)
{
    xfer_start();
    w_byte(addr);
    w_byte(val);
    xfer_stop();
}

static uint8_t reg_r(uint8_t addr)
{
    xfer_start();
    w_byte((uint8_t)(addr | 0x01u));
    uint8_t v = r_byte();
    xfer_stop();
    return v;
}

/* 写保护开关。DS1302 的 WP 位会屏蔽**所有**写入（含 RAM），
 * 原工程的 burst 写入就是因为没清 WP 而在某些情况下静默失败。 */
static void wp_enable(bool protect)
{
    reg_w(DS1302_REG_CONTROL_W, protect ? 0x80 : 0x00);
}

/* 三线接口没有 ACK：写操作永远"成功"，读回的也可能是全 0。
 * 只有 init 里的 RAM 回环自检能证明通信真的通，结果存在这里。 */
static bool s_comm_ok = true;

/* 掉线重试周期：线松了是能插回去的，所以判了离线也不能一直判死 */
#define RECOVER_PERIOD_MS 30000u
static uint32_t s_last_recover_ms;

/* 通信自检：空闲电平 + RAM 回环。true = 三线通信可用。
 * 抽成函数是为了掉线后能反复重试，只做一次的话，线插回去也不会自己好。 */
static bool probe_comm(bool verbose)
{
    /* 自检 0：DAT 空闲电平。
     * RST 拉低时 DS1302 的 I/O 是高阻，线正常应被上拉成 1；
     * 读到 0 说明这条线被别的东西拉死了（接错脚/短路/没上拉）。
     * 这一条不用发任何时钟，纯 GPIO，能把"接线坏了"和"时序算错了"分开。 */
    rst_l();
    clk_l();
    dat_dir_rx();
    esp_rom_delay_us(100);
    if (verbose) {
        if (gpio_get_level(PIN_DAT)) {
            ESP_LOGI(TAG, "DAT 空闲电平=1（正常）");
        } else {
            ESP_LOGE(TAG, "DAT 空闲电平=0！这根线一直被拉低：检查 DAT 是否接 GPIO%d、"
                          "是否和 CLK/RST 接反、模块是否供电", PIN_DAT);
        }
    }

    /* 自检 1：RAM 回环。三线接口没有 ACK，写永远"成功"，
     * 只有写一个特征值再读回来，才能证明通信真的通。
     *
     * 这里绝不能用 0xA5：0xA5 = 1010 0101，正着念反着念一样，
     *   位序搞反了它也照样"读回成功"——这个自检就形同虚设。
     *   换成 0x96 = 1001 0110（反序 0110 1001）才验得出位移。 */
    lock();
    wp_enable(false);
    bool    found   = false;
    bool    win_lsb = true;
    uint8_t probe   = 0;

    /* 自校准：位序 LSB / MSB 两种，各写一次 0x96 再读回，谁读得准就用谁。
     * 采样相位已按原版固定（下降沿后 30us 采样），不再参与自校准。 */
    for (int pass = 0; pass < 2 && !found; pass++) {
        s_lsb_first = (pass == 0);
        reg_w(0xC0u, 0x96u);          /* RAM 字节 0 的写命令 */
        probe = reg_r(0xC0u);
        if (verbose) {
            ESP_LOGI(TAG, "自校准 %d/2：位序=%s → 读回 0x%02X（期望 0x96）",
                     pass + 1, s_lsb_first ? "LSB" : "MSB", probe);
        }
        if (probe == 0x96u) {
            found   = true;
            win_lsb = s_lsb_first;
        }
    }
    wp_enable(true);
    unlock();

    /* 无论成功与否，都锁定在最后试过的位序上，后续读写用同一套时序 */
    s_lsb_first = win_lsb;

    /* 自检 1b：写保护位能不能清掉。
     * WP(0x80) 只保护时钟寄存器 0x80~0x8C，**不保护区 RAM** ——
     * 所以"RAM 回环保住了、时间却读回乱码"这种组合，最可能是 WP 没清掉：
     * 写时钟寄存器的命令被芯片静默忽略，读回来的还是上电垃圾值。 */
    lock();
    wp_enable(false);
    const uint8_t ctl = reg_r(DS1302_REG_CONTROL_W);
    wp_enable(true);
    unlock();

    /* 自检 1c：时钟寄存器到底能不能写，这是分辨率最高的一条。
     * WP 清得掉 ≠ 时钟区能写：山寨片常常只有 RAM 区能写。
     * 往"分"寄存器(0x82)写合法 BCD 0x59，读回，再还原旧值。 */
    lock();
    wp_enable(false);
    const uint8_t old_min = reg_r(0x82u);
    reg_w(0x82u, 0x59u);
    const uint8_t back_min = reg_r(0x82u);
    reg_w(0x82u, old_min);
    wp_enable(true);
    unlock();

    /* 自检 1d：秒寄存器会不会自己走。
     * 晶振不起振时，秒值停在写入值上不动，这能区分"写不进去"和"不走路"。 */
    lock();
    const uint8_t sec_a = reg_r(0x80u);
    unlock();
    esp_rom_delay_us(1100 * 1000);      /* 等过一个整秒边界 */
    lock();
    const uint8_t sec_b = reg_r(0x80u);
    unlock();

    if (verbose) {
        if (ctl == 0x00u) {
            ESP_LOGI(TAG, "写保护位可控（已清 0），时钟寄存器可写");
        } else {
            ESP_LOGE(TAG, "写保护位清不掉：读回 0x%02X（应为 0x00）。"
                          "时钟寄存器会拒写，时间会一直是上电垃圾值", ctl);
        }

        if (back_min == 0x59u) {
            ESP_LOGI(TAG, "时钟寄存器可写（写 0x59 读回 0x59）");
        } else {
            ESP_LOGE(TAG, "时钟寄存器写不进去：写 0x59 读回 0x%02X（原值 0x%02X）。"
                          "WP 清得掉却写不进时钟区 → 基本可判定是假片/只读片", back_min, old_min);
        }

        if ((sec_a & 0x7Fu) == (sec_b & 0x7Fu)) {
            ESP_LOGW(TAG, "秒值 1.1 秒内没变（0x%02X → 0x%02X）：晶振没起振，或时钟停在 CH=1",
                     sec_a, sec_b);
        } else {
            ESP_LOGI(TAG, "秒值在走（0x%02X → 0x%02X），晶振正常", sec_a, sec_b);
        }
    }

    if (found) {
        if (verbose) {
            ESP_LOGI(TAG, "RAM 回环自检通过（0x96）：位序=%s，时序已锁定",
                     win_lsb ? "LSB" : "MSB");
        }
        return true;
    }

    if (verbose) {
        if (probe == 0x00u) {
            ESP_LOGE(TAG, "RAM 回环自检读回 0x00：DAT 线一直是低电平。"
                          "检查 DAT 是否接在 GPIO%d、模块有没有 10k 上拉、VCC 是否为 3.3V",
                     PIN_DAT);
        } else if (probe == 0xFFu) {
            ESP_LOGE(TAG, "RAM 回环自检读回 0xFF：DAT 线悬空。检查 DAT/CLK/RST 是否接反、模块是否供电");
        } else if (probe == 0x4Bu) {
            ESP_LOGE(TAG, "RAM 回环读回 0x4B = 0x96>>1：整整错一位。"
                          "LSB/MSB 都试过还是错位 → 片子不是标准 DS1302，或 CLK 上有毛刺");
        } else {
            ESP_LOGE(TAG, "RAM 回环读回 0x%02X（期望 0x96）：两种位序都对不上。"
                          "基本可判定是兼容片/坏片，或三线走线太长", probe);
        }
    }
    return false;
}

esp_err_t ds1302_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateRecursiveMutex();
        if (s_lock == NULL) {
            ESP_LOGE(TAG, "创建互斥量失败");
            return ESP_ERR_NO_MEM;
        }
    }

    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_CLK) | (1ULL << PIN_DAT) | (1ULL << PIN_RST),
        .mode         = GPIO_MODE_INPUT_OUTPUT,
        /* DAT 是开漏双向线，DS1302 芯片侧不会主动拉高，必须由外部/内部上拉。
         * 模块上通常有 10k 电阻，但便宜模块经常省掉，省掉的表现恰好就是
         * "读回来全是 0x00"，然后被误判成"时间就是 0 点 0 分"。
         * 这里打开 ESP32 内部上拉（约 45kΩ，弱但足够 100kHz 以下的三线时序）。 */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 上电先摆成空闲态：RST 低、CLK 低、IO 低 */
    rst_l();
    clk_l();
    dat_l();
    esp_rom_delay_us(10);

    lock();
    wp_enable(false);
    /* trickle charger 关掉：绝大多数模块是 3V 纽扣电池，开充电反而会烧电池。
     * 要开的话必须按手册 0x5C 之类写特定二极管+电阻组合，不能随便写。 */
    reg_w(DS1302_REG_CHARGER_W, 0x00);
    wp_enable(true);
    unlock();

    s_ready = true;

    /* 自检 0（DAT 空闲电平） + 自检 1（RAM 回环），详见 probe_comm() */
    s_comm_ok = probe_comm(true);
    if (!s_comm_ok) {
        ESP_LOGE(TAG, "RTC 通信不可用，时间功能暂停（其它功能不受影响）；"
                      "之后每 %u 秒自动重试一次，接好线就会自己恢复",
                 (unsigned)(RECOVER_PERIOD_MS / 1000u));
        ESP_LOGI(TAG, "初始化完成 CLK=%d DAT=%d RST=%d", PIN_CLK, PIN_DAT, PIN_RST);
        return ESP_OK;
    }

    if (ds1302_is_halted()) {
        ESP_LOGW(TAG, "时钟处于停振状态（CH=1），电池可能没电，时间不可信");
    }

    /* ---------------- 自检 3：时间是否可用 ----------------
     * 上电后 DS1302 的时间寄存器可能是 "00-00-00 00:00:00"（月/日为 0，非法）。
     * 这时不能一直报错干等网络对时，写一个合法的默认时间让时钟先跑起来，
     * 屏幕上至少能看到"秒在走"，等 MQTT 下发时间后再校准。
     */
    ds1302_time_t probe;
    if (ds1302_read_time(&probe) != ESP_OK) {
        const ds1302_time_t def = {
            .second = 0, .minute = 0, .hour = 0,
            .date = 1, .month = 1, .week = 6, .year = 0,   /* 2000-01-01 00:00:00 */
        };
        if (ds1302_write_time(&def) == ESP_OK) {
            (void)ds1302_clock_start();
            ESP_LOGW(TAG, "时间不可用，已写入默认值 2000-01-01 00:00:00 并启动时钟（等网络对时后校准）");
        } else {
            ESP_LOGE(TAG, "写入默认时间失败，RTC 不可用");
        }
    }

    ESP_LOGI(TAG, "初始化完成 CLK=%d DAT=%d RST=%d", PIN_CLK, PIN_DAT, PIN_RST);
    return ESP_OK;
}

bool ds1302_is_halted(void)
{
    if (!s_ready) {
        return true;
    }
    lock();
    uint8_t sec = reg_r(DS1302_REG_SEC_W);
    unlock();
    return (sec & DS1302_SEC_CH) != 0;
}

esp_err_t ds1302_clock_start(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    lock();
    wp_enable(false);
    uint8_t sec = reg_r(DS1302_REG_SEC_W);
    reg_w(DS1302_REG_SEC_W, (uint8_t)(sec & ~DS1302_SEC_CH));
    wp_enable(true);
    unlock();
    return ESP_OK;
}

esp_err_t ds1302_clock_stop(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    lock();
    wp_enable(false);
    uint8_t sec = reg_r(DS1302_REG_SEC_W);
    reg_w(DS1302_REG_SEC_W, (uint8_t)(sec | DS1302_SEC_CH));
    wp_enable(true);
    unlock();
    return ESP_OK;
}

void ds1302_burst_to_time(const uint8_t *burst, ds1302_time_t *t)
{
    if (burst == NULL || t == NULL) {
        return;
    }

    t->second = bcd2bin((uint8_t)(burst[0] & 0x7Fu));   /* bit7 是 CH，不是数值 */
    t->minute = bcd2bin(burst[1]);

    /* 小时寄存器 bit7=1 表示 12 小时制，bit5 是 AM/PM。
     * 原工程直接 BCD2HEX，12 小时制下会把"下午 9 点"读成 0x91 → 91 点。 */
    uint8_t h = burst[2];
    if (h & 0x80u) {
        uint8_t h12 = bcd2bin((uint8_t)(h & 0x1Fu));
        if (h12 == 0) {
            h12 = 12;
        }
        t->hour = (uint8_t)((h & 0x20u) ? ((h12 % 12) + 12) : (h12 % 12));
    } else {
        t->hour = bcd2bin((uint8_t)(h & 0x3Fu));
    }

    t->date  = bcd2bin((uint8_t)(burst[3] & 0x3Fu));
    t->month = bcd2bin((uint8_t)(burst[4] & 0x1Fu));
    t->week  = bcd2bin((uint8_t)(burst[5] & 0x07u));
    t->year  = bcd2bin(burst[6]);
}

void ds1302_time_to_burst(const ds1302_time_t *t, uint8_t *burst)
{
    if (t == NULL || burst == NULL) {
        return;
    }

    burst[0] = bin2bcd((uint8_t)(t->second % 60));      /* bit7=0 → 启动振荡 */
    burst[1] = bin2bcd((uint8_t)(t->minute % 60));
    burst[2] = bin2bcd((uint8_t)(t->hour % 24));        /* bit7=0 → 24 小时制 */
    burst[3] = bin2bcd((uint8_t)(t->date % 32));
    burst[4] = bin2bcd((uint8_t)(t->month % 13));
    burst[5] = bin2bcd((uint8_t)(t->week % 8));
    burst[6] = bin2bcd((uint8_t)(t->year % 100));
    burst[7] = 0x80;                                    /* 突发第 8 字节 = 控制寄存器，置写保护 */
}

bool ds1302_time_is_sane(const ds1302_time_t *t)
{
    if (t == NULL) {
        return false;
    }
    return t->year <= 99 &&
           t->month >= 1 && t->month <= 12 &&
           t->date  >= 1 && t->date  <= 31 &&
           t->hour  <= 23 &&
           t->minute <= 59 &&
           t->second <= 59 &&
           t->week  <= 7;
}

esp_err_t ds1302_recover(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    s_last_recover_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (probe_comm(false)) {
        if (!s_comm_ok) {
            ESP_LOGI(TAG, "RTC 通信恢复，重新启用时间功能");
        }
        s_comm_ok = true;
        return ESP_OK;
    }

    s_comm_ok = false;
    return ESP_FAIL;
}

bool ds1302_is_online(void)
{
    return s_comm_ok;
}

/* 掉线时按周期重试一次；true 表示这一次可以继续往下读 */
static bool ensure_comm(void)
{
    if (s_comm_ok) {
        return true;
    }

    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if ((uint32_t)(now - s_last_recover_ms) < RECOVER_PERIOD_MS) {
        return false;
    }
    return (ds1302_recover() == ESP_OK);
}

esp_err_t ds1302_read_time(ds1302_time_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 通信自检没过就别再刷日志了：主循环每 500ms 读一次，
     * 每次都打一条"时间越界"会把串口淹掉，也会让人误以为时间逻辑有问题。
     * 但也不能永久闭嘴，每 30 秒试一次，线插回去要能自己好。 */
    if (!s_comm_ok && !ensure_comm()) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t b[8];

    lock();
    xfer_start();
    w_byte(DS1302_CMD_CLOCK_BURST_R);
    for (int i = 0; i < 8; i++) {
        b[i] = r_byte();
    }
    xfer_stop();
    unlock();

     /* 校验 BCD，避免把掉电垃圾当合法时间返回 */
    for (int i = 0; i < 7; i++) {
        uint8_t raw = b[i];
        if (i == 0) {
            raw &= 0x7Fu; // 屏蔽秒寄存器的 CH 位
        } else if (i == 2) {
            // 小时寄存器：bit7 是 12/24 制，bit6 是 AM/PM
            if (raw & 0x80u) {
                raw &= 0x1Fu; // 12 小时制：屏蔽最高 3 位，保留低 5 位
            } else {
                raw &= 0x3Fu; // 24 小时制：屏蔽最高 2 位，保留低 6 位
            }
        }
        if (!bcd_ok(raw)) {
            ESP_LOGW(TAG, "BCD 非法 [%d]=0x%02X，疑似电池掉电；...", i, b[i]);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    ds1302_burst_to_time(b, out);

    if (!ds1302_time_is_sane(out)) {
        ESP_LOGW(TAG, "时间越界 %04d-%02d-%02d %02d:%02d:%02d；原始字节 "
                      "%02X %02X %02X %02X %02X %02X %02X %02X",
                 2000 + out->year, out->month, out->date,
                 out->hour, out->minute, out->second,
                 b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t ds1302_write_time(const ds1302_time_t *t)
{
    if (t == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ds1302_time_is_sane(t)) {
        ESP_LOGE(TAG, "拒绝写入非法时间 %04d-%02d-%02d %02d:%02d:%02d",
                 2000 + t->year, t->month, t->date, t->hour, t->minute, t->second);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b[8];
    ds1302_time_to_burst(t, b);

    /* 一条事务写完 8 个字节：原工程是 9 条独立事务（含两次 WP 开关），
     * 期间任何一次被打断都可能留下"半新半旧"的时间。
     * 突发写的第 8 字节正好是控制寄存器，写 0x80 顺便把写保护恢复，一举两得。 */
    lock();
    xfer_start();
    w_byte(DS1302_CMD_CLOCK_BURST_W);
    for (int i = 0; i < 8; i++) {
        w_byte(b[i]);
    }
    xfer_stop();
    unlock();

    ESP_LOGI(TAG, "写入时间 %04d-%02d-%02d %02d:%02d:%02d (周%u)",
             2000 + t->year, t->month, t->date, t->hour, t->minute, t->second, t->week);
    return ESP_OK;
}

/* 公历 → Unix 天数（Howard Hinnant 的 days_from_civil，1970-01-01 = 0） */
static int32_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return (int32_t)era * 146097 + (int32_t)doe - 719468;
}

esp_err_t ds1302_read_unix(uint32_t *out_epoch)
{
    if (out_epoch == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ds1302_time_t t;
    esp_err_t err = ds1302_read_time(&t);
    if (err != ESP_OK) {
        return err;
    }

    int32_t days = days_from_civil(2000 + t.year, t.month, t.date);
    *out_epoch = (uint32_t)days * 86400u
               + (uint32_t)t.hour * 3600u
               + (uint32_t)t.minute * 60u
               + t.second;
    return ESP_OK;
}

uint8_t ds1302_ram_read(uint8_t addr)
{
    if (!s_ready || addr >= DS1302_RAM_BYTES) {
        return 0;
    }
    lock();
    uint8_t v = reg_r((uint8_t)(DS1302_RAM_START + 2 * addr));
    unlock();
    return v;
}

esp_err_t ds1302_ram_write(uint8_t addr, uint8_t val)
{
    if (addr >= DS1302_RAM_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    lock();
    wp_enable(false);
    reg_w((uint8_t)(DS1302_RAM_START + 2 * addr), val);
    wp_enable(true);
    unlock();
    return ESP_OK;
}

esp_err_t ds1302_ram_read_burst(uint8_t len, uint8_t *buf)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > DS1302_RAM_BYTES) {
        len = DS1302_RAM_BYTES;
    }

    lock();
    xfer_start();
    w_byte(DS1302_CMD_RAM_BURST_R);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = r_byte();
    }
    xfer_stop();
    unlock();
    return ESP_OK;
}

esp_err_t ds1302_ram_write_burst(uint8_t len, const uint8_t *buf)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > DS1302_RAM_BYTES) {
        len = DS1302_RAM_BYTES;
    }

    lock();
    wp_enable(false);   /* 原工程就是漏了这一步，WP=1 时突发写会静默失效 */
    xfer_start();
    w_byte(DS1302_CMD_RAM_BURST_W);
    for (uint8_t i = 0; i < len; i++) {
        w_byte(buf[i]);
    }
    xfer_stop();
    wp_enable(true);
    unlock();
    return ESP_OK;
}

void DS1302_Init(void)
{
    (void)ds1302_init();
}

void DS1302_ClockStart(void)
{
    (void)ds1302_clock_start();
}

void DS1302_ClockStop(void)
{
    (void)ds1302_clock_stop();
}

void DS1302_ClockClear(void)
{
    /* 原意：清空时间并停振，等主机重新对时。
     * 原实现把月/日也清零，导致后续 ReadTime 读到"0 月 0 日"这种非法值，
     * 这里改成写一个合法的 2000-01-01 00:00:00，然后停振。 */
    const ds1302_time_t zero = {
        .second = 0, .minute = 0, .hour = 0,
        .date = 1, .month = 1, .week = 7, .year = 0,
    };
    (void)ds1302_write_time(&zero);
    (void)ds1302_clock_stop();
    ESP_LOGW(TAG, "时间已清零并停振，等待重新对时");
}

void DS1302_ReadTime(DS1302_Time_t *time)
{
    if (time == NULL) {
        return;
    }
    if (ds1302_read_time(time) != ESP_OK) {
        memset(time, 0, sizeof(*time));
    }
}

void DS1302_WriteTime(uint8_t *buf)
{
    if (buf == NULL) {
        return;
    }
    /* 兼容原工程的下标约定：[1]=年 [2]=月 [3]=日 [4]=时 [5]=分 [6]=秒 [7]=星期 */
    const ds1302_time_t t = {
        .year   = buf[1],
        .month  = buf[2],
        .date   = buf[3],
        .hour   = buf[4],
        .minute = buf[5],
        .second = buf[6],
        .week   = buf[7],
    };
    (void)ds1302_write_time(&t);
}

void DS1302_Write_Time(uint8_t year, uint8_t month, uint8_t day,
                       uint8_t hour, uint8_t minute, uint8_t second, uint8_t week)
{
    const ds1302_time_t t = {
        .year = year, .month = month, .date = day,
        .hour = hour, .minute = minute, .second = second, .week = week,
    };
    (void)ds1302_write_time(&t);
}

void DS1302_WriteRam(uint8_t addr, uint8_t val)
{
    (void)ds1302_ram_write(addr, val);
}

uint8_t DS1302_ReadRam(uint8_t addr)
{
    return ds1302_ram_read(addr);
}

void DS1302_ClearRam(void)
{
    const uint8_t zero[DS1302_RAM_BYTES] = {0};
    /* 原实现是 31 次单字节写（每次 2 条 WP 事务 = 62 条事务），
     * 这里一条突发事务搞定。 */
    (void)ds1302_ram_write_burst(DS1302_RAM_BYTES, zero);
}

void DS1302_ReadTimeBurst(uint8_t *buf)
{
    if (buf == NULL || !s_ready) {
        return;
    }
    lock();
    xfer_start();
    w_byte(DS1302_CMD_CLOCK_BURST_R);
    for (int i = 0; i < 8; i++) {
        buf[i] = r_byte();
    }
    xfer_stop();
    unlock();
}

void DS1302_WriteTimeBurst(uint8_t *buf)
{
    if (buf == NULL || !s_ready) {
        return;
    }
    lock();
    wp_enable(false);
    xfer_start();
    w_byte(DS1302_CMD_CLOCK_BURST_W);
    for (int i = 0; i < 8; i++) {
        w_byte(buf[i]);
    }
    xfer_stop();
    wp_enable(true);
    unlock();
}

void DS1302_ReadRamBurst(uint8_t len, uint8_t *buf)
{
    (void)ds1302_ram_read_burst(len, buf);
}

void DS1302_WriteRamBurst(uint8_t len, uint8_t *buf)
{
    (void)ds1302_ram_write_burst(len, buf);
}