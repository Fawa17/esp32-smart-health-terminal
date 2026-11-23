/* ============================================================
 * ds1302.h - DS1302 三线实时时钟（GPIO 位操作）
 *
 * 不用硬件 SPI：DAT 是双向单线，字节级换向，SPI 主机半双工也做不到。
 * 不用开漏：模块板上无上拉，开漏读不到高电平；用 INPUT_OUTPUT 推送，读时切 INPUT。
 *
 * 接线：VCC-3V3  GND-GND  CLK-GPIO5  DAT-GPIO6  RST-GPIO7
 *
 * 两套缓冲布局不同，不要混用：
 *   ds1302_time_to_burst / burst_to_time:
 *       [0]秒 [1]分 [2]时 [3]日 [4]月 [5]星期 [6]年 [7]控制
 *   DS1302_WriteTime（兼容接口）:
 *       [0]-   [1]年 [2]月 [3]日 [4]时 [5]分 [6]秒 [7]星期
 * 新代码一律用 ds1302_time_t。
 * ==========================================================*/
#ifndef DS1302_H
#define DS1302_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DS1302_REG_SEC_W      0x80
#define DS1302_REG_MIN_W      0x82
#define DS1302_REG_HOUR_W     0x84
#define DS1302_REG_DATE_W     0x86
#define DS1302_REG_MONTH_W    0x88
#define DS1302_REG_DAY_W      0x8A
#define DS1302_REG_YEAR_W     0x8C
#define DS1302_REG_CONTROL_W  0x8E
#define DS1302_REG_CHARGER_W  0x90

/* 突发（burst）命令 */
#define DS1302_CMD_CLOCK_BURST_R  0xBF   /* 一次读回 秒分时日月星期年 控制 */
#define DS1302_CMD_CLOCK_BURST_W  0xBE
#define DS1302_CMD_RAM_BURST_R    0xFF   /* 一次读完 31 字节 RAM */
#define DS1302_CMD_RAM_BURST_W    0xFE

#define DS1302_RAM_START   0xC0   /* RAM 起始地址，之后每字节地址 +2 */
#define DS1302_RAM_BYTES   31

/* 控制寄存器位 */
#define DS1302_CTRL_WP     0x80   /* 1 = 写保护（所有写操作失效） */
#define DS1302_SEC_CH      0x80   /* 秒寄存器 bit7：1 = 时钟停振 */

/* 字段顺序刻意与原工程 DS1302_Time_t 保持一致，方便平滑替换 */
typedef struct {
    uint8_t second;   /* 0-59 */
    uint8_t minute;   /* 0-59 */
    uint8_t hour;     /* 0-23 */
    uint8_t date;     /* 1-31  当月第几天 */
    uint8_t month;    /* 1-12 */
    uint8_t week;     /* 1-7   1=周日 … 7=周六（DS1302 不校验，调用方自己填对） */
    uint8_t year;     /* 0-99  表示 2000-2099 */
} ds1302_time_t;


/**
 * @brief 初始化 GPIO 并关闭 trickle charger。
 *
 * 可重复调用，内部只做一次。初始化后芯片处于"写保护"状态。
 * 注意：本函数**不会**自动把时钟跑起来，也不会设置初始时间 ——
 * 电池掉过的板子 CH 位会是 1（时钟停振），需要自己判断后 ds1302_write_time()。
 */
esp_err_t ds1302_init(void);

/** @brief 时钟是否停振（CH 位 = 1 表示掉过电，读出的时间不可信）。 */
bool ds1302_is_halted(void);

/**
 * @brief 三线通信是否正常（以 RAM 回环自检为准）
 *
 * 三线接口没有 ACK，写操作永远"成功"，所以只能用"写特征值再读回"来证明通信。
 * 判定离线后 `ds1302_read_time()` 每 30 秒会自动调一次 `ds1302_recover()`。
 */
bool ds1302_is_online(void);

/** @brief 立刻重做一次通信自检（空闲电平 + RAM 回环），成功则恢复时间功能 */
esp_err_t ds1302_recover(void);

/** @brief 启动振荡器（清 CH 位）。 */
esp_err_t ds1302_clock_start(void);

/** @brief 停止振荡器（置 CH 位），停振时功耗最低但时间不再走。 */
esp_err_t ds1302_clock_stop(void);

/**
 * @brief 一次突发读回整个时间。
 *
 * 内部是一条完整事务（RST 拉高 → 发 0xBF → 连读 8 字节 → RST 拉低），
 * 所以读到的秒分时永远是"同一瞬间"的，不会出现 59→00 跨秒撕裂。
 *
 * @param out 输出
 * @return ESP_OK；BCD 非法返回 ESP_ERR_INVALID_RESPONSE；未初始化返回 ESP_ERR_INVALID_STATE
 */
esp_err_t ds1302_read_time(ds1302_time_t *out);

/**
 * @brief 一次突发写入整个时间，写完自动置写保护。
 *
 * 内部同样是单条事务，不会出现"只写了年、月还没写"的中间态被别人读到。
 * 写入会把 CH 位清零（即时钟开始走）。
 */
esp_err_t ds1302_write_time(const ds1302_time_t *t);

/** @brief 校验时间字段是否在合法范围内。 */
bool ds1302_time_is_sane(const ds1302_time_t *t);

/**
 * @brief 读时间并换算成 Unix 时间戳（2000-01-01 之后有效）。
 *
 * 上行 JSON 里的时间戳、MQTT 的 NTP 对时基准都用得上。
 */
esp_err_t ds1302_read_unix(uint32_t *out_epoch);

/* 31 字节掉电保持 RAM，可用来存"网络已配网"标志、跌倒阈值等小配置 */

uint8_t    ds1302_ram_read(uint8_t addr);
esp_err_t  ds1302_ram_write(uint8_t addr, uint8_t val);
esp_err_t  ds1302_ram_read_burst(uint8_t len, uint8_t *buf);
esp_err_t  ds1302_ram_write_burst(uint8_t len, const uint8_t *buf);

/** @brief 8 字节突发缓冲 → 结构体（[0]=秒 … [6]=年，[7]=控制位忽略）。 */
void ds1302_burst_to_time(const uint8_t *burst, ds1302_time_t *t);

/** @brief 结构体 → 8 字节突发缓冲（[7] 固定写 0x80 置写保护）。 */
void ds1302_time_to_burst(const ds1302_time_t *t, uint8_t *burst);

typedef ds1302_time_t DS1302_Time_t;

void    DS1302_Init(void);
void    DS1302_ClockStart(void);
void    DS1302_ClockStop(void);
void    DS1302_ClockClear(void);
void    DS1302_ReadTime(DS1302_Time_t *time);
void    DS1302_WriteTime(uint8_t *buf);   /* 布局 [1]=年…[7]=星期，见文件头说明 */
void    DS1302_Write_Time(uint8_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute, uint8_t second, uint8_t week);
void    DS1302_WriteRam(uint8_t addr, uint8_t val);
uint8_t DS1302_ReadRam(uint8_t addr);
void    DS1302_ClearRam(void);
void    DS1302_ReadTimeBurst(uint8_t *buf);        /* 仍是原始 8 字节突发布局 */
void    DS1302_WriteTimeBurst(uint8_t *buf);       /* 仍是原始 8 字节突发布局 */
void    DS1302_ReadRamBurst(uint8_t len, uint8_t *buf);
void    DS1302_WriteRamBurst(uint8_t len, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* DS1302_H */