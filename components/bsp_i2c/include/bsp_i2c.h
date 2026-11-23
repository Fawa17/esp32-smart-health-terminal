/**
 * @file    bsp_i2c.h
 * @brief   硬件 I2C 总线封装（ESP-IDF legacy driver，driver/i2c.h）
 *
 * 三个器件共用 I2C0：MPU6050 0x68 / MAX30102 0x57 / CST816S 0x15。
 * 总线统一在这里安装，避免各器件重复 i2c_driver_install 同一端口。
 * 引脚 SDA=GPIO8，SCL=GPIO9，400 kHz。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================= 引脚与总线参数（全工程唯一定义处） =======================
 * 选脚依据（ESP32-S3）：
 *   GPIO8 / GPIO9 属于普通 IO，不参与 Flash(26~32)、八线 PSRAM(33~37)、
 *   USB(19/20)、strapping(0/3/45/46)，最安全。
 * 若改板，只改这里，三个器件驱动一起生效。
 */
#ifndef BSP_I2C_PORT
#define BSP_I2C_PORT            I2C_NUM_0   /* ESP32-S3 有 I2C0 / I2C1 两个控制器 */
#endif

#define BSP_I2C_SDA_GPIO        GPIO_NUM_8
#define BSP_I2C_SCL_GPIO        GPIO_NUM_9
#define BSP_I2C_CLK_HZ          400000      /* 400kHz 快速模式；线长 <20cm 时可试 1MHz */
#define BSP_I2C_TIMEOUT_MS      50          /* 单次事务超时，勿设太小：被 WiFi 抢占时会抖动 */


/**
 * @brief 安装并配置 I2C 主机驱动。可重复调用（内部做幂等保护）。
 * @note  必须在任何器件读写之前调用。器件init() 内部也会兜底调用一次。
 */
esp_err_t bsp_i2c_init(void);

/* ============================== 通用收发 ==============================
 * 所有 dev7 参数都是 **7 位地址**（0x68），不是 0xD0 那种 8 位写地址。
 */

/** 向器件连续写 len 字节（buf[0] 通常就是寄存器地址） */
esp_err_t bsp_i2c_write(uint8_t dev7, const uint8_t *buf, size_t len);

/** 从器件连续读 len 字节（从当前指针位置开始，一般配合 write 先设地址） */
esp_err_t bsp_i2c_read(uint8_t dev7, uint8_t *buf, size_t len);

/** 写单个寄存器：等价于 STM32 版的 I2C_WriteReg() */
esp_err_t bsp_i2c_write_reg(uint8_t dev7, uint8_t reg, uint8_t val);

/** 读寄存器块：等价于 STM32 版的 I2C_ReadReg() / I2C_Read_Regs()
 *  内部是一整条"写 reg → 重复起始 → 读 len"的复合事务，不会中途放总线 */
esp_err_t bsp_i2c_read_reg(uint8_t dev7, uint8_t reg, uint8_t *buf, size_t len);

/** 探测某地址是否存在 ACK（返回 ESP_OK 表示器件在线） */
esp_err_t bsp_i2c_probe(uint8_t dev7);


/**
 * @brief 总线级恢复：卸载驱动 → 引脚还原为 GPIO → 打 9 个 SCL 脉冲 + STOP → 重装驱动。
 *
 * 为什么需要：某个从机在传输中途掉电/复位，可能把 SDA 一直拉低，主机此后所有 START 都失败。
 * 硬件 I2C 的外设解不了这个死锁，只能把引脚抢回来手动敲时钟。
 *
 * @note 调用期间会短暂卸载驱动，必须保证没有别的任务正在收发（内部已用互斥量串行化）。
 */
esp_err_t bsp_i2c_bus_reset(void);


/** 扫描 0x08~0x77 并在日志里打印每个有应答的地址（上电自检用，正式版可删） */
void bsp_i2c_scan(void);

#ifdef __cplusplus
}
#endif