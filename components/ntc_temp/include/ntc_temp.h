/**
 * @file    ntc_temp.h
 * @brief   NTC 热敏电阻模块（4 针：VCC / GND / DO / AO）驱动，ESP-IDF 版
 *
 * 和原 DS18B20 的本质区别：
 *   DS18B20 是数字单总线，主机读出来就是 ℃；
 *   NTC 模块是模拟输出，AO 上是一段跟温度相关的电压，温度没有"读"这个动作，
 *   只有"采样 + 换算"。所以本驱动里没有任何 1-Wire 时序，
 *   取而代之的是 ADC 采样 + 分压反算电阻 + Beta 公式。
 *
 * 接线：
 *   VCC → 3V3      GND → GND      AO → GPIO4(ADC1_CH3)      DO → 不接也行
 *   注意：不要接 5V。ESP32-S3 的 ADC 满量程约 3.1V（12dB 衰减），
 *         5V 供电会直接打满甚至损伤引脚。
 *
 * 精度说明：
 *   反算公式用的是 Beta（B 值）模型，不是查表，常温段误差 ~±0.5℃，
 *   两端（<-20℃ / >80℃）误差会到 ±2℃。要更准得做多点标定或换 PT100+MAX31865。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ESP32-S3 上 ADC1_CHn 固定对应 GPIO(n+1)：CH3 = GPIO4。
 * 只能选 ADC1（CH0~CH9，GPIO1~GPIO10）—— ADC2 与 WiFi 冲突，开了 WiFi 就读不到。 */
#define NTC_TEMP_ADC_CHANNEL_DEFAULT    3       /* → GPIO4 */

/* DO 脚（模块上 LM393 比较器输出，电位器调阈值，低电平有效）。
 * 不用就保持 -1，驱动不会去碰这个脚。要用就改成具体 GPIO 号。 */
#define NTC_TEMP_DO_GPIO_IO             (-1)

/* 采样平均次数：NTC 内阻大、易受 WiFi 射频干扰，单次采样抖动可达 20mV */
#define NTC_TEMP_AVG_SAMPLES            16


/**
 * @brief 初始化 ADC1 单次采样通道 + 电压校准，可选配置 DO 脚。
 * @note  可重复调用（幂等）。不初始化 I2C，温度模块跟 I2C 总线无关。
 */
esp_err_t ntc_temp_init(void);


/**
 * @brief 采一次并换算成摄氏度。
 * @param[out] celsius 结果
 * @return ESP_OK / ESP_ERR_INVALID_STATE（未初始化）/ ESP_ERR_INVALID_RESPONSE（开路/短路/超量程）
 */
esp_err_t ntc_temp_read_celsius(float *celsius);

/** 最近一次有效温度（未初始化或从未读成功时返回 NAN） */
float ntc_temp_get_celsius(void);

/** 最近一次读数是否有效 */
bool ntc_temp_is_valid(void);

/** 器件是否在线（连续 3 次读失败即判离线，最常见的原因是 AO 线松了） */
bool ntc_temp_is_online(void);

/** 原始 ADC 值（调试用：判断是"没接"还是"接错分压方向"） */
esp_err_t ntc_temp_read_raw(int *raw);


/** 周期采样的心跳回调（接回 app_task_heartbeat），不设置也能跑 */
void ntc_temp_set_heartbeat_cb(void (*cb)(void));

/** 创建周期采样任务（周期/栈/优先级/绑核走 Kconfig） */
esp_err_t ntc_temp_start_task(void);

/** 周期采样任务本体：原 vTemperature_Task 的替代实现 */
void vTemperature_Task(void *arg);


/** 原 DS18B20 的取值接口，现在返回 NTC 的滤波后温度 */
float ds18b20_get_temperature(void);

/** 原 DS18B20 的另两个别名（App 里可能用到） */
float DS18B20_GetTemp(void);
float DS18B20_ReadTemperature(void);

/** 原 DS18B20 的"启动转换"。NTC 没有转换过程，这里是空实现，仅为保持链接通过 */
void DS18B20_StartConversion(void);

#ifdef __cplusplus
}
#endif