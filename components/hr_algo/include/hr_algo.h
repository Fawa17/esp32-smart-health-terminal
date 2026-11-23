/**
 * @file    hr_algo.h
 * @brief   心率 / 血氧算法（Maxim AN6409 修正版）
 *
 * 输入：200 点 @100Hz 的 PPG 原始波形（IR 与 RED，各 18 bit）
 * 输出：心率 bpm、血氧 %、以及一组诊断量（记录本次未出数的原因）
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 算法窗口长度：200 点 @100Hz = 2 秒（原工程 BUFFER_SIZE） */
#define HR_ALGO_WINDOW      200

/** 采样率，必须和 MAX30102 的 SPO2_CONFIG 实际配置一致（0x27 → 100sps） */
#define HR_ALGO_FS          100

/** 无效值哨兵，与原工程保持一致 */
#define HR_ALGO_INVALID     (-999)

/**
 * @brief 一次算法运算的完整结果
 */
typedef struct {
    int32_t heart_rate;         /**< bpm；无效时 = HR_ALGO_INVALID */
    int32_t spo2;               /**< %；无效时 = HR_ALGO_INVALID */
    bool    hr_valid;
    bool    spo2_valid;

    int32_t peak_count;         /**< 2 秒内检出的脉搏波峰数 */
    int32_t interval_median;    /**< 峰间隔中位数（采样点数） */
    int32_t interval_spread;    /**< 最大间隔 - 最小间隔（采样点数） */
    int32_t perfusion;          /**< 灌注指数 PI，单位 0.1%：AC/DC×1000 */
    int32_t ratio_x20;          /**< R 值 ×20，血氧查表用的就是它 */
    const char *reject;         /**< 无效原因；有效时为 "ok" */
} hr_algo_result_t;

/**
 * @brief 处理一窗 200 点 PPG 数据。
 *
 * @param[in]  ir   红外通道原始值，长度必须 = HR_ALGO_WINDOW
 * @param[in]  red  红光通道原始值，长度必须 = HR_ALGO_WINDOW
 * @param[in]  len  长度
 * @param[out] out  结果（函数内部会先整体清零并置无效值）
 *
 * @return ESP_OK            运算完成（注意：ESP_OK 不代表算出了数，
 *                           要看 out->hr_valid / out->spo2_valid）
 *         ESP_ERR_INVALID_ARG 参数错
 *
 * @note 非可重入：内部用了 3 个静态缓冲区。只在 HR 算法任务里调用。
 */
esp_err_t hr_algo_process(const uint32_t *ir, const uint32_t *red,
                          int len, hr_algo_result_t *out);

/**
 * @brief 原工程的函数原型，签名一字不改，max30102.c 无需修改即可链接。
 * @note  加了这个强符号之后，max30102.c 里的 __attribute__((weak)) 桩自动失效。
 */
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                            int32_t   n_ir_buffer_length,
                                            uint32_t *pun_red_buffer,
                                            int32_t  *pn_spo2,
                                            int8_t   *pch_spo2_valid,
                                            int32_t  *pn_heart_rate,
                                            int8_t   *pch_hr_valid);

#ifdef __cplusplus
}
#endif