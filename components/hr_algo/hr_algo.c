/**
 * @file    hr_algo.c
 * @brief   心率 / 血氧算法，源自 Maxim AN6409（MAX30102 官方例程），已修正 + 加固
 *
 * [BUG-6] 199~203 行：int32 乘法可能溢出
 * 与官方流程完全一致，只是把上面这些坑填了。
 */

#include "hr_algo.h"

#include <string.h>


#define MA4_SIZE            4       /* 4 点滑动平均，不要改 */
#define HAMMING_SIZE        5       /* 汉明窗长度，不要改 */
#define MAX_PEAKS           15      /* 峰/谷数组容量，与官方一致 */

/* 汉明窗系数：long16(512 * hamming(5)) */
static const int16_t s_hamm[HAMMING_SIZE] = {41, 276, 512, 276, 41};

/* 血氧查表：R 值 → SpO2(%)，由 -45.060R² + 30.354R + 94.845 预先算好，下标即 R×20。
 * 数组长度交给编译器推导：原版写死 [184] 但只给了 183 个初值，
 * 末尾那个隐式的 0 会让 R=183 查到"血氧 0%"这种荒唐值。 */
static const uint8_t s_spo2_table[] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
    97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
    90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
    80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
    66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
    28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
    3, 2, 1
};

/* 生理量程判据（[OPT-1]） */
#define HR_MIN_BPM              30
#define HR_MAX_BPM              220
#define HR_INTERVAL_SPREAD_PCT  35      /* 间隔极差 / 中位数，超过则判节律不齐 */
#define SPO2_MIN_PCT            70      /* 低于此值必定是信号问题，不是真人血氧 */
#define PERFUSION_MIN           1       /* PI 单位 0.1%，即 0.1% */
#define DC_MIN                  2000    /* IR 直流分量下限（18 位满量程 262143） */
#define DC_MAX                  260000  /* 接近满量程 → 饱和 */

/* 3 × 200 × 4B = 2.4KB 静态内存。非可重入，只在 HR 任务里调用。 */
static int32_t s_x[HR_ALGO_WINDOW];     /* 先放去直流的 IR，后放原始 IR */
static int32_t s_y[HR_ALGO_WINDOW];     /* 原始 RED */
static int32_t s_dx[HR_ALGO_WINDOW];    /* 差分 + 汉明窗后的陡峭度 */


/** 插入排序，升序。n 很小（≤15），比快速排序更快也更省栈。 */
static void sort_ascend(int32_t *x, int32_t n)
{
    for (int32_t i = 1; i < n; i++) {
        int32_t tmp = x[i];
        int32_t j = i;
        while (j > 0 && tmp < x[j - 1]) {
            x[j] = x[j - 1];
            j--;
        }
        x[j] = tmp;
    }
}

/** 按 x[idx] 的值对下标做降序排序 */
static void sort_indices_descend(const int32_t *x, int32_t *idx, int32_t n)
{
    for (int32_t i = 1; i < n; i++) {
        int32_t tmp = idx[i];
        int32_t j = i;
        while (j > 0 && x[tmp] > x[idx[j - 1]]) {
            idx[j] = idx[j - 1];
            j--;
        }
        idx[j] = tmp;
    }
}


/** 找出所有高于 min_height 的峰（含平台峰，取平台左沿） */
static void peaks_above_min_height(int32_t *locs, int32_t *npks,
                                   const int32_t *x, int32_t n,
                                   int32_t min_height)
{
    int32_t i = 1;
    *npks = 0;

    while (i < n - 1) {
        if (x[i] > min_height && x[i] > x[i - 1]) {
            /* 向右找平台右沿 */
            int32_t width = 1;
            while (i + width < n && x[i] == x[i + width]) {
                width++;
            }
            /* [BUG-4 同类] 平台一直延伸到数组末尾时 i+width == n，必须先判边界再取值 */
            if (i + width >= n) {
                break;
            }
            if (x[i] > x[i + width] && *npks < MAX_PEAKS) {
                locs[(*npks)++] = i;
                i += width + 1;
            } else {
                i += width;
            }
        } else {
            i++;
        }
    }
}

/** 去掉间隔小于 min_distance 的相邻峰（保留幅值大的那个） */
static void remove_close_peaks(int32_t *locs, int32_t *npks,
                               const int32_t *x, int32_t min_distance)
{
    sort_indices_descend(x, locs, *npks);

    for (int32_t i = -1; i < *npks; i++) {
        int32_t old_npks = *npks;
        *npks = i + 1;
        for (int32_t j = i + 1; j < old_npks; j++) {
            int32_t dist = locs[j] - (i == -1 ? -1 : locs[i]);
            if (dist > min_distance || dist < -min_distance) {
                if (*npks < MAX_PEAKS) {
                    locs[(*npks)++] = locs[j];
                }
            }
        }
    }

    sort_ascend(locs, *npks);
}

static void find_peaks(int32_t *locs, int32_t *npks, const int32_t *x, int32_t n,
                       int32_t min_height, int32_t min_distance, int32_t max_num)
{
    peaks_above_min_height(locs, npks, x, n, min_height);
    remove_close_peaks(locs, npks, x, min_distance);
    if (*npks > max_num) {
        *npks = max_num;
    }
}


esp_err_t hr_algo_process(const uint32_t *ir, const uint32_t *red,
                          int len, hr_algo_result_t *out)
{
    if (ir == NULL || red == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* [BUG-2] 先整体清零并置无效：任何一条提前 return 的分支都不会留下垃圾值 */
    memset(out, 0, sizeof(*out));
    out->heart_rate = HR_ALGO_INVALID;
    out->spo2       = HR_ALGO_INVALID;
    out->hr_valid   = false;
    out->spo2_valid = false;
    out->reject     = "ok";

    if (len != HR_ALGO_WINDOW) {
        out->reject = "窗口长度必须为 200";
        return ESP_ERR_INVALID_ARG;
    }

    int64_t sum_ir = 0, sum_red = 0;
    for (int k = 0; k < len; k++) {
        sum_ir  += ir[k];
        sum_red += red[k];
    }
    const int32_t dc_ir  = (int32_t)(sum_ir / len);
    const int32_t dc_red = (int32_t)(sum_red / len);

    /* 手指没贴、或者灯太亮打饱和，后面所有计算都是垃圾，直接判无效 */
    if (dc_ir < DC_MIN || dc_ir > DC_MAX || dc_red < DC_MIN || dc_red > DC_MAX) {
        out->reject = (dc_ir > DC_MAX) ? "IR 信号饱和（灯太亮或距离太近）"
                                       : "无有效信号（手指未贴合 / LED 未亮）";
        return ESP_OK;
    }

    for (int k = 0; k < len - MA4_SIZE; k++) {
        int32_t v = (int32_t)(((int64_t)ir[k] + ir[k + 1] + ir[k + 2] + ir[k + 3]) / 4);
        s_x[k] = v - dc_ir;
    }

    for (int k = 0; k < len - MA4_SIZE - 1; k++) {
        s_dx[k] = s_x[k + 1] - s_x[k];
    }

    for (int k = 0; k < len - MA4_SIZE - 2; k++) {
        s_dx[k] = (s_dx[k] + s_dx[k + 1]) / 2;
    }

    const int32_t n3 = len - MA4_SIZE - HAMMING_SIZE - 2;   /* 189 */
    for (int32_t i = 0; i < n3; i++) {
        int32_t acc = 0;
        for (int32_t k = i; k < i + HAMMING_SIZE; k++) {
            acc -= s_dx[k] * s_hamm[k - i];
        }
        s_dx[i] = acc / 1146;      /* 1146 = sum(s_hamm) */
    }

    /* [BUG-7] 统计范围与有效范围统一为 n3 */
    int64_t abs_sum = 0;
    for (int32_t k = 0; k < n3; k++) {
        abs_sum += (s_dx[k] > 0) ? s_dx[k] : -s_dx[k];
    }
    int32_t th = (int32_t)(abs_sum / n3);
    if (th < 1) {
        out->reject = "信号平坦（无脉动）";
        return ESP_OK;
    }

    int32_t peak_locs[MAX_PEAKS];
    int32_t npks = 0;
    find_peaks(peak_locs, &npks, s_dx, n3, th, 8, 5);

    out->peak_count = npks;
    if (npks < 2) {
        out->reject = "波峰数不足（灌注太弱或手指抖动）";
        return ESP_OK;
    }

    /* [BUG-5] 原版用算术平均，一个漏检/多检就能把结果拉偏十几 bpm */
    int32_t iv[MAX_PEAKS];
    const int32_t n_iv = npks - 1;
    for (int32_t k = 0; k < n_iv; k++) {
        iv[k] = peak_locs[k + 1] - peak_locs[k];
    }
    sort_ascend(iv, n_iv);

    int32_t med_iv;
    if (n_iv >= 2) {
        med_iv = (iv[(n_iv - 1) / 2] + iv[n_iv / 2]) / 2;   /* 偶数个取中间两个的平均 */
    } else {
        med_iv = iv[0];
    }
    out->interval_median = med_iv;
    out->interval_spread = iv[n_iv - 1] - iv[0];

    if (med_iv <= 0) {
        out->reject = "峰间隔异常";
        return ESP_OK;
    }

    /* 60 * FS / 间隔(点) = 6000 / 间隔，四舍五入 */
    const int32_t hr = (6000 + med_iv / 2) / med_iv;
    out->heart_rate = hr;

    if (hr < HR_MIN_BPM || hr > HR_MAX_BPM) {
        out->reject = "心率超出 30~220 生理范围";
        return ESP_OK;
    }
    /* 节律不齐：多半是运动伪影或手指在动，此时的血氧也不可信 */
    if (n_iv >= 2 && out->interval_spread * 100 > med_iv * HR_INTERVAL_SPREAD_PCT) {
        out->reject = "节律不齐（运动伪影）";
        return ESP_OK;
    }
    out->hr_valid = true;

    for (int k = 0; k < len; k++) {
        s_x[k] = (int32_t)ir[k];
        s_y[k] = (int32_t)red[k];
    }

    /* [BUG-3] 用独立写指针紧凑存放，避免中间留洞后按 [0..count-1] 访问到未初始化内存 */
    int32_t valley[MAX_PEAKS];
    int32_t n_valley = 0;
    for (int32_t k = 0; k < npks; k++) {
        const int32_t m = peak_locs[k] + HAMMING_SIZE / 2;
        if (m - 5 <= 0 || m + 5 >= len) {
            continue;                       /* [BUG-4] 严格的边界判断，不再越界读 */
        }
        int32_t best = m;
        int32_t best_v = s_x[m];
        for (int32_t i = m - 5; i < m + 5; i++) {
            if (s_x[i] < best_v) {
                best_v = s_x[i];
                best = i;
            }
        }
        valley[n_valley++] = best;
    }

    if (n_valley < 2) {
        out->reject = "谷点不足，无法计算 AC/DC";
        return ESP_OK;
    }

    for (int k = 0; k < len - MA4_SIZE; k++) {
        s_x[k] = (s_x[k] + s_x[k + 1] + s_x[k + 2] + s_x[k + 3]) / 4;
        s_y[k] = (s_y[k] + s_y[k + 1] + s_y[k + 2] + s_y[k + 3]) / 4;
    }

    int32_t ratio[5];
    int32_t n_ratio = 0;
    int32_t pi_best = 0;

    for (int32_t k = 0; k < n_valley - 1; k++) {
        const int32_t a = valley[k];
        const int32_t b = valley[k + 1];
        if (b - a <= 10) {
            continue;                       /* 一个周期太短，不算 */
        }

        int32_t x_max = -1, x_idx = a;
        int32_t y_max = -1, y_idx = a;
        for (int32_t i = a; i < b; i++) {
            if (s_x[i] > x_max) { x_max = s_x[i]; x_idx = i; }
            if (s_y[i] > y_max) { y_max = s_y[i]; y_idx = i; }
        }
        if (x_max <= 0 || y_max <= 0) {
            continue;
        }

        /* 用两个谷点之间的直线拟合出"纯直流基线"，再从峰值里减掉，
         * 得到的就是纯交流分量（这就是 AC 的定义） */
        const int64_t span = b - a;
        const int32_t base_y = (int32_t)(s_y[a] + (int64_t)(s_y[b] - s_y[a]) * (y_idx - a) / span);
        const int32_t base_x = (int32_t)(s_x[a] + (int64_t)(s_x[b] - s_x[a]) * (x_idx - a) / span);

        /* [BUG-1] 红外 AC 必须用红外自己的峰值下标 x_idx，原版误用了 y_idx */
        const int32_t y_ac = s_y[y_idx] - base_y;
        const int32_t x_ac = s_x[x_idx] - base_x;

        if (x_ac <= 0 || y_ac <= 0) {
            continue;
        }

        /* [BUG-6] 全程 int64，彻底避免 18 位量的乘积溢出，也消掉 >>7 的截断 */
        const int64_t num = (int64_t)y_ac * x_max * 20;   /* ×20：R 的分辨率 0.05 */
        const int64_t den = (int64_t)x_ac * y_max;
        if (den <= 0) {
            continue;
        }

        const int32_t r = (int32_t)(num / den);
        if (r > 0 && n_ratio < 5) {
            ratio[n_ratio++] = r;
        }

        /* 灌注指数 PI = AC/DC，单位 0.1% */
        const int32_t pi = (int32_t)((int64_t)x_ac * 1000 / x_max);
        if (pi > pi_best) {
            pi_best = pi;
        }
    }

    out->perfusion = pi_best;

    if (n_ratio == 0) {
        out->reject = "没有有效的计算周期";
        return ESP_OK;
    }

    sort_ascend(ratio, n_ratio);
    int32_t med_r;
    if (n_ratio >= 2) {
        med_r = (ratio[(n_ratio - 1) / 2] + ratio[n_ratio / 2]) / 2;
    } else {
        med_r = ratio[0];
    }
    out->ratio_x20 = med_r;

    /* [OPT-1] 灌注太弱时 R 值没有意义，直接判无效而不是硬查表 */
    if (pi_best < PERFUSION_MIN) {
        out->reject = "灌注指数过低（手指没压紧或手太凉）";
        return ESP_OK;
    }

    if (med_r > 2 && med_r < (int32_t)sizeof(s_spo2_table)) {
        const int32_t spo2 = s_spo2_table[med_r];
        if (spo2 < SPO2_MIN_PCT) {
            /* 表里能查到但低到不可能，一定是信号问题，宁可报无效 */
            out->reject = "R 值异常（血氧低于 70%）";
            return ESP_OK;
        }
        out->spo2 = spo2;
        out->spo2_valid = true;
    } else {
        out->reject = "R 值超出查表范围";
    }

    return ESP_OK;
}


void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                            int32_t   n_ir_buffer_length,
                                            uint32_t *pun_red_buffer,
                                            int32_t  *pn_spo2,
                                            int8_t   *pch_spo2_valid,
                                            int32_t  *pn_heart_rate,
                                            int8_t   *pch_hr_valid)
{
    hr_algo_result_t r;

    /* 无论算得成算不成，先把输出写成无效值，绝不把未初始化的栈内容交给调用方 */
    if (pn_spo2)       { *pn_spo2 = HR_ALGO_INVALID; }
    if (pch_spo2_valid){ *pch_spo2_valid = 0; }
    if (pn_heart_rate) { *pn_heart_rate = HR_ALGO_INVALID; }
    if (pch_hr_valid)  { *pch_hr_valid = 0; }

    if (pun_ir_buffer == NULL || pun_red_buffer == NULL) {
        return;
    }

    if (hr_algo_process(pun_ir_buffer, pun_red_buffer, (int)n_ir_buffer_length, &r) != ESP_OK) {
        return;
    }

    if (pn_heart_rate)  { *pn_heart_rate = r.heart_rate; }
    if (pch_hr_valid)   { *pch_hr_valid = r.hr_valid ? 1 : 0; }
    if (pn_spo2)        { *pn_spo2 = r.spo2; }
    if (pch_spo2_valid) { *pch_spo2_valid = r.spo2_valid ? 1 : 0; }
}