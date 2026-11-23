#pragma once

#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

/**
 * @file time_keeper.h
 * @brief 把当前时间存进 NVS，断电后开机可恢复。
 *
 * 系统时间和 SNTP 都只在联网后有效，拔电那一刻没人记录。
 * 这里每 5 分钟写一次，开机时若系统时间 < 2020 就恢复，
 * 有网后由 SNTP 覆盖。DS1302 可靠时以 DS1302 为准（精度到秒）。
 */

/** @brief 打开 NVS 并尝试恢复时间，幂等 */
esp_err_t time_keeper_init(void);

/** @brief 立刻写入当前时间（时间不可信时不写） */
void time_keeper_save(void);

/** @brief 主循环里周期调用，内部按 5 分钟节流 */
void time_keeper_tick(void);