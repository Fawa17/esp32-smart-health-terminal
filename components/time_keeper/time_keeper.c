#include "time_keeper.h"

#include <sys/time.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "time_keeper";

#define TK_NVS_NAMESPACE   "tk"
#define TK_KEY_UNIX        "unix"     /* int64：上次保存的 Unix 时间戳 */
#define TK_KEY_VALID       "valid"    /* u8：这条记录是否可信 */
#define TK_SAVE_PERIOD_S   (5 * 60)   /* 5 分钟存一次：掉电最多丢 5 分钟，又能控制 flash 磨损 */

/* 2020-01-01 00:00:00 UTC。比它早的时间一律视为"没校过时"
 * （ESP32 上电默认是 1970-01-01，DS1302 没电时是 2000-01-01）。 */
#define TK_MIN_VALID_UNIX  1577836800LL

static bool    s_ready;         /* NVS 可用 */
static int64_t s_last_save_s;   /* 上次保存的时间戳，0 = 还没存过 */

static bool time_is_valid(time_t t)
{
    return (int64_t)t >= TK_MIN_VALID_UNIX;
}

static esp_err_t nvs_open_rw(nvs_handle_t *h)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS 分区满或者版本不匹配：擦掉重建。会丢 NVS 里所有数据，
         * 本项目 NVS 只存时间戳，代价可以接受。 */
        ESP_LOGW(TAG, "NVS 需要重建（%s），擦除后重试", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }
    return nvs_open(TK_NVS_NAMESPACE, NVS_READWRITE, h);
}

esp_err_t time_keeper_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 需要重建（%s），擦除后重试", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败：%s（断电保持功能不可用）", esp_err_to_name(err));
        return err;
    }

    nvs_handle_t h;
    err = nvs_open(TK_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS 里还没有时间记录（首次启动或刚擦除过）");
        s_ready = true;
        return ESP_OK;
    }

    int64_t saved = 0;
    uint8_t valid = 0;
    const esp_err_t e_time  = nvs_get_i64(h, TK_KEY_UNIX, &saved);
    const esp_err_t e_valid = nvs_get_u8(h, TK_KEY_VALID, &valid);
    nvs_close(h);

    s_ready = true;

    if (e_time != ESP_OK || e_valid != ESP_OK || !valid || !time_is_valid((time_t)saved)) {
        ESP_LOGI(TAG, "NVS 里没有可用的时间记录");
        return ESP_OK;
    }

    if (time_is_valid(time(NULL))) {
        /* 系统时间已经可信（比如刚 SNTP 校过），就不覆盖它 */
        ESP_LOGI(TAG, "系统时间已有效，忽略 NVS 里的旧值");
        return ESP_OK;
    }

    const struct timeval tv = { .tv_sec = (time_t)saved, .tv_usec = 0 };
    if (settimeofday(&tv, NULL) == 0) {
        s_last_save_s = saved;
        struct tm tmv;
        localtime_r((time_t *)&saved, &tmv);
        ESP_LOGI(TAG, "系统时间未校准，已从 NVS 恢复：%04d-%02d-%02d %02d:%02d:%02d",
                 tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                 tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    } else {
        ESP_LOGW(TAG, "settimeofday 失败，无法恢复时间");
    }
    return ESP_OK;
}

void time_keeper_save(void)
{
    if (!s_ready) {
        return;
    }
    const time_t now = time(NULL);
    if (!time_is_valid(now)) {
        return;                     /* 时间不可信，存下去只会污染下次启动 */
    }

    nvs_handle_t h;
    if (nvs_open_rw(&h) != ESP_OK) {
        return;
    }
    const int64_t v = (int64_t)now;
    esp_err_t err = nvs_set_i64(h, TK_KEY_UNIX, v);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, TK_KEY_VALID, 1);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err == ESP_OK) {
        s_last_save_s = v;
    } else {
        ESP_LOGW(TAG, "保存时间到 NVS 失败：%s", esp_err_to_name(err));
    }
}

void time_keeper_tick(void)
{
    if (!s_ready) {
        return;
    }
    const time_t now = time(NULL);
    if (!time_is_valid(now)) {
        return;
    }
    if (s_last_save_s != 0 && (int64_t)now - s_last_save_s < TK_SAVE_PERIOD_S) {
        return;
    }
    if (s_last_save_s != 0) {
        ESP_LOGD(TAG, "周期保存时间到 NVS");
    }
    time_keeper_save();
}