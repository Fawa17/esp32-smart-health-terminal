/*
 * app_data.c，数据中枢实现
 *
 * 用 portMUX 而不是互斥量：临界区里只做一次几十字节的 memcpy，
 * 没有阻塞调用、没有日志、没有内存分配，这种场景 portMUX 更省也更快，
 * 而且可以在中断里安全使用（虽然目前没有中断会调它）。
 *
 * 如果哪天想支持"读者拿指针直接访问"，就必须改成双缓冲 + 序号校验
 * （读者发现序号是奇数就重读），但那种优化在几十字节的结构上没有意义。
 */

#include <string.h>

#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "app_data.h"

static const char *TAG = "app_data";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static app_snapshot_t s_snapshot;
static app_cfg_t      s_cfg;
static bool           s_has_data = false;
static uint32_t       s_seq      = 0;
/*
 * 数据中枢初始化
*/
esp_err_t app_data_init(void)
{
    portENTER_CRITICAL(&s_lock);
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_cfg.heart_max = 120;
    s_cfg.spo2_min  = 90;
    s_cfg.temp_max  = 37.5f;
    s_cfg.temp_min  = 35.0f;
    s_cfg.mode      = 1;      /* 1=标准运行；下行可以改成 2/3 */
    s_has_data = false;
    s_seq      = 0;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "数据中枢就绪（默认阈值 心率<%ld 血氧>%ld 温度 %.1f~%.1f）",
             (long)s_cfg.heart_max, (long)s_cfg.spo2_min,
             (double)s_cfg.temp_min, (double)s_cfg.temp_max);
    return ESP_OK;
}
/*
 * 发布一份新快照（只允许一个写者）。返回 false 表示参数非法
*/
bool app_data_publish(const app_snapshot_t *s)
{
    if (s == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_lock);
    s_snapshot  = *s;                 /* 结构体整体赋值，一次进入一次退出 */
    s_seq++;
    s_snapshot.seq = s_seq;
    s_has_data  = true;
    portEXIT_CRITICAL(&s_lock);
    return true;
}

/*
 * 读一份一致性快照。返回 false 表示还没发布过任何数据
*/
bool app_data_read(app_snapshot_t *out)
{
    if (out == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_lock);
    bool ok = s_has_data;
    if (ok) {
        *out = s_snapshot;
    }
    portEXIT_CRITICAL(&s_lock);
    return ok;
}

/*
 * 获取数据序列号
 */
uint32_t app_data_seq(void)
{
    portENTER_CRITICAL(&s_lock);
    uint32_t v = s_seq;
    portEXIT_CRITICAL(&s_lock);
    return v;
}
/*
 * 获取配置信息
 */
void app_data_get_cfg(app_cfg_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_cfg;
    portEXIT_CRITICAL(&s_lock);
}
/*
 * 设置配置信息
*/
void app_data_set_cfg(const app_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    s_cfg = *cfg;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "阈值已更新：心率<%ld 血氧>%ld 温度 %.1f~%.1f",
             (long)cfg->heart_max, (long)cfg->spo2_min,
             (double)cfg->temp_min, (double)cfg->temp_max);
}