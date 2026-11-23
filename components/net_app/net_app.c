/* */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#include "sdkconfig.h"
#include "esp_app_desc.h"

#include "app_data.h"
#include "net_app.h"
#include "net_internal.h"

static const char *TAG = "net";

static net_time_cb_t s_time_cb = NULL;

void net_app_set_time_cb(net_time_cb_t cb)
{
    s_time_cb = cb;
}

static void on_time_sync(struct timeval *tv)
{
    (void)tv;
    time_t now = 0;
    struct tm t = { 0 };

    time(&now);
    localtime_r(&now, &t);

    ESP_LOGI(TAG, "SNTP 对时完成：%04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);

    if (s_time_cb != NULL) {
        s_time_cb(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                  t.tm_hour, t.tm_min, t.tm_sec);
    }
}

static void net_time_start(void)
{
    /* 设置时区为东八区 */
    setenv("TZ", "CST-8", 1);
    tzset();

    /* 初始化 SNTP */
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_set_time_sync_notification_cb(on_time_sync);
    esp_sntp_init();
}

/* 把快照拼成物模型格式的 params 段。
 * 无效值一律发 null，不要发 0，不然上位机分不清"没测出来"和"真的是 0"。 */
static int build_params(char *buf, size_t len, const app_snapshot_t *s, int alarm)
{
    char heart[12];
    char spo2[12];
    char temp[16];
    /* 先读阈值配置，判断告警 */
    app_cfg_t cfg;
    app_data_get_cfg(&cfg);

    if (s->hr_ok) {
        (void)snprintf(heart, sizeof(heart), "%ld", (long)s->hr);
    } else {
        (void)snprintf(heart, sizeof(heart), "null");
    }

    if (s->spo2_ok) {
        (void)snprintf(spo2, sizeof(spo2), "%ld", (long)s->spo2);
    } else {
        (void)snprintf(spo2, sizeof(spo2), "null");
    }

    /* 只要传感器读到了有效温度就上报，不再拿"下限"去卡。
     *
     * 原来的逻辑是：低于 temp_min(35℃) 就发 null，当作"探头没贴在人身上"。
     * 问题是这样一来，环境温度、刚上电没捂热、贴着手指但散热快……
     * 全都变成 null，页面上永远看不到温度，反而比"显示一个偏低的值"更没用。
     *
     * 现在的分工：
     *   - 传感器读不到（ntc_temp_is_valid() == false）→ 发 null，这是真故障；
     *   - 读到了就照实发，高低由前端自己判断（前端本来就有 35~38 的显示习惯）；
     *   - temp_min 只用来在本地日志里提示"可能没贴合"，不影响上报。 */
    if (s->temp_ok) {
        (void)snprintf(temp, sizeof(temp), "%.1f", (double)s->temp_c);
    } else {
        (void)snprintf(temp, sizeof(temp), "null");
    }
    /* 用 snprintf 自己拼接成 JSON 字符串，返回写入的字节数（不含结尾 \0） */
    return snprintf(buf, len,
                    "{"
                    "\"temp\":{\"value\":%s},"
                    "\"heart\":{\"value\":%s},"
                    "\"spo2\":{\"value\":%s},"
                    "\"step\":{\"value\":%lu},"
                    "\"tumble\":{\"value\":%u},"
                    "\"fault\":{\"value\":%u},"
                    "\"alarm\":{\"value\":%u}"
                    "}",
                    temp, heart, spo2,
                    (unsigned long)s->steps,
                    (unsigned)s->fall,
                    (unsigned)s->fault,
                    (unsigned)alarm);
}

/* 阈值判断放在这里而不是 app_data：中枢只管"存"，不管"怎么算告警" */
static bool check_alarm(const app_snapshot_t *s, char *why, size_t why_len)
{
    app_cfg_t cfg;
    app_data_get_cfg(&cfg);

    if (s->hr_ok && s->hr > cfg.heart_max) {
        (void)snprintf(why, why_len, "心率 %ld 超过上限 %ld", (long)s->hr, (long)cfg.heart_max);
        return true;
    }
    if (s->spo2_ok && s->spo2 < cfg.spo2_min) {
        (void)snprintf(why, why_len, "血氧 %ld 低于下限 %ld", (long)s->spo2, (long)cfg.spo2_min);
        return true;
    }
    /* 只对"发热"告警。低于下限是"没贴合"，不是低体温 ——
     * 20℃左右是室温，说明传感器没贴在人身上，报警只会误导。 */
    if (s->temp_ok && s->temp_c > cfg.temp_max) {
        (void)snprintf(why, why_len, "体温 %.1f 超过上限 %.1f",
                       (double)s->temp_c, (double)cfg.temp_max);
        return true;
    }
    return false;
}

static int publish_snapshot(const app_snapshot_t *s, char *why, size_t why_len)
{
    char params[256];
    char payload[320];

    if (why != NULL && why_len > 0) {
        why[0] = '\0';
    }
    const int alarm = check_alarm(s, why, why_len) ? 1 : 0;

    int n = build_params(params, sizeof(params), s, alarm);
    if (n <= 0 || n >= (int)sizeof(params)) {
        ESP_LOGE(TAG, "params 拼装失败（缓冲区太小）");
        return -1;
    }

    n = snprintf(payload, sizeof(payload),
                 "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":%s}",
                 (unsigned long)s->seq, params);
    if (n <= 0 || n >= (int)sizeof(payload)) {
        ESP_LOGE(TAG, "payload 拼装失败（缓冲区太小）");
        return -1;
    }

    return mqtt_cli_publish_data(payload);
}

static void uplink_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   last_alarm_log = 0;

    for (;;) {
        app_snapshot_t s;

        if (!app_data_read(&s)) {
            /* 还没人发布过，等下一轮 */
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_NET_UPLINK_PERIOD_MS));
            continue;
        }

        if (mqtt_cli_connected()) {
            char why[64] = {0};
            int mid = publish_snapshot(&s, why, sizeof(why));
            if (mid < 0) {
                ESP_LOGW(TAG, "这一帧没发出去（outbox 满或连接刚断）");
            } else if (s.seq % 15 == 0) {
                /* 每 15 帧打一条，避免日志把串口占满 */
                ESP_LOGI(TAG, "上报 seq=%lu msg_id=%d 心率=%s 血氧=%s 温度=%s",
                         (unsigned long)s.seq, mid,
                         s.hr_ok ? "有效" : "无效",
                         s.spo2_ok ? "有效" : "无效",
                         s.temp_ok ? "有效" : "无效");
            }

            if (why[0] != '\0') {
                uint32_t now = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
                if (now - last_alarm_log >= 10) {   /* 同一类告警 10s 最多一条 */
                    ESP_LOGW(TAG, "阈值告警：%s", why);
                    last_alarm_log = now;
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_NET_UPLINK_PERIOD_MS));
    }
}


/* 返回当前网络状态：离线 / WiFi 通 / MQTT 通*/
net_state_t net_app_state(void)
{
    if (!wifi_mgr_connected()) {
        return NET_STATE_OFFLINE;
    }
    return mqtt_cli_connected() ? NET_STATE_MQTT : NET_STATE_WIFI;
}

/*返回 WiFi 连接状态 */
bool net_app_wifi_connected(void)
{
    return wifi_mgr_connected();
}
/*返回 MQTT 连接状态 */
bool net_app_mqtt_connected(void)
{
    return mqtt_cli_connected();
}

/* 立即上报一次快照（阻塞，返回值是 MQTT publish 的 mid）；
 * 失败返回 -1（MQTT 没连上或 app_data 还没发布过数据） */
int net_app_publish_once(void)
{
    app_snapshot_t s;
    if (!app_data_read(&s) || !mqtt_cli_connected()) {
        return -1;
    }
    return publish_snapshot(&s, NULL, 0);
}

/* 界面上点"开始升级"时调用：问服务器有没有新固件。
 * 服务器在 <prefix>/ota 上回 {"url":...,"version":...}，
 * 有 url 就自动开始下载，设备不需要预先知道任何 IP 或 URL。 */
int net_app_request_ota_check(void)
{
    return mqtt_cli_request_ota_check(esp_app_get_description()->version);
}

/* 等 WiFi 拿到 IP 之后再启动 MQTT。
 *
 * 必须等 WiFi 就绪：esp-mqtt 在 esp_mqtt_client_start() 里立刻做一次
 * getaddrinfo("broker.emqx.io")。开机时 DHCP 还没完成、DNS 服务器还没下发给
 * lwIP，这一步必然失败，串口就是那行：
 *     E esp-tls: couldn't get hostname for :broker.emqx.io: getaddrinfo() returns 202
 * 然后 esp-mqtt 要等 reconnect_timeout 才重试，白白延迟十几秒才连上。
 *
 * 用独立任务等，不阻塞 app_main，屏幕和采集任务要立刻跑起来。 */
static void net_boot_task(void *arg)
{
    (void)arg;
    const int max_wait = 300;   /* 300 × 100ms = 30 秒 */
    int waited = 0;
    while (!wifi_mgr_connected() && waited < max_wait) {
        vTaskDelay(pdMS_TO_TICKS(100));
        waited++;
    }

    if (wifi_mgr_connected()) {
        ESP_LOGI(TAG, "WiFi 已就绪（等了 %d ms），启动 MQTT", waited * 100);
    } else {
        /* 连不上也照样启动：esp-mqtt 会自己按 reconnect_timeout_ms 重试，
         * 比在这里死等更有意义 */
        ESP_LOGW(TAG, "WiFi 30 秒内没就绪，仍然启动 MQTT（它会自己重试）");
    }

    esp_err_t err = mqtt_cli_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mqtt_cli_start 失败: %s", esp_err_to_name(err));
    }

    /* SNTP：没网时它自己按 60 秒重试，连上后第一次成功就回调 */
    net_time_start();

    vTaskDelete(NULL);
}

/* 启动网络层：NVS、WiFi、MQTT、SNTP、上报任务 */
esp_err_t net_app_start(void)
{
    /* NVS 是 WiFi 驱动存校准数据的地方，必须最先初始化 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 需要重建");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init");

    ESP_RETURN_ON_ERROR(wifi_mgr_start(), TAG, "wifi_mgr_start");
    /* MQTT / SNTP 交给 net_boot_task：等拿到 IP 再启动，避免开机时 DNS 失败 */
    if (xTaskCreate(net_boot_task, "net_boot", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(uplink_task, "net_up", CONFIG_NET_UPLINK_TASK_STACK, NULL,
                    CONFIG_NET_UPLINK_TASK_PRIO, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "网络层启动完成，主题前缀 %s，周期 %d ms",
             CONFIG_NET_TOPIC_PREFIX, CONFIG_NET_UPLINK_PERIOD_MS);
    return ESP_OK;
}