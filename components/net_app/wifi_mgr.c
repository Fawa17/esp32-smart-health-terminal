/* */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "sdkconfig.h"
#include "net_internal.h"

static const char *TAG = "wifi";

#define WIFI_BIT_GOT_IP  BIT0
#define WIFI_BIT_LOST    BIT1

#define RECONNECT_MIN_MS 1000
#define RECONNECT_MAX_MS 30000

static EventGroupHandle_t s_eg;
static bool               s_connected = false;

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;

    switch (id) {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "STA 启动，开始连接 \"%s\"", CONFIG_NET_WIFI_SSID);
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_DISCONNECTED: {
        const wifi_event_sta_disconnected_t *d = (const wifi_event_sta_disconnected_t *)data;
        s_connected = false;
        xEventGroupClearBits(s_eg, WIFI_BIT_GOT_IP);
        xEventGroupSetBits(s_eg, WIFI_BIT_LOST);
        ESP_LOGW(TAG, "断开（reason=%d），交给重连任务处理", d ? d->reason : -1);
        break;
    }

    default:
        break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;

    if (id != IP_EVENT_STA_GOT_IP) {
        return;
    }

    const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)data;
    s_connected = true;
    xEventGroupClearBits(s_eg, WIFI_BIT_LOST);
    xEventGroupSetBits(s_eg, WIFI_BIT_GOT_IP);

    ESP_LOGI(TAG, "已连接，IP=" IPSTR "，信号 %d dBm",
             IP2STR(&ev->ip_info.ip), wifi_mgr_rssi());
}

static void reconnect_task(void *arg)
{
    (void)arg;
    uint32_t wait_ms = RECONNECT_MIN_MS;

    for (;;) {
        /* 等到"断了"这个事件，取走后清除，避免重复处理 */
        xEventGroupWaitBits(s_eg, WIFI_BIT_LOST, pdTRUE, pdFALSE, portMAX_DELAY);

        if (s_connected) {
            wait_ms = RECONNECT_MIN_MS;
            continue;
        }

        ESP_LOGW(TAG, "%lu ms 后重连", (unsigned long)wait_ms);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));

        if (s_connected) {                  /* 等待期间自己连上了 */
            wait_ms = RECONNECT_MIN_MS;
            continue;
        }

        esp_wifi_connect();

        wait_ms *= 2;
        if (wait_ms > RECONNECT_MAX_MS) {
            wait_ms = RECONNECT_MAX_MS;
        }
    }
}

bool wifi_mgr_connected(void)
{
    return s_connected;
}

int8_t wifi_mgr_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}

esp_err_t wifi_mgr_start(void)
{
    s_eg = xEventGroupCreate();
    if (s_eg == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif_init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event_loop");
    (void)esp_netif_create_default_wifi_sta();     /* 内部会建 DHCP 客户端 */

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi_init");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL),
                        TAG, "reg wifi event");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_event, NULL, NULL),
                        TAG, "reg ip event");

    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, CONFIG_NET_WIFI_SSID, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, CONFIG_NET_WIFI_PASSWORD, sizeof(cfg.sta.password) - 1);

    /* 有密码就要求 WPA2，没密码走开放网络。门限设低了会连上加密网络却拿不到 IP */
    cfg.sta.threshold.authmode = (CONFIG_NET_WIFI_PASSWORD[0] != '\0')
                                 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    cfg.sta.pmf_cfg.capable  = true;
    cfg.sta.pmf_cfg.required = false;   /* 有些老路由器不支持 PMF，不强制 */

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set_mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &cfg), TAG, "set_config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi_start");

    if (xTaskCreate(reconnect_task, "wifi_re", 3072, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "初始化完成（省电模式关闭，保证上报时延）");
    return ESP_OK;
}