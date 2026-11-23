/*
 * mqtt_cli.c，esp-mqtt 客户端
 *
 * 和 AT 指令版最关键的三处不同：
 *
 * 1. esp_mqtt_client_publish() 的返回值是"入队序号"，不是"发送成功"。
 *    真正的发送在 esp-mqtt 自己的任务里做。所以：
 *      - 返回 <0 只说明"当前发不出去"（没连上 / outbox 满了）
 *      - 返回 >=0 只说明"排进队列了"
 *    要确认对方收到，靠 QoS1 的 PUBACK，而不是这个返回值。
 *
 * 2. 回调跑在 mqtt 任务上下文里，里面不能阻塞、不能做长耗时的事，
 *    否则心跳（keepalive）发不出去，broker 会把你踢下线。
 *    这里只做 JSON 解析 + 阈值落库 + 一次 ack 入队，全是微秒级操作。
 *
 * 3. MQTT_EVENT_DATA 的数据**不以 '\0' 结尾**，而且可能分片
 *    （total_data_len != data_len）。所以必须用 cJSON_ParseWithLength()，
 *    不能用 cJSON_Parse()。分片时这里直接丢弃，命令都是几十字节，
 *    真出现分片说明对面发的是别的东西。
 */

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "sdkconfig.h"

#include "app_data.h"
#include "net_internal.h"
#include "ota_app.h"

static const char *TAG = "mqtt";

/* MQTT 客户端句柄和连接状态 */
static esp_mqtt_client_handle_t s_client;
static volatile bool            s_connected = false;

/* 主题：前缀在 menuconfig 里配，拼一次就够了，别每次 publish 都 snprintf */
static char s_topic_data[96];
static char s_topic_set[96];
static char s_topic_ack[96];
static char s_topic_status[96];
/* OTA 自助升级：设备发 <prefix>/ota_req 问"有没有新版本"，
 * 服务器回 <prefix>/ota {"url":"...","version":"..."}。
 * url 为空字符串 = 已是最新，不用升级。 */
static char s_topic_ota_req[96];
static char s_topic_ota[96];

static void build_topics(void)
{
    (void)snprintf(s_topic_data,   sizeof(s_topic_data),   "%s/data",   CONFIG_NET_TOPIC_PREFIX);
    (void)snprintf(s_topic_set,    sizeof(s_topic_set),    "%s/set",    CONFIG_NET_TOPIC_PREFIX);
    (void)snprintf(s_topic_ack,    sizeof(s_topic_ack),    "%s/ack",    CONFIG_NET_TOPIC_PREFIX);
    (void)snprintf(s_topic_status, sizeof(s_topic_status), "%s/status", CONFIG_NET_TOPIC_PREFIX);
    (void)snprintf(s_topic_ota_req, sizeof(s_topic_ota_req), "%s/ota_req", CONFIG_NET_TOPIC_PREFIX);
    (void)snprintf(s_topic_ota,     sizeof(s_topic_ota),     "%s/ota",     CONFIG_NET_TOPIC_PREFIX);
}


/* 把一个 JSON 节点当数字取出来，兼容三种写法： */
static bool node_to_number(const cJSON *item, double *out)
{
    if (item == NULL) {
        return false;
    }
    if (cJSON_IsNumber(item)) {
        *out = item->valuedouble;
        return true;
    }
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        char *end = NULL;
        double v = strtod(item->valuestring, &end);
        if (end != item->valuestring) {
            *out = v;
            return true;
        }
        return false;
    }
    if (cJSON_IsObject(item)) {
        return node_to_number(cJSON_GetObjectItemCaseSensitive(item, "value"), out);
    }
    return false;
}

/* 把 "key=value" 落到配置里。返回 false = 不认识的键 */
static bool apply_threshold(app_cfg_t *cfg, const char *key, double v)
{
    if (strcmp(key, "heart_max") == 0) {
        cfg->heart_max = (int32_t)(v + 0.5);
        return true;
    }
    /* 上位机发的键名是 spo2_max，它的语义其实是"下限"，
     * 所以这里两个名字都认，落到同一个字段 */
    if (strcmp(key, "spo2_min") == 0 || strcmp(key, "spo2_max") == 0) {
        cfg->spo2_min = (int32_t)(v + 0.5);
        return true;
    }
    if (strcmp(key, "temp_max") == 0) {
        cfg->temp_max = (float)v;
        return true;
    }
    if (strcmp(key, "temp_min") == 0) {
        cfg->temp_min = (float)v;
        return true;
    }
    return false;
}

/* 处理下行命令 JSON，更新阈值 / 工作模式 / 重启。内部会发 ack。
 * 失败的 JSON 会被静默丢弃（不发 ack），因为上位机会重发。 */
static void handle_command(const char *json, int len)
{
    cJSON *root = cJSON_ParseWithLength(json, (size_t)len);
    if (root == NULL) {
        ESP_LOGW(TAG, "下行不是合法 JSON，丢弃");
        return;
    }

    /* 兼容两种写法：{"params":{...}} 和 {"heart_max":90} */
    const cJSON *params = cJSON_GetObjectItemCaseSensitive(root, "params");
    if (!cJSON_IsObject(params)) {
        params = root;
    }

    /* 远程重启：演示用，正式产品要加鉴权 */
    const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(params, "cmd");
    if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "reboot") == 0) {
        ESP_LOGW(TAG, "收到 reboot 命令，3 秒后重启");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

    /* 远程升级：{"cmd":"ota","url":"http://192.168.1.10:3000/api/ota/firmware.bin"} */
    if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "ota") == 0) {
        const cJSON *jurl = cJSON_GetObjectItemCaseSensitive(params, "url");
        if (!cJSON_IsString(jurl) || jurl->valuestring == NULL) {
            ESP_LOGW(TAG, "OTA 命令缺少 url 字段");
            (void)mqtt_cli_publish_ack("{\"code\":400,\"msg\":\"ota missing url\"}");
        } else {
            esp_err_t oerr = ota_app_start_async(jurl->valuestring);
            if (oerr == ESP_OK) {
                ESP_LOGW(TAG, "OTA 已开始：%s", jurl->valuestring);
                (void)mqtt_cli_publish_ack("{\"code\":200,\"cmd\":\"ota\",\"status\":\"started\"}");
            } else {
                (void)mqtt_cli_publish_ack("{\"code\":409,\"msg\":\"ota busy or bad url\"}");
            }
        }
        cJSON_Delete(root);
        return;
    }

    app_cfg_t cfg;
    app_data_get_cfg(&cfg);
    bool changed = false;

    const cJSON *jkey = cJSON_GetObjectItemCaseSensitive(params, "key");
    const cJSON *jval = cJSON_GetObjectItemCaseSensitive(params, "value");
    if (cJSON_IsString(jkey) && jkey->valuestring != NULL) {
        double v = 0;
        if (node_to_number(jval, &v)) {
            if (apply_threshold(&cfg, jkey->valuestring, v)) {
                ESP_LOGI(TAG, "阈值更新：%s = %g", jkey->valuestring, v);
                changed = true;
            } else {
                ESP_LOGW(TAG, "不认识的阈值键：%s", jkey->valuestring);
            }
        }
    }

    double mode_v = 0;
    if (node_to_number(cJSON_GetObjectItemCaseSensitive(params, "mode"), &mode_v)) {
        cfg.mode = (uint8_t)mode_v;
        ESP_LOGI(TAG, "工作模式切换为 %u", (unsigned)cfg.mode);
        changed = true;
    }

    /* ---- 形态 C：{"heart_max":90,"spo2_max":88,...}（扁平写法，
     *      也覆盖 {"params":{"heart_max":90}}） ---- */
    const cJSON *it = NULL;
    cJSON_ArrayForEach(it, params) {
        const char *k = it->string;
        double v = 0;

        if (k == NULL || !node_to_number(it, &v)) {
            continue;
        }
        /* 上面两种形态已经处理过的字段，别重复应用 */
        if (strcmp(k, "key") == 0 || strcmp(k, "value") == 0 ||
            strcmp(k, "mode") == 0 || strcmp(k, "cmd") == 0) {
            continue;
        }
        if (apply_threshold(&cfg, k, v)) {
            ESP_LOGI(TAG, "阈值更新：%s = %g", k, v);
            changed = true;
        }
    }

    if (changed) {
        app_data_set_cfg(&cfg);
    } else {
        ESP_LOGW(TAG, "下行里没有认识的字段");
    }

    /* 回 ack：把"现在生效的完整配置"发回去，
     * 这样上位机不用猜哪条命令生效了、哪条被忽略了 */
    char ack[224];
    int n = snprintf(ack, sizeof(ack),
                     "{\"code\":200,\"params\":{\"mode\":%u,\"heart_max\":%ld,"
                     "\"spo2_min\":%ld,\"temp_max\":%.1f,\"temp_min\":%.1f}}",
                     (unsigned)cfg.mode,
                     (long)cfg.heart_max, (long)cfg.spo2_min,
                     (double)cfg.temp_max, (double)cfg.temp_min);
    if (n > 0 && n < (int)sizeof(ack)) {
        (void)mqtt_cli_publish_ack(ack);
    }

    cJSON_Delete(root);
}


/* 服务器回了"有没有新固件"，payload 形如： */
static void handle_ota_reply(const char *data, int len)
{
    ESP_LOGI(TAG, "收到升级查询结果：%.*s", len, data);

    cJSON *root = cJSON_ParseWithLength(data, (size_t)len);
    if (root == NULL) {
        ESP_LOGE(TAG, "升级查询结果不是合法 JSON，忽略");
        return;
    }

    const cJSON *ju = cJSON_GetObjectItemCaseSensitive(root, "url");
    const cJSON *jv = cJSON_GetObjectItemCaseSensitive(root, "version");
    const char *url = cJSON_IsString(ju) ? ju->valuestring : NULL;
    const char *ver = cJSON_IsString(jv) ? jv->valuestring : "?";

    if (url == NULL || url[0] == '\0') {
        ESP_LOGI(TAG, "服务器版本 %s，设备已是最新，不升级", ver);
    } else if (ota_app_in_progress()) {
        ESP_LOGW(TAG, "已经在升级中，忽略这次查询结果");
    } else {
        ESP_LOGI(TAG, "发现新固件 %s，开始下载", ver);
        ota_app_start_async(url);
    }
    cJSON_Delete(root);
}

static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "已连接 broker，订阅 %s 与 %s", s_topic_set, s_topic_ota);
        esp_mqtt_client_subscribe(s_client, s_topic_set, 1);
        esp_mqtt_client_subscribe(s_client, s_topic_ota, 1);
        /* 上线通告，retain=1 让后来连上的订阅者也能看到当前状态 */
        esp_mqtt_client_publish(s_client, s_topic_status, "{\"online\":true}", 0, 1, 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "与 broker 断开，esp-mqtt 会自动重连");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "订阅成功，msg_id=%d", e->msg_id);
        break;

    case MQTT_EVENT_DATA: {
        if (e->total_data_len != e->data_len) {
            ESP_LOGW(TAG, "收到分片消息（%d/%d），本轮丢弃",
                     e->data_len, e->total_data_len);
            break;
        }
        /* 只处理 set 主题；topic 同样不以 '\0' 结尾 */
        if (e->topic_len == (int)strlen(s_topic_set) &&
            strncmp(e->topic, s_topic_set, (size_t)e->topic_len) == 0) {
            ESP_LOGI(TAG, "收到命令：%.*s", e->data_len, e->data);
            handle_command(e->data, e->data_len);
        } else if (e->topic_len == (int)strlen(s_topic_ota) &&
                   strncmp(e->topic, s_topic_ota, (size_t)e->topic_len) == 0) {
            handle_ota_reply(e->data, e->data_len);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT 错误（看 esp-tls 的日志判断是网络还是证书问题）");
        break;

    default:
        break;
    }
}


/* 问服务器"有没有新固件"。返回值是 msg_id，0/-1 表示当前发不出去（离线） */
int mqtt_cli_request_ota_check(const char *current_version)
{
    if (!s_connected) {
        return -1;
    }
    char payload[128];
    (void)snprintf(payload, sizeof(payload),
                   "{\"cmd\":\"ota_check\",\"current\":\"%s\"}",
                   (current_version != NULL) ? current_version : "0.0.0");
    ESP_LOGI(TAG, "查询新固件，当前版本 %s", current_version);
    return esp_mqtt_client_publish(s_client, s_topic_ota_req, payload, 0, 1, 0);
}
bool mqtt_cli_connected(void)
{
    return s_connected;
}

/* 发布数据（阻塞，返回值是 msg_id） */
int mqtt_cli_publish_data(const char *payload)
{
    if (!s_connected || payload == NULL) {
        return -1;
    }
    return esp_mqtt_client_publish(s_client, s_topic_data, payload, 0,
                                   CONFIG_NET_MQTT_QOS, 0);
}

/* 发布确认（阻塞，返回值是 msg_id） */
int mqtt_cli_publish_ack(const char *payload)
{
    if (!s_connected || payload == NULL) {
        return -1;
    }
    return esp_mqtt_client_publish(s_client, s_topic_ack, payload, 0,
                                   CONFIG_NET_MQTT_QOS, 0);
}

/* 启动 MQTT 客户端（阻塞，内部会创建任务）；
 * 失败返回 esp_err_t，成功返回 ESP_OK。
 */
esp_err_t mqtt_cli_start(void)
{
    build_topics();

    esp_mqtt_client_config_t cfg = {0};
    cfg.broker.address.uri      = CONFIG_NET_MQTT_URI;
    cfg.credentials.client_id   = CONFIG_NET_MQTT_CLIENT_ID;

    if (CONFIG_NET_MQTT_USERNAME[0] != '\0') {
        cfg.credentials.username = CONFIG_NET_MQTT_USERNAME;
        cfg.credentials.authentication.password = CONFIG_NET_MQTT_PASSWORD;
    }

    cfg.session.keepalive   = 60;     /* 60s 没心跳 broker 就踢你 */
    cfg.session.last_will.topic  = s_topic_status;
    cfg.session.last_will.msg    = "{\"online\":false}";
    cfg.session.last_will.qos    = 1;
    cfg.session.last_will.retain = 1; /* 掉线时 broker 会替你发这条"遗嘱" */

    cfg.network.reconnect_timeout_ms = 5000;

    /* 下面两个 buffer 是 esp-mqtt 内部的环形队列，默认 1024/4096 对于 TLS/JSON 的数据量不够用 */
    cfg.buffer.out_size = 4096;
    cfg.buffer.size     = 1024;

    cfg.task.stack_size = 6144;       /* 默认 4096 在加了 TLS/JSON 后不够用 */

    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* 注册事件回调，esp-mqtt 内部会创建一个任务来跑回调 */
    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                                       mqtt_event_handler, NULL),
                        TAG, "register_event");
    /* 启动客户端，阻塞直到连接上 broker 或超时 */
    ESP_RETURN_ON_ERROR(esp_mqtt_client_start(s_client), TAG, "client_start");

    ESP_LOGI(TAG, "客户端启动：%s（client_id=%s）",
             CONFIG_NET_MQTT_URI, CONFIG_NET_MQTT_CLIENT_ID);
    return ESP_OK;
}