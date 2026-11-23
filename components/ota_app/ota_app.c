/**
 * @file  ota_app.c
 * @brief OTA 下载 / 写入 / 校验 / 切换 + 回滚保护
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_http_client.h"

#include "ota_app.h"

static const char *TAG = "ota";

/* 下载缓冲区：4KB。取值理由——太小则 HTTP 读次数多、flash 写次数多；
 * 太大则占内部 RAM（这个缓冲区必须能 ping-pong 给 flash 写，最好在内部 RAM）。 */
#define OTA_BUF_SIZE 4096

static volatile bool s_in_progress = false;
static volatile bool s_pending     = false;
static ota_ready_check_t s_ready_check = NULL;
/**
 *  @brief 注册"新固件是否已经证明可用"的回调
 */
void ota_app_set_ready_check(ota_ready_check_t fn)
{
    s_ready_check = fn;
}
/**
 *  @brief 当前固件是否处于"待验证"状态（新固件的第一次启动）
 */
bool ota_app_is_pending_verify(void)
{
    return s_pending;
}
/**
 *  @brief 是否正在下载（防止重复触发）
 */
bool ota_app_in_progress(void)
{
    return s_in_progress;
}

/**
 * @brief 确认当前固件可用，解除回滚保护
 *
 * 这个函数会在"新固件第一次启动"时被 ota_task 调用，也可以在应用里手动调用。
 * 只要调用了这个函数，bootloader 就不会再回滚到旧固件了。
 */
esp_err_t ota_app_confirm(void)
{
    //确认当前固件可用，解除回滚保护
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        s_pending = false;
        ESP_LOGI(TAG, "已确认当前固件可用，回滚保护解除");
    } else {
        ESP_LOGE(TAG, "确认固件失败: %s", esp_err_to_name(err));
    }
    return err;
}

/**
 *  @brief 确认任务：如果在规定时间内没证明自己能工作，就回滚
 */
static void confirm_task(void *arg)
{
    (void)arg;
    const int timeout_s = CONFIG_OTA_CONFIRM_TIMEOUT_S;

    ESP_LOGW(TAG, "新固件处于 PENDING_VERIFY：%d 秒内必须证明自己能工作，否则自动回滚", timeout_s);

    for (int i = 0; i < timeout_s; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (s_ready_check != NULL && s_ready_check()) {
            (void)ota_app_confirm();
            /* 把新固件"转正"，写一份日志方便事后确认是哪次升级生效了 */
            const esp_app_desc_t *d = esp_app_get_description();
            ESP_LOGI(TAG, "当前运行固件：%s v%s (编译于 %s %s)",
                     d->project_name, d->version, d->date, d->time);
            vTaskDelete(NULL);
            return;
        }
    }

    ESP_LOGE(TAG, "%d 秒内没等到联网成功，判定新固件不可用 → 回滚到上一版并重启", timeout_s);
    esp_ota_mark_app_invalid_rollback_and_reboot();

    /* 正常情况下上面这个函数不会返回 */
    vTaskDelete(NULL);
}

/**
 *  @brief 启动时调用一次
 */
/* 分区状态的可读名字。启动时打出来，一眼看出上次升级到底发生了什么，
 * 排查"升级完还是旧版本"主要看这一行。 */
static const char *state_name(esp_ota_img_states_t s)
{
    switch (s) {
    case ESP_OTA_IMG_NEW:            return "NEW(刚写入,未启动过)";
    case ESP_OTA_IMG_PENDING_VERIFY: return "PENDING_VERIFY(待确认)";
    case ESP_OTA_IMG_VALID:          return "VALID(已确认)";
    case ESP_OTA_IMG_INVALID:        return "INVALID(曾被判无效)";
    case ESP_OTA_IMG_ABORTED:        return "ABORTED(写入未完成)";
    default:                         return "UNDEFINED(从未升级过)";
    }
}

esp_err_t ota_app_init(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        ESP_LOGE(TAG, "取不到当前运行分区");
        return ESP_FAIL;
    }
 
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_err_t err = esp_ota_get_state_partition(running, &state);
    if (err != ESP_OK) {
        /* 只有开了 rollback 且分区是 ota_x 才会有状态，失败不算致命 */
        ESP_LOGD(TAG, "读取分区状态失败: %s", esp_err_to_name(err));
    }

    const esp_app_desc_t *d = esp_app_get_description();
    ESP_LOGI(TAG, "运行分区 %s，固件 %s v%s，状态 %s",
             running->label, d->project_name, d->version, state_name(state));

    /* 顺便打出"下次会写到哪个槽、那个槽现在什么状态"。
     * 排查"升级完还是旧版本"时，这两行就是判据。 */
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (next != NULL) {
        esp_ota_img_states_t ns = ESP_OTA_IMG_UNDEFINED;
        esp_ota_get_state_partition(next, &ns);
        ESP_LOGI(TAG, "下次升级写入 %s（状态 %s）", next->label, state_name(ns));
    }

    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        s_pending = true;
        /* 优先级 5：比采样任务低，别抢 CPU；栈 4KB 够用（里面只是 sleep + 一次 flash 写） */
        if (xTaskCreate(confirm_task, "ota_confirm",
                        CONFIG_OTA_CONFIRM_TASK_STACK, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "创建确认任务失败");
            return ESP_ERR_NO_MEM;
        }
    } else if (state == ESP_OTA_IMG_VALID) {
        ESP_LOGI(TAG, "固件已是确认状态，无需回滚保护");
    } else if (state == ESP_OTA_IMG_INVALID) {
        ESP_LOGW(TAG, "这个槽位曾被判无效：上次升级后 60 秒内没联网，已经自动回滚过一次");
    } else if (state == ESP_OTA_IMG_ABORTED) {
        ESP_LOGW(TAG, "这个槽位的固件是残缺的：上次写入被中断（掉电或断网）");
    }

    return ESP_OK;
}

/**
 *  @brief OTA 更新任务
 */
static void ota_task(void *arg)
{
    char *url = (char *)arg;
    esp_err_t ret = ESP_FAIL;
    esp_ota_handle_t update = 0;
    uint8_t *buf = NULL;
    esp_http_client_handle_t http = NULL;
    bool ota_begun = false;

    ESP_LOGW(TAG, "开始 OTA：%s", url);

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        ESP_LOGE(TAG, "没有可用的升级分区（分区表里需要 ota_0 / ota_1）");
        goto cleanup;
    }
    ESP_LOGI(TAG, "目标分区 %s @0x%lx 大小 %lu 字节",
             target->label, (unsigned long)target->address, (unsigned long)target->size);

    buf = malloc(OTA_BUF_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "下载缓冲区分配失败");
        goto cleanup;
    }
    //初始化 HTTP 客户端
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = CONFIG_OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
        .buffer_size = 2048,
    };
    http = esp_http_client_init(&cfg);
    if (http == NULL) {
        ESP_LOGE(TAG, "HTTP 客户端初始化失败（URL 格式对吗？）");
        goto cleanup;
    }
    esp_http_client_set_header(http, "User-Agent", "esp32s3-smarthome-ota/1.0");

    if (esp_http_client_open(http, 0) != ESP_OK) {
        ESP_LOGE(TAG, "连接固件服务器失败：检查 URL、WiFi、服务器是否在跑");
        goto cleanup;
    }

    int64_t content_len = esp_http_client_fetch_headers(http);
    int status = esp_http_client_get_status_code(http);
    ESP_LOGI(TAG, "HTTP %d，Content-Length=%lld", status, (long long)content_len);

    if (status != 200) {
        ESP_LOGE(TAG, "服务器返回 %d（不是 200），放弃升级", status);
        goto cleanup;
    }

    /* content_len 可能是 -1（chunked）或 0，那种情况只能靠 esp_ota_end 校验 */
    esp_err_t err = esp_ota_begin(target,
                                 (content_len > 0) ? (size_t)content_len : OTA_SIZE_UNKNOWN,
                                 &update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin 失败: %s", esp_err_to_name(err));
        goto cleanup;
    }
    ota_begun = true;

    size_t written = 0;
    size_t next_log = 64 * 1024;
    while (true) {
        int r = esp_http_client_read(http, (char *)buf, OTA_BUF_SIZE);
        if (r == 0) {
            break;                       /* 正常结束 */
        }
        if (r < 0) {
            ESP_LOGE(TAG, "读取失败，中止（已写 %u 字节）", (unsigned)written);
            goto cleanup;
        }
        err = esp_ota_write(update, buf, (size_t)r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ota write 失败 %s，已写 %u B", esp_err_to_name(err), (unsigned)written);
            goto cleanup;
        }
        written += (size_t)r;

        if (written >= next_log) {
            if (content_len > 0) {
                ESP_LOGI(TAG, "已写 %u / %lld 字节（%d%%）",
                         (unsigned)written, (long long)content_len,
                         (int)(written * 100 / (size_t)content_len));
            } else {
                ESP_LOGI(TAG, "已写 %u 字节", (unsigned)written);
            }
            next_log += 64 * 1024;
        }
    }

    /* 长度对不上 = 服务器中途断了，写进去的是半张固件 */
    if (content_len > 0 && written != (size_t)content_len) {
        ESP_LOGE(TAG, "长度不符：期望 %lld，实际 %u，丢弃",
                 (long long)content_len, (unsigned)written);
        goto cleanup;
    }

    /* esp_ota_end 会校验镜像头里的 SHA256，这一步才是"固件完整性"的真正保证 */
    err = esp_ota_end(update);
    ota_begun = false;                   /* 无论成败 handle 都已释放 */
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "镜像校验失败（不是合法的 app bin，或者传输中损坏）");
        } else {
            ESP_LOGE(TAG, "esp_ota_end 失败: %s", esp_err_to_name(err));
        }
        goto cleanup;
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "切换启动分区失败: %s", esp_err_to_name(err));
        goto cleanup;
    }

    ESP_LOGW(TAG, "升级完成，%u 字节已写入 %s，3 秒后重启进入新固件",
             (unsigned)written, target->label);
    ret = ESP_OK;
//  @brief 清理资源
cleanup:
    if (ota_begun) {
        esp_ota_abort(update);           /* 释放 ota handle，否则下次 begin 会失败 */
        ESP_LOGW(TAG, "本次升级已中止");
    }
    if (http != NULL) {
        esp_http_client_close(http);
        esp_http_client_cleanup(http);
    }
    free(buf);

    s_in_progress = false;
    free(url);

    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    ESP_LOGE(TAG, "升级失败，保持旧固件继续运行");

    vTaskDelete(NULL);
}
/**
 *  @brief  异步开始升级，立即返回
 */
esp_err_t ota_app_start_async(const char *url)
{
    if (url == NULL || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_in_progress) {
        ESP_LOGW(TAG, "已有升级在进行，忽略本次请求");
        return ESP_ERR_INVALID_STATE;
    }

    /* 自己复制一份 URL：调用方（MQTT 回调）的缓冲区随后就会被回收 */
    char *copy = strdup(url);
    if (copy == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_in_progress = true;
    /* 栈 8KB：里面有 esp_http_client + esp_ota_write 的调用链，TLS 时还要更多 */
    if (xTaskCreate(ota_task, "ota", CONFIG_OTA_TASK_STACK, copy, 5, NULL) != pdPASS) {
        s_in_progress = false;
        free(copy);
        ESP_LOGE(TAG, "创建升级任务失败");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}