/**
 * @file  ota_app.h
 * @brief 固件空中升级（OTA）与回滚保护
 *
 * 自实现下载流程：esp_http_client 流式下载，边下边 esp_ota_write（4KB 一块），
 * 写入长度与 Content-Length 比对，结束后由 esp_ota_end 校验镜像 SHA256。
 *
 * 回滚：新固件首次启动为 PENDING_VERIFY，60 秒内必须连上 MQTT 自证可用，
 * 否则标记无效并退回旧固件。
 */
#ifndef OTA_APP_H
#define OTA_APP_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 注册"新固件已证明可用"的回调，每秒调用一次，必须非阻塞 */
typedef bool (*ota_ready_check_t)(void);
void ota_app_set_ready_check(ota_ready_check_t fn);

/** @brief 启动时调用一次，需在联网模块启动之后 */
esp_err_t ota_app_init(void);

/** @brief 当前固件是否处于待验证状态（新固件首次启动） */
bool ota_app_is_pending_verify(void);

/** @brief 是否正在下载（防止重复触发） */
bool ota_app_in_progress(void);

/**
 * @brief 异步开始升级，立即返回
 *
 * @param url 固件地址，支持 http:// 和 https://
 * @return ESP_OK 已开始；ESP_ERR_INVALID_STATE 正在升级中
 *
 * url 必须指向整个 app 分区的 bin（build 目录里的 sample_project.bin），
 * 不是 bootloader，也不是分区表。
 */
esp_err_t ota_app_start_async(const char *url);

/** @brief 手动确认当前固件可用（取消回滚） */
esp_err_t ota_app_confirm(void);

#ifdef __cplusplus
}
#endif

#endif /* OTA_APP_H */