/*
 * net_app 组件内部，三个 .c 之间互相调用用，不放进 include/。
 * 上层（main）只该用 net_app.h。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t wifi_mgr_start(void);      /* 阻塞，内部建任务 */
bool      wifi_mgr_connected(void);  /* WiFi 是否连上，不看 MQTT */
int8_t    wifi_mgr_rssi(void);

esp_err_t mqtt_cli_start(void);      /* 阻塞，内部建任务 */
bool      mqtt_cli_connected(void);  /* MQTT 是否连上，不看 WiFi */
int       mqtt_cli_publish_data(const char *payload);  /* 返回 msg_id */
int       mqtt_cli_publish_ack(const char *payload);

/* 发 <prefix>/ota_req 查询新固件；返回 msg_id，-1 表示当前离线 */
int       mqtt_cli_request_ota_check(const char *current_version);