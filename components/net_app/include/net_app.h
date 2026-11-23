/*
 * net_app.h，网络层（WiFi STA + MQTT 客户端 + 周期上报）
 *
 * 和原 STM32 方案的对应关系：
 *   原方案：STM32 --UART-- ESP8266 --AT 指令-- WiFi --MQTT--> broker.emqx.io
 *   现方案：ESP32-S3 内部协议栈 --------------- WiFi --MQTT--> broker.emqx.io
 *
 * 去掉的不只是那 380 行 AT 指令，还有三层"状态同步"问题：
 *   1. AT 指令是文本协议，每条都要等 "OK"/"ERROR"，超时时间靠猜
 *   2. 断线后 ESP8266 的状态和 STM32 的认知会不一致（最典型：以为连着其实早断了）
 *   3. 每发一帧要两次往返（AT+MQTTPUBRAW 报长度 → 再发裸数据），
 *      中间被别的任务插进来就发一半
 * 这三件事在 esp-mqtt 里都不存在：API 是函数调用，连接状态由库自己维护，
 * 发送是入队（outbox），断了会自己重连并把队列里的消息补发。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NET_STATE_OFFLINE = 0,   /* 没连上 WiFi */
    NET_STATE_WIFI    = 1,   /* WiFi 通了，MQTT 还没连上 */
    NET_STATE_MQTT    = 2,   /* MQTT 连上 ，全部就绪，可以上报 */
} net_state_t;

/**
 * @brief SNTP 对时成功后的回调（已是东八区本地时间）
 *
 * net_app 不认识 DS1302，用回调把"网络"和"RTC"解耦：
 * 上层在这个回调里调 ds1302_write_time() 即可。
 * 回调在 lwIP 的 SNTP 任务上下文里执行，别在里面做耗时操作。
 */
typedef void (*net_time_cb_t)(int year, int mon, int day, int hour, int min, int sec);
void net_app_set_time_cb(net_time_cb_t cb);

/**
 * @brief 启动网络：NVS → WiFi STA → MQTT → SNTP 对时 → 上报任务
 *
 * 不阻塞等连接成功：WiFi 断了会按 1s/2s/4s…30s 退避重连，
 * MQTT 由 esp-mqtt 自己的重连机制处理。想等就轮询 net_app_state()。
 */
esp_err_t net_app_start(void);

/** @brief 当前状态，给界面/日志显示用 */
net_state_t net_app_state(void);
bool net_app_wifi_connected(void);
bool net_app_mqtt_connected(void);

/** @brief 立刻发一帧（不等周期）。返回 msg_id，<0 表示当前发不出去 */
int net_app_publish_once(void);

/**
 * @brief 问服务器有没有新固件（发 <prefix>/ota_req）
 *
 * 设备**不需要**预先配置 OTA 地址：服务器收到请求后自己算出本机 IP 和
 * 固件 URL，回在 <prefix>/ota 上，设备收到非空 url 就自动开始下载。
 * 返回 msg_id；-1 表示当前离线，请求没发出去。
 */
int net_app_request_ota_check(void);

#ifdef __cplusplus
}
#endif