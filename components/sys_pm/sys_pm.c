/*
 * sys_pm.c，屏幕电源状态机 + 按键唤醒
 *
 * 三个角色：
 *   - 按键 ISR：只叫醒任务，别的什么都不干
 *   - sys_pm 任务：消抖、翻转状态、超时判定、调 hooks
 *   - 其它任务：读 sys_pm_is_active() 决定自己要不要降频
 */

#include "sys_pm.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sys_pm";

#define PM_NOW_MS() ((int64_t)(esp_timer_get_time() / 1000))

/* 消抖：中断来了先等这么久再看电平，避开机械抖动 */
#define KEY_DEBOUNCE_MS   20
/* 松手判定：每这么久查一次电平 */
#define KEY_RELEASE_POLL_MS 20
/* 进入休眠前等 UI 任务把手上的这一帧画完 */
#define SLEEP_SETTLE_MS   200

static volatile sys_pm_state_t s_state  = SYS_PM_ACTIVE;
static volatile int64_t        s_last_activity_ms;

static TaskHandle_t            s_task;
static sys_pm_hooks_t          s_hooks;
static bool                  (*s_busy_check)(void);

/* 按键中断                                                           */

static void IRAM_ATTR key_isr(void *arg)
{
    (void)arg;
    BaseType_t hp = pdFALSE;

    if (s_task != NULL) {
        /* 用任务通知而不是信号量：不进堆、不建内核对象、唤醒路径最短。
         * ISR 里不读 GPIO、不打日志、不发 I2C，那些都在任务里做。 */
        vTaskNotifyGiveFromISR(s_task, &hp);
        if (hp) {
            portYIELD_FROM_ISR();
        }
    }
}

static bool key_level_pressed(void)
{
#if CONFIG_SYS_PM_KEY_ACTIVE_LOW
    return gpio_get_level(CONFIG_SYS_PM_KEY_GPIO) == 0;
#else
    return gpio_get_level(CONFIG_SYS_PM_KEY_GPIO) != 0;
#endif
}

static esp_err_t key_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_SYS_PM_KEY_GPIO,
        .mode         = GPIO_MODE_INPUT,
#if CONFIG_SYS_PM_KEY_ACTIVE_LOW
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
#else
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
#endif
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "按键 GPIO 配置失败：%s", esp_err_to_name(err));
        return err;
    }

    /* max30102 和 touch 可能已经装过 ISR 服务，重复安装返回 INVALID_STATE，
     * 这是正常的，不是错误。 */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service 失败：%s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(CONFIG_SYS_PM_KEY_GPIO, key_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册按键中断失败：%s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "按键就绪：GPIO%d（%s 有效），当前电平=%d",
             CONFIG_SYS_PM_KEY_GPIO,
             CONFIG_SYS_PM_KEY_ACTIVE_LOW ? "低" : "高",
             gpio_get_level(CONFIG_SYS_PM_KEY_GPIO));
    return ESP_OK;
}

/* 状态切换                                                           */

void sys_pm_wake(void)
{
    if (s_state == SYS_PM_ACTIVE) {
        s_last_activity_ms = PM_NOW_MS();
        return;
    }

    ESP_LOGI(TAG, "唤醒：面板 → 亮、触摸 → 50Hz、LVGL → 继续渲染");

    /* 先把状态翻成 ACTIVE：主循环和触摸任务看到它就会恢复各自的工作，
     * 而不是傻等这个函数里那 120ms 的 SLPOUT 延时。
     * 真正"能在屏幕上看到东西"靠 hook 末尾的整屏重画保证。 */
    s_state            = SYS_PM_ACTIVE;
    s_last_activity_ms = PM_NOW_MS();

    /* 顺序很重要：先把面板点亮（SLPOUT 要等 120ms 才稳定），
     * 再恢复触摸和渲染，否则第一批像素可能落在面板还没醒的时候丢掉。 */
    if (s_hooks.on_wake != NULL) {
        s_hooks.on_wake();
    }
}

void sys_pm_sleep(void)
{
    if (s_state == SYS_PM_SLEEP) {
        return;
    }

    if (s_busy_check != NULL && s_busy_check()) {
        ESP_LOGW(TAG, "有正忙的任务（比如 OTA），本次不进入休眠");
        return;
    }

    /* 先改状态：UI 任务下一次循环（≤150ms）就会看到"不活跃"并暂停刷新 */
    s_state = SYS_PM_SLEEP;

    /* 再等一下，让已经在路上的那一次 SPI 传输发完。
     * 否则 DISPOFF 可能正好落在 DMA 传输中间。 */
    vTaskDelay(pdMS_TO_TICKS(SLEEP_SETTLE_MS));

    ESP_LOGI(TAG, "休眠：面板 DISPOFF+SLPIN、触摸挂起、LVGL 暂停（采集与网络照常）");

    if (s_hooks.on_sleep != NULL) {
        s_hooks.on_sleep();
    }
}

bool sys_pm_is_active(void)
{
    return s_state == SYS_PM_ACTIVE;
}

sys_pm_state_t sys_pm_state(void)
{
    return s_state;
}

void sys_pm_notify_activity(void)
{
    s_last_activity_ms = PM_NOW_MS();
}

bool sys_pm_key_pressed(void)
{
    return key_level_pressed();
}

void sys_pm_set_busy_check(bool (*fn)(void))
{
    s_busy_check = fn;
}

/* 管理任务                                                           */

static void sys_pm_task(void *arg)
{
    (void)arg;

    for (;;) {
        /* 最多睡 200ms 醒一次：既能让按键立刻响应，又能按时跑超时判定。
         * 不用更长的超时是因为 ulTaskNotifyTake 的返回值只表示"有没有通知"，
         * 超时粒度决定了空闲判定的精度。 */
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

        if (notified > 0) {
            /* 有边沿 → 消抖：等 20ms 再确认电平还按着 */
            vTaskDelay(pdMS_TO_TICKS(KEY_DEBOUNCE_MS));

            if (key_level_pressed()) {
                if (s_state == SYS_PM_ACTIVE) {
                    sys_pm_sleep();
                } else {
                    /* 休眠中按键 = 唤醒。sys_pm_sleep 里那句 200ms 延时
                     * 会让"按住不放"在这期间又被判定一次，所以唤醒后
                     * 必须等松手，否则一按就开关好几轮。 */
                    sys_pm_wake();
                }

                /* 等松手（带上限，防止按键卡死把任务挂在这里） */
                for (int i = 0; i < 500 && key_level_pressed(); i++) {
                    vTaskDelay(pdMS_TO_TICKS(KEY_RELEASE_POLL_MS));
                }
                s_last_activity_ms = PM_NOW_MS();
            }
        }

        /* 无操作超时 */
        if (s_state == SYS_PM_ACTIVE &&
            (PM_NOW_MS() - s_last_activity_ms) >= (int64_t)CONFIG_SYS_PM_IDLE_TIMEOUT_S * 1000) {
            if (s_busy_check == NULL || !s_busy_check()) {
                ESP_LOGI(TAG, "%d 秒无操作，自动息屏", CONFIG_SYS_PM_IDLE_TIMEOUT_S);
                sys_pm_sleep();
            }
        }
    }
}

/* 初始化                                                             */

esp_err_t sys_pm_init(const sys_pm_hooks_t *hooks)
{
    if (hooks != NULL) {
        s_hooks = *hooks;
    } else {
        memset(&s_hooks, 0, sizeof(s_hooks));
    }

    esp_err_t err = key_init();
    if (err != ESP_OK) {
        return err;
    }

    s_last_activity_ms = PM_NOW_MS();

#if !CONFIG_SYS_PM_START_ACTIVE
    /* 调试用：上电直接进息屏，方便验证"按一下能不能亮" */
    s_state = SYS_PM_SLEEP;
    if (s_hooks.on_sleep != NULL) {
        s_hooks.on_sleep();
    }
#endif

    BaseType_t ok = xTaskCreatePinnedToCore(sys_pm_task, "sys_pm",
                                           CONFIG_SYS_PM_TASK_STACK, NULL,
                                           CONFIG_SYS_PM_TASK_PRIO, &s_task,
                                           CONFIG_SYS_PM_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建 sys_pm 任务失败");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "已启动：起始状态=%s，无操作 %d 秒后息屏",
             s_state == SYS_PM_ACTIVE ? "亮屏" : "息屏",
             CONFIG_SYS_PM_IDLE_TIMEOUT_S);
    return ESP_OK;
}