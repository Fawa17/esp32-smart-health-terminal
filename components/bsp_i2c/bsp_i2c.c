/**
 * @file    bsp_i2c.c
 * @brief   硬件 I2C 总线封装（ESP-IDF legacy driver，driver/i2c.h）
 *
 * 而且必须靠 __disable_irq() 保护时序。现在时序由 I2C 外设硬件产生：
 * - CPU 不再参与每个 bit，读 14 字节从"~2ms 且不能被中断"变成"一次 DMA 式传输 + 阻塞等完成"
 * - 失败会明确返回 esp_err_t（NACK / 超时 / 总线忙），不再像软件 I2C 那样
 */

#include "bsp_i2c.h"

#include <string.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "bsp_i2c";

/* 总线互斥量：
 *   legacy 驱动内部对"一次 cmd_begin"是原子的，但复合操作
 *   （比如 bus_reset 要先删驱动再敲时钟）需要更大粒度的互斥，
 *   否则另一个任务正好插进来收发就会踩到已卸载的驱动。 */
static SemaphoreHandle_t s_bus_mux  = NULL;
static bool              s_installed = false;

/* 内部：仅安装驱动，调用者必须已持有 s_bus_mux                          */
static esp_err_t install_locked(void)
{
    if (s_installed) {
        return ESP_OK;
    }

    const i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BSP_I2C_SDA_GPIO,
        .scl_io_num       = BSP_I2C_SCL_GPIO,
        /* 内部上拉只有 ~45kΩ，只够"防浮空"，400kHz 必须靠外部 4.7kΩ。
         * 这里保持 ENABLE，是为了在没焊外部电阻时至少能扫到地址（速率会掉到 100k）。 */
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_CLK_HZ,
        .clk_flags        = 0,
    };

    esp_err_t err = i2c_param_config(BSP_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 主机模式不需要收发缓冲区，slv_rx/tx_buf_len 传 0；intr_alloc_flags 传 0 用默认中断分配 */
    err = i2c_driver_install(BSP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install 失败: %s", esp_err_to_name(err));
        return err;
    }

    s_installed = true;
    ESP_LOGI(TAG, "I2C%d 已就绪: SDA=GPIO%d SCL=GPIO%d %dkHz",
             (int)BSP_I2C_PORT, (int)BSP_I2C_SDA_GPIO, (int)BSP_I2C_SCL_GPIO,
             BSP_I2C_CLK_HZ / 1000);
    return ESP_OK;
}

/* 生命周期                                                            */
esp_err_t bsp_i2c_init(void)
{
    if (s_bus_mux == NULL) {
        /* 二值信号量当互斥量用即可，不需要优先级继承之外的特性；
         * 这里直接用 Mutex 语义（xSemaphoreCreateMutex 支持优先级继承） */
        s_bus_mux = xSemaphoreCreateMutex();
        if (s_bus_mux == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (xSemaphoreTake(s_bus_mux, pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = install_locked();
    xSemaphoreGive(s_bus_mux);
    return err;
}

/*
 * 向 I2C 设备写入数据
 */
esp_err_t bsp_i2c_write(uint8_t dev7, const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_bus_mux == NULL || !s_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    const TickType_t to = pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS);
    if (xSemaphoreTake(s_bus_mux, to) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = i2c_master_write_to_device(BSP_I2C_PORT, dev7, buf, len, to);
    xSemaphoreGive(s_bus_mux);
    return err;
}
/*
 * 从 I2C 设备读取数据
 */
esp_err_t bsp_i2c_read(uint8_t dev7, uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_bus_mux == NULL || !s_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    const TickType_t to = pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS);
    if (xSemaphoreTake(s_bus_mux, to) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = i2c_master_read_from_device(BSP_I2C_PORT, dev7, buf, len, to);
    xSemaphoreGive(s_bus_mux);
    return err;
}
/*
 * 写单个寄存器
 */
esp_err_t bsp_i2c_write_reg(uint8_t dev7, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return bsp_i2c_write(dev7, buf, sizeof(buf));
}

/*
 * 读寄存器块，这条函数替换掉了原工程 4 个位翻转函数
 * （I2C_Start / I2C_SendByte / I2C_RecviceByte / I2C_SendACK）的全部工作。
 *
 * i2c_master_write_read_device 内部就是：
 *   START → addr|W → reg → 重复START → addr|R → 读 len 字节 → STOP
 * 与手工写的时序完全一致，只是由硬件外设产生。
 */
esp_err_t bsp_i2c_read_reg(uint8_t dev7, uint8_t reg, uint8_t *buf, size_t len)
{
    // 1. 参数检查
    if (buf == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    if (s_bus_mux == NULL || !s_installed) return ESP_ERR_INVALID_STATE;

    // 2. 拿锁（最多等 50ms）
    const TickType_t to = pdMS_TO_TICKS(50);
    if (xSemaphoreTake(s_bus_mux, to) != pdTRUE) return ESP_ERR_TIMEOUT;

    // 3. 调用 IDF 的复合事务函数
    esp_err_t err = i2c_master_write_read_device(
        BSP_I2C_PORT,   // 哪个 I2C 端口
        dev7,           // 7 位从机地址 如（0x68）
        &reg, 1,        // 先写 1 字节：寄存器地址
        buf, len,       // 再读 len 字节
        to              // 超时
    );

    // 4. 放锁
    xSemaphoreGive(s_bus_mux);
    return err;
}

/*
 * 探测 I2C 设备是否在线
 */
esp_err_t bsp_i2c_probe(uint8_t dev7)
{
    if (s_bus_mux == NULL || !s_installed) return ESP_ERR_INVALID_STATE;

    const TickType_t to = pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS);
    if (xSemaphoreTake(s_bus_mux, to) != pdTRUE) return ESP_ERR_TIMEOUT;

    /* 方式一（首选）：只发"START + 地址字节 + STOP"，不看数据阶段。
     *
     * 之前用 bsp_i2c_write(dev7, &dummy, 1) 判断，写的是"一个数据字节"：
     * 触摸 IC 这类器件通常地址阶段 ACK、数据阶段 NACK（它期望先收到
     * 寄存器地址，收到别的就当非法命令）。而 IDF 的 i2c_master_write_to_device
     * 只要有任意一个字节 NACK 就返回 ESP_FAIL，于是"在线"的器件被
     * 判成"不在线"，扫描表里直接消失。
     * 只用地址字节探测就没有这个问题：ACK 只代表"这个地址有人应答"。 */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((dev7 << 1) | I2C_MASTER_WRITE), true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(BSP_I2C_PORT, cmd, to);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(s_bus_mux);

    if (err == ESP_OK) return ESP_OK;

    /* 方式二（兜底）：有少数器件对"地址+立即 STOP"不感冒，却愿意被读。
     * 读 1 字节：地址阶段 ACK 且能给数据，才返回 ESP_OK。 */
    uint8_t dummy = 0;
    return bsp_i2c_read(dev7, &dummy, 1);
}

/* 故障恢复                                                            */
esp_err_t bsp_i2c_bus_reset(void)
{
    if (s_bus_mux == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 用 portMAX_DELAY：宁可等，也不能删掉别人正在用的驱动 */
    if (xSemaphoreTake(s_bus_mux, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 1) 卸载驱动。i2c_driver_delete 会把 GPIO 矩阵里的 I2C 信号断开。
     *    没装过就忽略返回值（ESP_ERR_INVALID_STATE）。 */
    if (s_installed) {
        (void)i2c_driver_delete(BSP_I2C_PORT);
        s_installed = false;
    }

    /* 2) 引脚还原成普通开漏 GPIO（开漏 = 与真实 I2C 电气一致，
     *    从机仍在拉低时主机写 1 也不会短路） */
    gpio_reset_pin(BSP_I2C_SDA_GPIO);
    gpio_reset_pin(BSP_I2C_SCL_GPIO);
    gpio_set_direction(BSP_I2C_SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(BSP_I2C_SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(BSP_I2C_SDA_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BSP_I2C_SCL_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_level(BSP_I2C_SDA_GPIO, 1);
    gpio_set_level(BSP_I2C_SCL_GPIO, 1);
    esp_rom_delay_us(10);

    /* 3) 9 个 SCL 脉冲：把卡在"等第 9 个时钟"状态的从机逐出 */
    for (int i = 0; i < 9; i++) {
        gpio_set_level(BSP_I2C_SCL_GPIO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(BSP_I2C_SCL_GPIO, 1);
        esp_rom_delay_us(5);
    }

    /* 4) 补一个 STOP：SCL 保持高，SDA 由低跳高 */
    gpio_set_level(BSP_I2C_SCL_GPIO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(BSP_I2C_SDA_GPIO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(BSP_I2C_SCL_GPIO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(BSP_I2C_SDA_GPIO, 1);
    esp_rom_delay_us(10);

    /* 5) 重装驱动，恢复硬件外设接管引脚 */
    esp_err_t err = install_locked();
    xSemaphoreGive(s_bus_mux);

    if (err == ESP_OK) {
        ESP_LOGW(TAG, "总线已复位（9 时钟 + STOP）");
    }
    return err;
}

/* 调试辅助                                                            */
void bsp_i2c_scan(void)
{
    if (s_bus_mux == NULL || !s_installed) {
        ESP_LOGE(TAG, "扫描前请先调用 bsp_i2c_init()");
        return;
    }

    ESP_LOGI(TAG, "开始扫描 I2C 总线 ...");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (bsp_i2c_probe(addr) == ESP_OK) {
            const char *hint = "";
            switch (addr) {
            case 0x15: hint = "  <- 屏幕模块";      break;
            case 0x68: hint = "  <- MPU6050 (AD0=GND)";   break;
            case 0x69: hint = "  <- MPU6050 (AD0=VCC)";   break;
            case 0x57: hint = "  <- MAX30102";            break;
            default:   break;
            }
            ESP_LOGI(TAG, "  发现 0x%02X%s", addr, hint);
            found++;
        }
        /* 每扫一个地址让出 1 tick，避免长时间霸占 CPU（扫描约 130ms） */
        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "扫描结束，共 %d 个器件", found);
}