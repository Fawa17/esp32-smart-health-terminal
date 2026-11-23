/**
 * @file    max30102.h
 * @brief   MAX30102 心率/血氧传感器驱动（ESP-IDF 版）
 *
 * 相对 STM32 版的三处结构性变化：
 *   1. 位翻转软件 I2C 全部删除，改走 bsp_i2c（硬件 I2C0，共享总线 GPIO8/9）；
 *   2. EXTI + HAL_GPIO_EXTI_Callback 改成 ESP-IDF 的 gpio ISR + 二值信号量；
 *   3. 200 点采样窗口改成双缓冲（ping-pong），消除"算法还在算、采样已在覆盖"的竞争。
 *
 * 地址：0x57（7 位）。原工程里的 0xAE 是把它左移一位后的 8 位写地址，
 *       直接喂给 ESP-IDF 的 i2c_master_write_read_device() 会永远 NACK。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

/** @brief 器件是否在线（连续 I2C 失败，或 FIFO 流水线停摆，就判离线）。 */
bool max30102_is_online(void);

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif


#define MAX30102_I2C_ADDR       0x57    /* 7 位地址 */
#define MAX30102_PART_ID_VAL    0x15    /* REG_PART_ID 的期望值 */

/* INT 引脚：上电时配置为下降沿中断，接 MAX30102 的 INT（漏极开路/推挽低有效）。
 * INT 触发源由 REG_INTR_ENABLE_1 决定，本驱动用的是 A_FULL（FIFO 快满）。 */
#define MAX30102_INT_GPIO_IO    21

#define MAX30102_WINDOW         200     /* 算法窗口点数，与原工程一致 */
#define MAX30102_FIFO_DEPTH     32      /* FIFO 深度，溢出保护的循环上限 */


#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PILOT_PA        0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR       0x1F
#define REG_TEMP_FRAC       0x20
#define REG_TEMP_CONFIG     0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF

/* 位定义（便于读配置） */
#define MODE_RESET_BIT      0x40    /* MODE_CONFIG[6] = RESET */
#define MODE_SPO2           0x03    /* SpO2 模式：红灯 + 红外同时采集 */
#define MODE_HR_ONLY        0x02    /* 只测心率：只开红灯 */


/**
 * @brief 初始化：软复位 → 校验 PART_ID → 写配置 → 清 FIFO → 建信号量 → 挂 INT 中断。
 * @return ESP_OK / ESP_ERR_NOT_FOUND（PART_ID 不是 0x15，多半没接线或地址错）
 * @note   会顺带调用 bsp_i2c_init()，单独调用本函数也能工作。
 */
esp_err_t max30102_init(void);

/** 软复位：写 RESET 位后轮询等待自清（原工程用 Delay_us(20000) 死等） */
esp_err_t max30102_reset(void);

/** 读 PART_ID，返回 ESP_OK 且 *id==0x15 表示器件在线 */
esp_err_t max30102_read_part_id(uint8_t *id);

/** 寄存器读写 */
esp_err_t max30102_write_reg(uint8_t reg, uint8_t val);
esp_err_t max30102_read_reg(uint8_t reg, uint8_t *val);

/** 总线恢复：卸载 I2C → 9 个 SCL 脉冲 + STOP → 重装 → 校验 PART_ID，失败再软复位重配 */
esp_err_t max30102_recover_bus(void);


/** 线程安全地取最近一次有效的 HR/SpO2；指针可传 NULL 表示不关心该字段 */
void max30102_get_heart(uint8_t *heart, uint8_t *spo2,
                        uint8_t *hr_valid, uint8_t *spo2_valid);


/** 采样任务的心跳回调（接回 App/app_tasks.c 的 app_task_heartbeat），不设置也能跑 */
void max30102_set_heartbeat_cb(void (*cb)(void));

/** 创建 vMAX30102_Task（采样）与 vHR_Algorithm_Task（算法），栈/优先级/绑核走 Kconfig */
esp_err_t max30102_start_tasks(void);

/** 采样任务：等 INT 信号量 → 清中断标志 → 把 FIFO 抽空 → 填窗口 */
void vMAX30102_Task(void *arg);

/** 算法任务：等"窗口满"信号量 → 拷一份快照 → 跑 maxim 算法 → 更新结果 */
void vHR_Algorithm_Task(void *arg);

/* ============================== 心率算法（未移植时的弱符号桩） ==============================
 * 原工程 BSP_Drivers/algorithm.c 里的 maxim_heart_rate_and_oxygen_saturation()。
 * 本组件内置一个 __attribute__((weak)) 桩函数，保证工程现在就能编译通过；
 * 你把 algorithm.c 加进来之后，链接器会自动选中你的强符号，桩函数失效。
 */
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                           int32_t   n_ir_buffer_length,
                                           uint32_t *pun_red_buffer,
                                           int32_t  *pn_spo2,
                                           int8_t   *pch_spo2_valid,
                                           int32_t  *pn_heart_rate,
                                           int8_t   *pch_hr_valid);


extern SemaphoreHandle_t xMax30102Semaphore;    /* INT 信号量 */
extern SemaphoreHandle_t xSemaphore_HR_Ready;   /* 窗口满信号量 */

void    MAX30102_Init(void);
int8_t  MAX30102_RecoverBus(void);              /* 0 = 成功，与原工程一致 */
void    MAX30102_GetResult(uint8_t *heart, uint8_t *spo2);
void    MAX30102_GetData(uint8_t *heart, uint8_t *spo2);
uint8_t max30102_Bus_Write(uint8_t reg, uint8_t data);          /* 1 = 成功 */
uint8_t max30102_Bus_Read(uint8_t reg);
void    max30102_FIFO_ReadBytes(uint8_t reg, uint8_t *data);    /* 读 1 个样本 6 字节 */
void    max30102_FIFO_ReadWords(uint8_t reg, uint16_t data[][2], uint8_t count);
void    maxim_max30102_write_reg(uint8_t addr, uint8_t data);
void    maxim_max30102_read_reg(uint8_t addr, uint8_t *data);
void    maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);
void    MAX30102_CreateTask(void);

#ifdef __cplusplus
}
#endif