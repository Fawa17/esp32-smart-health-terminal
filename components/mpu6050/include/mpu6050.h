/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴传感器驱动（ESP-IDF 硬件 I2C 版）
 *
 * 由 STM32 HAL + 软件 I2C 版本迁移而来。寄存器定义、数据结构、
 * 计步/跌倒算法与任务周期全部保持原样，只把"怎么读总线"换掉。
 *
 * 命名说明：
 *   新代码统一用 mpu6050_xxx() + esp_err_t 返回；
 *   文件末尾保留了一段 "兼容层"，把原来的 MPU6050_XXX() 名字原样映射过去，
 *   所以 App/ 里已有的调用点（包括 X-Macro 任务表里的 vMPU6050_Task）
 *   一行都不用改就能编过。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief 器件是否在线。
 *
 * 连续多次 I2C 读失败、或 WHO_AM_I 读不到 0x68，就判离线。
 * 离线时采样任务照常在跑、照常给看门狗打卡（"任务活着"和"器件还在"是两件事），
 * 但上层应该把这帧的步数/跌倒当成无效数据，并在界面上给出告警。
 */
bool mpu6050_is_online(void);

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 从机地址 ============================
 * 迁移最容易踩的坑：
 *   原 STM32 代码里 MPU6050_ADDRESS = 0xD0，那是"8 位写地址"(0x68 << 1)，
 *   因为软件 I2C 是直接把整个字节怼到总线上。
 *   ESP-IDF 的 i2c_master_* 接口收的是 **7 位地址**，内部自己左移。
 *   如果这里还传 0xD0，驱动会先左移成 0x1A0 再截断成 0xA0，从机必然 NACK。
 */
#define MPU6050_I2C_ADDR        0x68    /* AD0 引脚接地 */
#define MPU6050_I2C_ADDR_AD0    0x69    /* AD0 引脚接高 */

/* WHO_AM_I 寄存器固定返回该值 */
#define MPU6050_WHO_AM_I_VAL    0x68

#define MPU_SAMPLE_RATE_REG     0x19
#define MPU_CFG_REG             0x1A
#define MPU_GYRO_CFG_REG        0x1B
#define MPU_ACCEL_CFG_REG       0x1C
#define MPU_MOTION_DET_REG      0x1F
#define MPU_FIFO_EN_REG         0x23
#define MPU_I2CMST_CTRL_REG     0x24
#define MPU_I2CSLV0_ADDR_REG    0x25
#define MPU_I2CSLV0_REG         0x26
#define MPU_I2CSLV0_CTRL_REG    0x27
#define MPU_I2CSLV1_ADDR_REG    0x28
#define MPU_I2CSLV1_REG         0x29
#define MPU_I2CSLV1_CTRL_REG    0x2A
#define MPU_I2CSLV2_ADDR_REG    0x2B
#define MPU_I2CSLV2_REG         0x2C
#define MPU_I2CSLV2_CTRL_REG    0x2D
#define MPU_I2CSLV3_ADDR_REG    0x2E
#define MPU_I2CSLV3_REG         0x2F
#define MPU_I2CSLV3_CTRL_REG    0x30
#define MPU_I2CSLV4_ADDR_REG    0x31
#define MPU_I2CSLV4_REG         0x32
#define MPU_I2CSLV4_DO_REG      0x33
#define MPU_I2CSLV4_CTRL_REG    0x34
#define MPU_I2CSLV4_DI_REG      0x35
#define MPU_I2CMST_STA_REG      0x36
#define MPU_INTBP_CFG_REG       0x37
#define MPU_INT_EN_REG          0x38
#define MPU_INT_STA_REG         0x3A
#define MPU_ACCEL_XOUTH_REG     0x3B    /* 连续读到 0x48 共 14 字节：accel(6)+temp(2)+gyro(6) */
#define MPU_ACCEL_XOUTL_REG     0x3C
#define MPU_ACCEL_YOUTH_REG     0x3D
#define MPU_ACCEL_YOUTL_REG     0x3E
#define MPU_ACCEL_ZOUTH_REG     0x3F
#define MPU_ACCEL_ZOUTL_REG     0x40
#define MPU_TEMP_OUTH_REG       0x41
#define MPU_TEMP_OUTL_REG       0x42
#define MPU_GYRO_XOUTH_REG      0x43
#define MPU_GYRO_XOUTL_REG      0x44
#define MPU_GYRO_YOUTH_REG      0x45
#define MPU_GYRO_YOUTL_REG      0x46
#define MPU_GYRO_ZOUTH_REG      0x47
#define MPU_GYRO_ZOUTL_REG      0x48
#define MPU_I2CSLV0_DO_REG      0x63
#define MPU_I2CSLV1_DO_REG      0x64
#define MPU_I2CSLV2_DO_REG      0x65
#define MPU_I2CSLV3_DO_REG      0x66
#define MPU_I2CMST_DELAY_REG    0x67
#define MPU_SIGPATH_RST_REG     0x68
#define MPU_MDETECT_CTRL_REG    0x69
#define MPU_USER_CTRL_REG       0x6A
#define MPU_PWR_MGMT1_REG       0x6B
#define MPU_PWR_MGMT2_REG       0x6C
#define MPU_FIFO_CNTH_REG       0x72
#define MPU_FIFO_CNTL_REG       0x73
#define MPU_FIFO_RW_REG         0x74
#define MPU_DEVICE_ID_REG       0x75    /* 即 WHO_AM_I */

typedef struct {
    int16_t  ax, ay, az;        /* 加速度原始值（ADC，±2g 时 16384 LSB/g） */
    int16_t  gx, gy, gz;        /* 陀螺仪原始值（已扣除零偏） */
    uint32_t step_count;        /* 计步数 */
    uint8_t  tumble_state;      /* 跌倒状态（0=正常，1=跌倒） */
} MPU6050_t;

/* IDF 风格的别名，新代码推荐用它 */
typedef MPU6050_t mpu6050_data_t;

/* 原工程遗留的调试用全局标志（供测试代码强制注入故障），保留以兼容 */
extern volatile uint8_t g_mpu_kill_flag;


/**
 * @brief 初始化 MPU6050：总线检查 → 软复位 → 唤醒 → 量程/速率配置 → WHO_AM_I 校验 → 陀螺仪零偏校准。
 * @note  内部会兜底调用 bsp_i2c_init()；整个流程约 250~400ms（含校准），
 *        因此必须在调度器启动之后（app_main 里）调用。
 * @note  校准时设备必须静止放平，否则零偏会被算进重力姿态里。
 * @return ESP_OK / ESP_ERR_NOT_FOUND（ID 不对，多半是接线或地址错）/ 其它 I2C 错误
 */
esp_err_t mpu6050_init(void);

/** 读 WHO_AM_I（0x75），用于在线自检 */
esp_err_t mpu6050_read_id(uint8_t *id);

/**
 * @brief 陀螺仪零偏校准：连续采 samples 次求平均，结果存入内部 offset。
 * @param samples 采样次数，原工程用 50（每 5ms 一次，共 250ms）
 */
esp_err_t mpu6050_calibrate_gyro(uint16_t samples);

/**
 * @brief 总线/器件级恢复（原 MPU6050_RecoverBus 的完整版）。
 *        先做硬件总线复位（9 时钟 + STOP + 重装驱动），仍失败则对芯片软复位后重走 init。
 * @return ESP_OK 表示恢复成功
 */
esp_err_t mpu6050_recover_bus(void);


/**
 * @brief 读一次原始数据（3 次采样求平均，加速度/陀螺仪在同一帧内取，时间对齐）。
 *        只填 ax..gz，不动 step_count / tumble_state。
 */
esp_err_t mpu6050_read_raw(MPU6050_t *out);

/**
 * @brief 采样 + 计步 + 跌倒判定，一次调用完成（原 MPU6050_UpdateStepAndTumble）。
 * @note  内部算法依赖"调用周期稳定"，原工程是 40ms 一次，改周期要重新调阈值。
 */
esp_err_t mpu6050_update_step_tumble(MPU6050_t *data);

/** 线程安全地取出计步数与跌倒状态（可在任意任务里调） */
void mpu6050_get_motion(uint32_t *steps, uint8_t *tumble);

/**
 * @brief 最近一次算出的动态加速度（合加速度 - 重力基线），单位 LSB。
 *
 * 计步不涨时先看这个：正常走路峰值应有 5000~8000；晃动板子它若还在
 * 几百以内，说明信号太弱（量程/安装/根本没动），与状态机无关。
 * 用来区分"没捡到峰值"和"阈值/间隔把步数卡掉了"。
 */
int32_t mpu6050_get_last_dyn(void);


/**
 * @brief 注册心跳回调。驱动不再反向依赖 app_tasks.h，
 *        由应用层把 app_task_heartbeat(APP_TASK_MPU) 之类的函数塞进来。
 *        传 NULL 表示不需要。
 */
void mpu6050_set_heartbeat_cb(void (*cb)(void));

/** 创建 40ms 周期的采样任务（独立跑时用；接了 X-Macro 任务表就别调这个，否则会建两个） */
void mpu6050_start_task(void);

/** 采样任务本体，签名与原来的 vMPU6050_Task 完全一致，可直接塞进 APP_TASK_LIST */
void vMPU6050_Task(void *pvParameters);

/* ==================================================================
 *                     兼容层（供旧代码零改动调用）
 *   原 App/ 里凡是写 MPU6050_Init() / MPU6050_RecoverBus() 的地方都能编过。
 *   新代码请直接用上面 mpu6050_xxx 版本，能拿到错误码。
 * ================================================================== */

static inline void MPU6050_Init(void) { (void)mpu6050_init(); }

/* 失败时返回 0（调用点的判断都是 id != 0x68，语义一致） */
uint8_t MPU6050_Read_ID(void);

void MPU6050_CalibrateGyro(uint16_t samples);

/* 返回 0=成功 / 1=失败，正好匹配 BSP_Drivers/tasks.h 里的 RecoverCallback_t */
int8_t MPU6050_RecoverBus(void);

void MPU6050_GetRawData(MPU6050_t *data);          /* 原头文件声明了但工程里没有实现，这里补上 */
void MPU6050_UpdateStepAndTumble(MPU6050_t *data); /* 原名字，忽略返回码 */
void MPU6050_CreateTask(void);                     /* 等价于 mpu6050_start_task() */

#ifdef __cplusplus
}
#endif