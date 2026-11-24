#ifndef DMIMU_H
#define DMIMU_H

#include <stdbool.h>
#include <stdint.h>

#include "CAN_driver.h"
#include "uart_driver.h"

/* 达妙 DM-IMU-L1，固件 v2.1.0.0 及以上，CAN 与 RS485 共用一套接口 */

/* 链路默认参数，外设波特率由 CubeMX 配置，这里只是约定值 */
#define DMIMU_CAN_BAUDRATE_DEFAULT      1000000U
#define DMIMU_RS485_BAUDRATE_DEFAULT    2000000U
/* 模块自身波特率索引：CAN 0 = 1M，RS485 8 = 2M，写入后保存并重启生效 */
#define DMIMU_CAN_BAUD_INDEX_1M         0U
#define DMIMU_RS485_BAUD_INDEX_2M       8U

#define DMIMU_SLAVE_ID_DEFAULT          1U
#define DMIMU_MASTER_ID_DEFAULT         0x11U
#define DMIMU_TIMEOUT_MS_DEFAULT        80U
#define DMIMU_REQUEST_GAP_MS_DEFAULT    5U
#define DMIMU_OFFLINE_MS_DEFAULT        500U

/* 默认量程（24g / 2000dps），改了模块量程需要同步修改配置 */
#define DMIMU_ACCEL_HALF_RANGE_DEFAULT  235.2f
#define DMIMU_GYRO_HALF_RANGE_DEFAULT   34.88f

/* 扩展寄存器，CAN 与 RS485 编号相同，配置类写入需要先进入设置模式 */
#define DMIMU_REG_MODE                  0x20U
#define DMIMU_REG_OUTPUT_BITMAP         0x29U
#define DMIMU_REG_REPORT_INTERVAL       0x2AU
#define DMIMU_REG_SLAVE_ID              0x2CU
#define DMIMU_REG_MASTER_ID             0x2DU
#define DMIMU_REG_CAN_BAUD_INDEX        0x2EU
#define DMIMU_REG_RS485_BAUD_INDEX      0x2FU
#define DMIMU_REG_TEMPERATURE           0x31U

#define DMIMU_MODE_NORMAL               0U
#define DMIMU_MODE_SETTING              1U

#define DMIMU_ACK_SUCCESS               0U

#define DMIMU_RS485_FRAME_LEN           24U
#define DMIMU_CAN_FRAME_LEN             8U

typedef enum {
    DMIMU_LINK_CAN,
    DMIMU_LINK_RS485
} DMIMULink_t;

typedef enum {
    DMIMU_DATA_ACCEL = 0,
    DMIMU_DATA_GYRO,
    DMIMU_DATA_EULER,
    DMIMU_DATA_QUAT,
    DMIMU_DATA_COUNT
} DMIMUDataType_t;

#define DMIMU_VALID_BIT(type)           (1U << (type))

typedef struct {
    DMIMULink_t link;

    /* 外设层：按 link 只用其中一个 */
    CanInstance_t *can;
    UartInstance_t *uart;

    /* 参数层 */
    uint16_t slave_id;
    uint16_t master_id;
    uint16_t timeout_ms;
    uint16_t request_gap_ms;
    uint16_t offline_ms;
    bool auto_request;
    float accel_half_range;
    float gyro_half_range;
} DMIMUConfig_t;

typedef struct {
    float acc[3];                       /* m/s^2 */
    float gyro[3];                      /* rad/s */
    float euler[3];                     /* 度，横滚 俯仰 偏航 */
    float quat[4];                      /* w x y z */
    float temp;                         /* 摄氏度，只有 CAN 的加速度帧带 */
    uint8_t valid;                      /* DMIMU_VALID_BIT 位图 */
    uint32_t tick_ms[DMIMU_DATA_COUNT]; /* 各类数据最近一次接收时刻 */
} DMIMUFeedback_t;

typedef struct {
    uint32_t frame_ok;
    uint32_t crc_error;
    uint32_t format_error;
    uint32_t timeout_count;
} DMIMUStats_t;

typedef struct {
    uint8_t cmd;
    uint8_t ack;
    uint32_t value;
    volatile uint8_t pending;
} DMIMUCmdResult_t;

typedef struct {
    DMIMUConfig_t config;
    DMIMUFeedback_t feedback;
    DMIMUStats_t stats;
    DMIMUCmdResult_t cmd_result;

    uint8_t rx_buf[DMIMU_RS485_FRAME_LEN];
    uint8_t tx_buf[DMIMU_RS485_FRAME_LEN];
    uint8_t rx_len;

    volatile uint8_t tx_inflight;
    uint8_t tx_is_cmd;
    uint8_t tx_id;
    uint8_t next_type;
    uint32_t tx_start_ms;
    uint32_t last_done_ms;
} DMIMU_t;

void DMIMU_ConfigDefault(DMIMUConfig_t *config);
/* 绑定接收回调，要在 CAN 或串口启动之前调用，失败说明绑定表已满 */
bool DMIMU_Init(DMIMU_t *imu, const DMIMUConfig_t *config);
/* 1~10 ms 周期调用：处理应答超时，按配置自动轮询四类数据 */
void DMIMU_Update(DMIMU_t *imu);

/* 请求一类数据，应答由接收回调解析；上一笔未完成返回 false */
bool DMIMU_Request(DMIMU_t *imu, DMIMUDataType_t type);
/* 最近一次接收距今超过 offline_ms 或从未收到数据都算掉线 */
bool DMIMU_IsOnline(const DMIMU_t *imu);

/* 命令接口，非阻塞，上一笔未完成返回 false，有副作用的命令不重发 */
bool DMIMU_ReadReg(DMIMU_t *imu, uint8_t reg);
bool DMIMU_WriteReg(DMIMU_t *imu, uint8_t reg, uint32_t value);
bool DMIMU_SetSettingMode(DMIMU_t *imu, bool enable);
bool DMIMU_SetBaudIndex(DMIMU_t *imu, uint8_t index);
bool DMIMU_YawZero(DMIMU_t *imu);
bool DMIMU_Save(DMIMU_t *imu);
bool DMIMU_Restart(DMIMU_t *imu);
/* 取走最近一次命令应答，有新结果返回 true */
bool DMIMU_GetCmdResult(DMIMU_t *imu, DMIMUCmdResult_t *result);

#endif
