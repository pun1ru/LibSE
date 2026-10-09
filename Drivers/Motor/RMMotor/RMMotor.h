#ifndef RMMOTOR_H
#define RMMOTOR_H

#include <stdint.h>

#include "CAN_driver.h"
#include "Motor.h"

#define RM_C610_CURRENT_LIMIT 10000.0f
#define RM_C620_CURRENT_LIMIT 16384.0f
#define RM_GM6020_CURRENT_LIMIT 16384.0f
#define RM_GM6020_VOLTAGE_LIMIT 25000.0f

typedef enum {
    RM_C610,
    RM_C620,
    RM_GM6020
} RMMotorType_t;

typedef enum {
    RM_CURRENT,
    RM_VOLTAGE
} RMMotorMode_t;

typedef struct {
    RMMotorType_t motor_type;
    RMMotorMode_t mode;
    uint8_t motor_id;
} RMMotorConfig_t;

typedef struct RMMotor RMMotor_t;

typedef struct {
    FDCAN_HandleTypeDef *can;
    uint16_t tx_id;
    RMMotor_t *motors[4];
} RMMotorGroup_t;

struct RMMotor {
    Motor_t base;
    RMMotorConfig_t motor_config;
    RMMotorGroup_t *group;
};

/* C610/C620: 0x200 (ID 1-4), 0x1FF (ID 5-8).
 * GM6020 voltage: 0x1FF/0x2FF; current: 0x1FE/0x2FE.
 * A CAN bus must have only one group for each TX ID. Feedback IDs can
 * also overlap between C610/C620 and GM6020.
 */
void RMMotorGroup_Init(RMMotorGroup_t *group, FDCAN_HandleTypeDef *can,
                       uint16_t tx_id);
void RMMotor_Init(RMMotor_t *motor, RMMotorGroup_t *group,
                  const RMMotorConfig_t *config);

/* motorcmd.current is the signed protocol setpoint: C610 +/-10000,
 * C620 +/-16384, GM6020 current +/-16384 or voltage +/-25000.
 * Both send callbacks transmit all four slots of the motor's group.
 */
void RMMotor_SendGroupCommand(RMMotor_t *motor);
void RMMotor_Feedback(RMMotor_t *motor, const uint8_t data[8], uint16_t rec_id);

#endif
