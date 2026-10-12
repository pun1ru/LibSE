#include "task_system.h"

#include "DMMotor.h"
#include "LKMotor.h"
#include "RMMotor.h"

#define TASK_FRIC_GROUP_COUNT     2U

CanInstance_t can_instances[TASK_CAN_INSTANCE_COUNT];
QueueSetHandle_t receive_can_queues;
QueueSetHandle_t transmit_can_queues;

UartInstance_t uart_instances[TASK_UART_INSTANCE_COUNT];
EventGroupHandle_t UartEventGroup;

Referee_t referee_receiver;
RC_Remote_t rc_receiver;

TaskHandle_t MonitorTaskHandle;
TaskHandle_t ReceiveTaskHandle;
TaskHandle_t TransmitTaskHandle;
TaskHandle_t INSTaskHandle;

INS_t board_ins;

static LKMotor_t pitchMotorInstance;
static RMMotor_t fricMotorInstances[TASK_FRIC_MOTOR_COUNT];
static RMMotorGroup_t fricMotorGroups[TASK_FRIC_GROUP_COUNT];
static DMMotor_t yawMotorInstance;

Motor_t *pitchMotor = &pitchMotorInstance.base;
Motor_t *fricMotors[TASK_FRIC_MOTOR_COUNT];
Motor_t *yawMotor = &yawMotorInstance.base;
TaskMotorBinding_t can1MotorBindings[TASK_CAN1_MOTOR_BINDING_COUNT];
TaskMotorBinding_t can2MotorBindings[TASK_CAN2_MOTOR_BINDING_COUNT];
TaskMotorBinding_t can3MotorBindings[TASK_CAN3_MOTOR_BINDING_COUNT];

#define TASK_PITCH_MOTOR_ID       1U
#define TASK_YAW_MOTOR_ID         0x007U
#define TASK_YAW_FEEDBACK_ID      0x017U
#define TASK_FRIC_LOW_GROUP_ID    0x200U
#define TASK_FRIC_HIGH_GROUP_ID   0x1FFU
#define TASK_FRIC_HIGH_GROUP_MIN  5U

void TaskSystem_InitMotors(void)
{
    static const uint8_t friction_motor_ids[TASK_FRIC_MOTOR_COUNT] = {
        1U, 2U, 3U, 5U, 6U, 7U
    };
    LKMotorConfig_t pitch_config = {
        .can = &hfdcan1,
        .motor_id = TASK_PITCH_MOTOR_ID,
        .max_speed_dps = 0U,
        .current_per_lsb = LK_MF_CURRENT_PER_LSB,
        .zero_point = 0U,
        .inverse = false
    };
    MotorConfig_t yaw_config = {
        .motor_type = DM_6006,
        .can = &hfdcan3,
        .motor_id = TASK_YAW_MOTOR_ID,
        .feedback_id = TASK_YAW_FEEDBACK_ID,
        .zero_point = 0U,
        .inverse = false
    };

    LKMotor_Init(&pitchMotorInstance, &pitch_config);
    RMMotorGroup_Init(&fricMotorGroups[0], &hfdcan2, TASK_FRIC_LOW_GROUP_ID);
    RMMotorGroup_Init(&fricMotorGroups[1], &hfdcan2, TASK_FRIC_HIGH_GROUP_ID);

    can1MotorBindings[0].feedback_id = LK_CAN_FEEDBACK_ID_BASE + TASK_PITCH_MOTOR_ID;
    can1MotorBindings[0].motor = pitchMotor;
    for (uint8_t index = 0U; index < TASK_FRIC_MOTOR_COUNT; ++index)
    {
        RMMotorConfig_t friction_config = {
            .motor_type = RM_C620,
            .mode = RM_CURRENT,
            .motor_id = friction_motor_ids[index],
            .zero_point = 0U,
            .inverse = false
        };
        RMMotorGroup_t *group = friction_config.motor_id < TASK_FRIC_HIGH_GROUP_MIN ?
                                &fricMotorGroups[0] : &fricMotorGroups[1];
        RMMotor_Init(&fricMotorInstances[index], group, &friction_config);
        fricMotors[index] = &fricMotorInstances[index].base;
        can2MotorBindings[index].feedback_id =
            RM_C620_FEEDBACK_ID_BASE + friction_motor_ids[index];
        can2MotorBindings[index].motor = fricMotors[index];
    }
    DMMotor_Init(&yawMotorInstance, &yaw_config);
    can3MotorBindings[0].feedback_id = TASK_YAW_FEEDBACK_ID;
    can3MotorBindings[0].motor = yawMotor;
}

