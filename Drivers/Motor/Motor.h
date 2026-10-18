#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

typedef enum {      //电机控制模式 Control mode
    MODE_POSITION,  ///< 位置控制模式 Position control mode
    MODE_VELOCITY,  ///< 速度控制模式 Velocity control mode
    MODE_TORQUE,    ///< 力矩控制模式 Torque control mode
    MODE_CURRENT,   ///< 电流控制模式 Current control mode
    MODE_MIT,       ///< MIT 控制模式 MIT control mode
} ControlMode_t;

typedef enum {
    MOTOR_DISABLED,
    MOTOR_ENABLED,
    MOTOR_FAULT,
    MOTOR_OVERVOLTAGE,
    MOTOR_UNDERVOLTAGE,
    MOTOR_OVERCURRENT,
    MOTOR_MOS_OVERTEMP,
    MOTOR_COIL_OVERTEMP,
    MOTOR_COMM_LOST,
    MOTOR_OVERLOAD,
    MOTOR_NO_FEEDBACK
} MotorStatus_t;

typedef struct {
    ControlMode_t control_mode;
    float position;
    float velocity;
    float torque;
    float current;
    float kp;
    float kd;
} MotorCmd_t;

#define MOTOR_PI              3.141592653589793f
#define MOTOR_TWO_PI          (2.0f * MOTOR_PI)
#define MOTOR_RAD_TO_DEG      (180.0f / MOTOR_PI)
#define MOTOR_DEG_TO_RAD      (MOTOR_PI / 180.0f)
#define MOTOR_RADS_TO_RPM     (60.0f / MOTOR_TWO_PI)
#define MOTOR_RPM_TO_RADS     (MOTOR_TWO_PI / 60.0f)

typedef struct {
    uint32_t timestamp;                  ///< Feedback function call count
    uint8_t error_id;                    ///< 电机错误码 Motor error code
    MotorStatus_t status;                ///< 电机状态 Motor status
    uint16_t encoder;                   // 电机反馈的原始编码器值
    float abs_angle_rad;  ///< 相对零点的单圈角度 (-pi, pi] rad
    float abs_angle_deg;  ///< 相对零点的单圈角度 (-180, 180] deg
    float multi_angle_rad; ///< 多圈角度 Multi-turn angle, rad
    float velocity_rpm;                  ///< 转速 Speed, rpm
    float omega_rads;                    ///< 角速度 Angular velocity, rad/s
    float torque_nm;                     ///< 扭矩 Torque, N*m
    float temp;                          ///< 温度 Temperature
    float current;                       // Motor current in amperes
} Feedback_t;

typedef struct Motor_t Motor_t;
typedef struct {
    void (*enable)(Motor_t *motor);
    void (*disable)(Motor_t *motor);
    void (*protect)(Motor_t *motor);
    void (*sendcommand)(Motor_t *motor);
    void (*sendgroupcommand)(Motor_t *motor);
    void (*requestfeedback)(Motor_t *motor);
    void (*clearerror)(Motor_t *motor);
    void (*getfeedback)(Motor_t *motor, const uint8_t *data, uint16_t rec_id);
} MotorOps_t;

struct Motor_t{
    Feedback_t feedback;
    MotorCmd_t motorcmd;
    MotorOps_t motorops;
    FDCAN_HandleTypeDef *can;           // 电机所在 CAN 总线
    uint16_t zero_point;                // 人为标定的原始编码器零点
    bool inverse;                       // 反转电流或力矩命令方向
    float gear_ratio;                   // 减速比 = 转子转速 / 输出轴转速，无减速箱为 1，由各电机驱动填写
    float torque_constant;              // 电流到力矩的系数 N*m/A，口径见各驱动，0 表示该驱动不提供
};

/* 基类构造：子类 Init 在 memset 之后调用，再填写各自的 motorops */
void Motor_BaseInit(Motor_t *motor, FDCAN_HandleTypeDef *can,
                    uint16_t zero_point, bool inverse, float gear_ratio);

/* 由相对零点的单圈角度同时填写 abs_angle_rad 与 abs_angle_deg，折算到 [-pi, pi] */
void Motor_SetAbsAngle(Feedback_t *feedback, float angle_rad);

#endif
