#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

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

typedef struct {
    uint32_t timestamp;                  ///< Feedback function call count
    uint8_t error_id;                    ///< 电机错误码 Motor error code
    MotorStatus_t status;                ///< 电机状态 Motor status
    uint16_t encoder;                   // 电机反馈的原始编码器值
    float abs_angle;  ///< 归一化到单圈的角度 Single-turn angle
    float multi_angle; ///< 多圈角度 Multi-turn angle
    float velocity;                      ///< 转速 Speed
    float omega;                         ///< 角速度 Angular velocity
    float torque;                        ///< 扭矩 Torque
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
    uint16_t zero_point;                // 人为标定的原始编码器零点
    bool inverse;                       // 反转电流或力矩命令方向
};

#endif
