#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

/* 任务启动时先初始化云台电机、CAN 反馈绑定表和云台对象，再进入控制循环 */
void GimbalTask(void *argument);

#endif
