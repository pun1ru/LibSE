#ifndef _LKMOTOR_H_
#define _LKMOTOR_H_

#include <stdint.h>
#include "can_driver.h"
#include "def_motor.h"
#define LK_STDID 0x140          		//LK电机CAN通信标识符
#define LK_STDID_BASE (LK_STDID+1)	//LK电机CAN通信标识符(寻址)
#define LK_CAN_HANDLE &hcan1    		//LK电机CAN句柄

#define LK_CMD_IQ_CTRL		0xA1
#define LK_CMD_SPEED_CTRL	0xA2
#define LK_CMD_READ_2			0x9C
#define MG_8016_REDUCTION_RATION 6.f

/**
 * \brief 翎控9025电机结构体 
 */
typedef struct 
{
    Motor Super;
    /**
     * \brief 电机ID 
     */
    uint8_t id;
    /**
     * \brief 电机返回数据 
     */

    struct {
        int8_t temperature; //电机温度(℃)
        int16_t iq;         //电机转矩电流(-2048~2048 -> -33A~33A)
        int16_t speed;      //电机转速(dps)
        uint16_t encoder;   //编码器位置(0~16383)
    } __PACKED feedback;
    /**
     * \brief 电机can驱动结构体 
     */
   // can_transmit_t can;
		uint32_t tx_timestamp;
} LK_MF9025_t;

//192.6219f
//#define LK_TORQUE_TO_IQ(torque) Limiter(192.f * torque ,1800)

/**
* \brief 读取电机内置PID参数
* \return 电机控制环路PI参数
*/
// void LK_pidRead(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 写PID参数到RAM中,掉电后丢失
//  * \return 将发送帧完整返回 
//  */
// void LK_pidWrite_RAM( 	can_transmit_t *can, uint8_t id, 
// 						uint8_t angle_kp, uint8_t angle_ki,
// 						uint8_t speed_kp, uint8_t speed_ki,
// 						uint8_t    iq_kp, uint8_t    iq_ki );

// /**
//  * \brief 写PID参数到ROM中，掉电后不丢失 
//  * \return 将发送帧完整返回
//  */
// void LK_pidWrite_ROM(	can_transmit_t *can, uint8_t id, 
// 	                    uint8_t angle_kp, uint8_t angle_ki,
// 						uint8_t speed_kp, uint8_t speed_ki,
// 						uint8_t    iq_kp, uint8_t    iq_ki );

// /**
//  * \brief 读取电机加速度 
//  * \return 电机加速度(1dps/s)
//  */
// void LK_accelRead(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 写入电机加速度(1dps/s) 
//  * \return 将发送帧完整返回
//  */
// void LK_accelWrite(can_transmit_t *can, uint8_t id, int32_t Accel);

// /**
//  * \brief 读取电机编码器当前位置 
//  * \return 电机编码器位置,原始位置,零偏
//  */
// void LK_encoderRead(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 写编码器值到ROM作为电机零点 
//  * \return 将发送帧完整返回
//  */
// void LK_encoderSet(can_transmit_t *can, uint8_t id, uint16_t encoderOffset);

// /**
//  * \brief 读取多圈绝对角度 
//  * \return 电机角度(0.01 degree)
//  */
// void LK_MultiLoop_angleRead(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 读取单圈绝对角度 
//  * \return 电机角度(0.01 degree)
//  */
// void LK_SingleLoop_angleRead(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 读取电机状态1 
//  * \return 电机电压,温度,errorState
//  */
// void LK_stateRead_1(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 清除错误标志位 
//  * \return 电机电压,温度,errorState
//  */
// void LK_errorClear(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 读取电机状态2 
//  * \return 电机温度,转矩电流,转速,编码器位置值
//  */
// void LK_stateRead_2(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 读取电机状态3 
//  * \return 电机温度,相电流
//  */
// void LK_stateRead_3(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 关闭电机,清空指令 
//  * \return 将发送帧完整返回
//  */
// void LK_Motor_off(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 停止电机,不清空指令 
//  * \return 将发送帧完整返回
//  */
// void LK_Motor_stop(can_transmit_t *can, uint8_t id);

// /**
//  * \brief 从停止电机后重新开启电机 
//  * \return 将发送帧完整返回
//  */
// void LK_Motor_run(LK_MF9025_t *Motor);

// /**
//  * \brief 控制转矩电流 
//  * \param[in] iqControl 目标转矩电流(-2048~2048 -> -33~33A)
//  * \return 电机温度,转矩电流,转速,编码器位置值
//  */
//  void LK_iqControl(LK_MF9025_t *Motor, float torque);

// /**
// * \brief 控制电流转速
// * \param[in] speedControl 目标电机转速(0.01dps/LSB) 
// * \return: 电机温度,转矩电流,转速,编码器位置值
// */
// void LK_speedControl(LK_MF9025_t *Motor, int32_t speedControl);
void LK_speedControl_(int id, int32_t speedControl);
// /**
// * \brief 多圈位置闭环控制
// * \param[in] angleControl 目标电机角度(0.01dgree/LSB) 
// * \return: 电机温度,转矩电流,转速,编码器位置值
// */
// void LK_MultiLoop_angleControl(can_transmit_t *can, uint8_t id, int32_t angleControl);

// /**
// * \brief 多圈位置闭环控制(限制上限)
// * \param[in] angleControl 目标电机角度(0.01dgree/LSB) 
// * \param[in] maxSpeed 限制电机最大转速
// * \return: 电机温度,转矩电流,转速,编码器位置值
// */
// void LK_MultiLoop_angleControl_limited(can_transmit_t *can, uint8_t id, uint16_t maxSpeed, int32_t angleControl);

// /**
// * \brief 单圈位置闭环控制
// * \param[in] angleControl 目标电机角度(0~35999, 0.01dgree/LSB) 
// * \param[in] spinDir 电机转动方向 0-顺时针 1-逆时针
// * \return: 电机温度,转矩电流,转速,编码器位置值
// */
// void LK_SingleLoop_angleControl(can_transmit_t *can, uint8_t id, uint8_t spinDir, uint16_t angleControl);

// /**
// * \brief 单圈位置闭环控制(限制上限)
// * \param[in] angleControl 目标电机角度(0~35999, 0.01dgree/LSB) 
// * \param[in] spinDir 电机转动方向 0-顺时针 1-逆时针
// * \param[in] maxSpeed 限制电机最大转速
// * \return: 电机温度,转矩电流,转速,编码器位置值
// */
// void LK_SingleLoop_angleControl_limited(can_transmit_t *can, uint8_t id, uint8_t spinDir, uint16_t maxSpeed, uint16_t angleControl);

// /**
// * \brief 增量位置闭环控制
// * \param[in] angleIncrement 目标电机角度增量(0.01dgree/LSB) 
// * \return: 电机温度,转矩电流,转速,编码器位置值
// */
// void LK_Increment_angleControl(can_transmit_t *can, uint8_t id, int32_t angleIncrement);

// /**
// * \brief 增量位置闭环控制(限制上限)
// * \param[in] maxSpeed 限制电机最大转速
// * \param[in] angleIncrement 目标电机角度增量(0.01dgree/LSB) 
// * \return: 电机温度,转矩电流,转速,编码器位置值
// */
// void LK_Increment_angleControl_limited(can_transmit_t *can, uint8_t id, uint16_t maxSpeed, int32_t angleIncrement);

/// @brief still couldn't understand? idiot! 
/// @param torque1 
/// @param torque2 
/// @param torque3 
/// @param torque4 
void xLK_iqControl(float torque1, float torque2, float torque3, float torque4);
#endif
