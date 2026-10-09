#include "LKMotor.h"
#include "cmsis_os.h"
#include "data_process.h"
#include "Safe.h"
//extern Mit_Motor_t HipMotor[4];
/**
* \brief 读取电机内置PID参数
* \return 电机控制环路PI参数
*/
void LK_pidRead(can_transmit_t *can, uint8_t id)
{										
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x30;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
 * \brief 写PID参数到RAM中,掉电后丢失
 * \return 将发送帧完整返回 
 */
void LK_pidWrite_RAM( 	can_transmit_t *can, uint8_t id, 
						uint8_t angle_kp, uint8_t angle_ki,
						uint8_t speed_kp, uint8_t speed_ki,
						uint8_t    iq_kp, uint8_t    iq_ki )
{										
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x31;		
	can->data[1] = 0;
	can->data[2] = angle_kp;
	can->data[3] = angle_ki;
	can->data[4] = speed_kp;
	can->data[5] = speed_ki;
	can->data[6] = iq_kp;
	can->data[7] = iq_ki;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
 * \brief 写PID参数到ROM中，掉电后不丢失 
 * \return 将发送帧完整返回
 */
void LK_pidWrite_ROM(	can_transmit_t *can, uint8_t id, 
	                    uint8_t angle_kp, uint8_t angle_ki,
						uint8_t speed_kp, uint8_t speed_ki,
						uint8_t    iq_kp, uint8_t    iq_ki )
{										
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x32;		
	can->data[1] = 0;
	can->data[2] = angle_kp;
	can->data[3] = angle_ki;
	can->data[4] = speed_kp;
	can->data[5] = speed_ki;
	can->data[6] = iq_kp;
	can->data[7] = iq_ki;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
 * \brief 读取电机加速度 
 * \return 电机加速度(1dps/s)
 */
void LK_accelRead(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x33;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
 * \brief 写入电机加速度(1dps/s) 
 * \return 将发送帧完整返回
 */
void LK_accelWrite(can_transmit_t *can, uint8_t id, int32_t Accel)
{										
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x34;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = *( uint8_t*)(&Accel);
	can->data[5] = *((uint8_t*)(&Accel)+1);
	can->data[6] = *((uint8_t*)(&Accel)+2);
	can->data[7] = *((uint8_t*)(&Accel)+3);

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
 * \brief 读取电机编码器当前位置 
 * \return 电机编码器位置,原始位置,零偏
 */
void LK_encoderRead(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x90;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	////can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
 * \brief 写编码器值到ROM作为电机零点 
 * \return 将发送帧完整返回
 */
void LK_encoderSet(can_transmit_t *can, uint8_t id, uint16_t encoderOffset)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x91;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = * (uint8_t*)(&encoderOffset);
	can->data[7] = *((uint8_t*)(&encoderOffset)+1);

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
 * \brief 读取多圈绝对角度 
 * \return 电机角度(0.01 degree)
 */
void LK_MultiLoop_angleRead(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x92;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
 * \brief 读取单圈绝对角度 
 * \return 电机角度(0.01 degree)
 */
void LK_SingleLoop_angleRead(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x94;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
 * \brief 读取电机状态1 
 * \return 电机电压,温度,errorState
 */
void LK_stateRead_1(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x9A;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
 * \brief 清除错误标志位 
 * \return 电机电压,温度,errorState
 */
void LK_errorClear(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x9B;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
 * \brief 读取电机状态2 
 * \return 电机温度,转矩电流,转速,编码器位置值
 */
void LK_stateRead_2(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x9C;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
 * \brief 读取电机状态3 
 * \return 电机温度,相电流
 */
void LK_stateRead_3(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x9D;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
 * \brief 关闭电机,清空指令 
 * \return 将发送帧完整返回
 */
void LK_Motor_off(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x80;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
 * \brief 停止电机,不清空指令 
 * \return 将发送帧完整返回
 */
void LK_Motor_stop(can_transmit_t *can, uint8_t id)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0x81;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = 0;
	can->data[5] = 0;
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);	
}

/**
 * \brief 从停止电机后重新开启电机 
 * \return 将发送帧完整返回
 */
// void LK_Motor_run(LK_MF9025_t *Motor)
// {											
// 	//Motor->can.stdid = LK_STDID + Motor->id;					

// 	Motor->can.data[0] = 0x88;		
// 	Motor->can.data[1] = 0;
// 	Motor->can.data[2] = 0;
// 	Motor->can.data[3] = 0;
// 	Motor->can.data[4] = 0;
// 	Motor->can.data[5] = 0;
// 	Motor->can.data[6] = 0;
// 	Motor->can.data[7] = 0;
	
// 	Motor->tx_timestamp = xTaskGetTickCount();
// 	//can_transmit(LK_CAN_HANDLE, LK_STDID + Motor->id, Motor->can.data);	
// }

// /**
//  * \brief 控制转矩电流 
//  * \param[in] iqControl 目标转矩电流(-2048~2048 -> -33~33A)
//  * \return 电机温度,转矩电流,转速,编码器位置值
//  */
// void LK_iqControl(LK_MF9025_t *Motor, float torque)
// {	
// 	//Motor->can.stdid = LK_STDID + Motor->id;	
	
// 	int16_t iqControl = - torque * 40;
// 	Motor->can.data[0] = 0xA1;		
// 	Motor->can.data[1] = 0;
// 	Motor->can.data[2] = 0;
// 	Motor->can.data[3] = 0;
// 	Motor->can.data[4] = *( uint8_t *)(&iqControl);
// 	Motor->can.data[5] = *((uint8_t *)(&iqControl)+1);
// 	Motor->can.data[6] = 0;
// 	Motor->can.data[7] = 0;
	
// 	Motor->tx_timestamp = xTaskGetTickCount();
// 	//can_transmit(LK_CAN_HANDLE,LK_STDID + Motor->id, Motor->can.data);		
// }
void xLK_iqControl(float torque1, float torque2, float torque3, float torque4)
{	
	//Motor->can.stdid = LK_STDID + Motor->id;	
	int16_t iqControl1 = - torque1 * 40;
	int16_t iqControl2 = - torque2 * 40;
	int16_t iqControl3 = - torque3 * 40;
	int16_t iqControl4 = - torque4 * 40;
	iqControl1 = Limiter(iqControl1 , 2000);
	iqControl2 = Limiter(iqControl2 , 2000);
	iqControl3 = Limiter(iqControl3 , 2000);
	iqControl4 = Limiter(iqControl4 , 2000);
	uint8_t adata[8];
	adata[0] = *( uint8_t *)(&iqControl1);
	adata[1] = *((uint8_t *)(&iqControl1)+1);
	adata[2] = *( uint8_t *)(&iqControl2);
	adata[3] = *((uint8_t *)(&iqControl2)+1);
	adata[4] = *( uint8_t *)(&iqControl3);
	adata[5] = *((uint8_t *)(&iqControl3)+1);
	adata[6] = *( uint8_t *)(&iqControl4);
	adata[7] = *((uint8_t *)(&iqControl4)+1);
	fdcanx_send_data(&hfdcan1, 0x280, adata, 8);
	//can_transmit(LK_CAN_HANDLE, 0x280, adata);		
}
/**
* \brief 控制电流转速
* \param[in] speedControl 目标电机转速(0.01dps/LSB) 
* \return: 电机温度,转矩电流,转速,编码器位置值
*/ 

void LK_speedControl_(int id, int32_t speedControl)
 {											
	uint8_t adata[8];
	adata[0] = 0xA2;
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *( uint8_t *)(&speedControl);
	adata[5] = *((uint8_t *)(&speedControl)+1);
	adata[6] = *((uint8_t *)(&speedControl)+2);
	adata[7] = *((uint8_t *)(&speedControl)+2);
	fdcanx_send_data(&hfdcan1, id, adata, 8);
 }
// void LK_speedControl(LK_MF9025_t *Motor, int32_t speedControl)
// {											

// 	Motor->can.data[0] = 0xA2;		
// 	Motor->can.data[1] = 0;
// 	Motor->can.data[2] = 0;
// 	Motor->can.data[3] = 0;
// 	Motor->can.data[4] = *( uint8_t *)(&speedControl);
// 	Motor->can.data[5] = *((uint8_t *)(&speedControl)+1);
// 	Motor->can.data[6] = *((uint8_t *)(&speedControl)+2);
// 	Motor->can.data[7] = *((uint8_t *)(&speedControl)+2);

// 	can_transmit(LK_CAN_HANDLE,LK_STDID + Motor->id, Motor->can.data);		
// }

/**
* \brief 多圈位置闭环控制
* \param[in] angleControl 目标电机角度(0.01dgree/LSB) 
* \return: 电机温度,转矩电流,转速,编码器位置值
*/
void LK_MultiLoop_angleControl(can_transmit_t *can, uint8_t id, int32_t angleControl)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0xA3;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = *( uint8_t *)(&angleControl);
	can->data[5] = *((uint8_t *)(&angleControl)+1);
	can->data[6] = *((uint8_t *)(&angleControl)+2);
	can->data[7] = *((uint8_t *)(&angleControl)+3);

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
* \brief 多圈位置闭环控制(限制上限)
* \param[in] angleControl 目标电机角度(0.01dgree/LSB) 
* \param[in] maxSpeed 限制电机最大转速
* \return: 电机温度,转矩电流,转速,编码器位置值
*/
void LK_MultiLoop_angleControl_limited(can_transmit_t *can, uint8_t id, uint16_t maxSpeed, int32_t angleControl)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0xA4;		
	can->data[1] = 0;
	can->data[2] = *( uint8_t *)(&maxSpeed);
	can->data[3] = *((uint8_t *)(&maxSpeed)+1);
	can->data[4] = *( uint8_t *)(&angleControl);
	can->data[5] = *((uint8_t *)(&angleControl)+1);
	can->data[6] = *((uint8_t *)(&angleControl)+2);
	can->data[7] = *((uint8_t *)(&angleControl)+3);

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
* \brief 单圈位置闭环控制
* \param[in] angleControl 目标电机角度(0~35999, 0.01dgree/LSB) 
* \param[in] spinDir 电机转动方向 0-顺时针 1-逆时针
* \return: 电机温度,转矩电流,转速,编码器位置值
*/
void LK_SingleLoop_angleControl(can_transmit_t *can, uint8_t id, uint8_t spinDir, uint16_t angleControl)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0xA5;		
	can->data[1] = spinDir;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = *( uint8_t *)(&angleControl);
	can->data[5] = *((uint8_t *)(&angleControl)+1);
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
* \brief 单圈位置闭环控制(限制上限)
* \param[in] angleControl 目标电机角度(0~35999, 0.01dgree/LSB) 
* \param[in] spinDir 电机转动方向 0-顺时针 1-逆时针
* \param[in] maxSpeed 限制电机最大转速
* \return: 电机温度,转矩电流,转速,编码器位置值
*/
void LK_SingleLoop_angleControl_limited(can_transmit_t *can, uint8_t id, uint8_t spinDir, uint16_t maxSpeed, uint16_t angleControl)
{										
	can->stdid = LK_STDID + id;					

	can->data[0] = 0xA6;		
	can->data[1] = spinDir;
	can->data[2] = *( uint8_t *)(&maxSpeed);
	can->data[3] = *((uint8_t *)(&maxSpeed)+1);
	can->data[4] = *( uint8_t *)(&angleControl);
	can->data[5] = *((uint8_t *)(&angleControl)+1);
	can->data[6] = 0;
	can->data[7] = 0;

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

/**
* \brief 增量位置闭环控制
* \param[in] angleIncrement 目标电机角度增量(0.01dgree/LSB) 
* \return: 电机温度,转矩电流,转速,编码器位置值
*/
void LK_Increment_angleControl(can_transmit_t *can, uint8_t id, int32_t angleIncrement)
{										
	can->stdid = LK_STDID + id;					

	can->data[0] = 0xA7;		
	can->data[1] = 0;
	can->data[2] = 0;
	can->data[3] = 0;
	can->data[4] = *((uint8_t *)(&angleIncrement));
	can->data[5] = *((uint8_t *)(&angleIncrement)+1);
	can->data[6] = *((uint8_t *)(&angleIncrement)+2);
	can->data[7] = *((uint8_t *)(&angleIncrement)+3);

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}


/**
* \brief 增量位置闭环控制(限制上限)
* \param[in] maxSpeed 限制电机最大转速
* \param[in] angleIncrement 目标电机角度增量(0.01dgree/LSB) 
* \return: 电机温度,转矩电流,转速,编码器位置值
*/
void LK_Increment_angleControl_limited(can_transmit_t *can, uint8_t id, uint16_t maxSpeed, int32_t angleIncrement)
{											
	can->stdid = LK_STDID + id;					

	can->data[0] = 0xA8;		
	can->data[1] = 0;
	can->data[2] = *((uint8_t *)(&maxSpeed));
	can->data[3] = *((uint8_t *)(&maxSpeed)+1);
	can->data[4] = *((uint8_t *)(&angleIncrement));
	can->data[5] = *((uint8_t *)(&angleIncrement)+1);
	can->data[6] = *((uint8_t *)(&angleIncrement)+2);
	can->data[7] = *((uint8_t *)(&angleIncrement)+3);

	//can_transmit(LK_CAN_HANDLE, can->stdid, can->data);		
}

void LK_Motor_Feedback(LK_MF9025_t * Motor){
	Motor->Super.torque = - Motor->feedback.iq / 40.f;
	Motor->Super.speed = - Motor->feedback.speed * 3.14159265358979f / (180.f * MG_8016_REDUCTION_RATION ) ;
	Motor->Super.angle = - Motor->feedback.encoder * 2 * 3.14159265358979f / 65536;
	Motor->Super.temperature = Motor->feedback.temperature;
	Motor->Super.TimeStamp = xTaskGetTickCountFromISR();
	Motor->Super.FrameCounter++;
}

/**
 * \brief CAN FIFO0 接收中断
 * \param[in] hcan CAN总线控制句柄 
 */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef RxHeader;                                      
//     uint8_t aData[8];                                                  
//     HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, aData);
//     uint8_t cmd = *(uint8_t*)(aData);
// 		if(RxHeader.StdId - LK_STDID_BASE <= 3)
// 		{
// 			switch (cmd)
// 				{
// 				case LK_CMD_READ_2:
// 				case LK_CMD_SPEED_CTRL:
// 				case LK_CMD_IQ_CTRL:
// 				if(RxHeader.StdId >= LK_STDID_BASE && RxHeader.StdId <= LK_STDID_BASE + 3){
// 						memcpy(&HipMotor[RxHeader.StdId - LK_STDID_BASE].feedback, aData + 1, 7);
// 						LK_Motor_Feedback(&HipMotor[RxHeader.StdId - LK_STDID_BASE]);
// 				}
// 				}
// 		}
// }
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//  {
//		FDCAN_RxHeaderTypeDef RxHeader;
//		uint8_t aData[8];
//		HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &RxHeader, aData);
//		if(RxHeader.IsFilterMatchingFrame == 0){
//			uint8_t cmd = *(uint8_t*)(aData);
//			switch (cmd)
//			{
//					case LK_CMD_READ_2:
//					case LK_CMD_SPEED_CTRL:
//					case LK_CMD_IQ_CTRL:
//						if(RxHeader.Identifier >= LK_STDID_BASE && RxHeader.Identifier <= LK_STDID_BASE + 3){
//								memcpy(&HipMotor[RxHeader.Identifier - LK_STDID_BASE].feedback, aData + 1, 7);
//								LK_Motor_Feedback(&HipMotor[RxHeader.Identifier - LK_STDID_BASE]);
//				}
//			}
//		}
//		else{
//			//ERRORCUR();
//		}
//  }
//}