#include "DMJ4310.h"


int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
    
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/// @brief DM电机MIT控制模式
void DM_MITControl(float _pos, float _vel, float _KP, float _KD, float _torq, uint8_t* adata)
{	
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	adata[0] = (pos_tmp >> 8);
	adata[1] = pos_tmp;
	adata[2] = (vel_tmp >> 4);
	adata[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	adata[4] = kp_tmp;
	adata[5] = (kd_tmp >> 4);
	adata[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	adata[7] = tor_tmp;
}

void MIT_Clear(MIT_Ctrl_t *mit)
{
	mit->p   = 0.0f;
	mit->v   = 0.0f;
	mit->Kp  = 0.0f;
	mit->Kd  = 0.0f;
	mit->Tff = 0.0f;
}

void MIT_SetParam(MIT_Ctrl_t *mit, float p, float v, float Kp, float Kd, float Tff)
{
	mit->p   = p;
	mit->v   = v;
	mit->Kp  = Kp;
	mit->Kd  = Kd;
	mit->Tff = Tff;
}

void DM_MITControl_Send(FDCAN_HandleTypeDef* hcan, uint16_t id,float _pos, float _vel, float _KP, float _KD, float _torq){
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	uint8_t adata[8];
	adata[0] = (pos_tmp >> 8);
	adata[1] = pos_tmp;
	adata[2] = (vel_tmp >> 4);
	adata[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	adata[4] = kp_tmp;
	adata[5] = (kd_tmp >> 4);
	adata[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	adata[7] = tor_tmp;
	CANTransmit_U8(hcan, id, adata);
}
/*电机什么控制模式？*/
void ctrl_motor2(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{
	uint8_t *pbuf,*vbuf;
	pbuf=(uint8_t*)&_pos;
	vbuf=(uint8_t*)&_vel;
	
  uint8_t Data[8];

	Data[0] = *pbuf;
	Data[1] = *(pbuf+1);
	Data[2] = *(pbuf+2);
	Data[3] = *(pbuf+3);
	Data[4] = *vbuf;
	Data[5] = *(vbuf+1);
	Data[6] = *(vbuf+2);
	Data[7] = *(vbuf+3);
	
	CANTransmit_U8(hcan, id, Data);
}	
/// @brief DM电机速度控制模式 
void DM_SpeedControl(float _vel, uint8_t* adata)
{
	uint8_t *vbuf;
	vbuf=(uint8_t*)&_vel;
	
	adata[0] = *vbuf;
	adata[1] = *(vbuf+1);
	adata[2] = *(vbuf+2);
	adata[3] = *(vbuf+3);
	adata[4] = (uint8_t)(0);
	adata[5] = (uint8_t)(0);
	adata[6] = (uint8_t)(0);
	adata[7] = (uint8_t)(0);
}	

/// @brief DM电机使能
void DM_Enable(uint8_t* adata)
{	
	adata[0] = 0xFF;
	adata[1] = 0xFF;
	adata[2] = 0xFF;
	adata[3] = 0xFF;
	adata[4] = 0xFF;
	adata[5] = 0xFF;
	adata[6] = 0xFF;
	adata[7] = 0xFC;
}	

/// @brief DM电机失能
void DM_Disable(uint8_t* adata)
{
	adata[0] = 0xFF;
	adata[1] = 0xFF;
	adata[2] = 0xFF;
	adata[3] = 0xFF;
	adata[4] = 0xFF;
	adata[5] = 0xFF;
	adata[6] = 0xFF;
	adata[7] = 0xFD;
}	

/**
 * @brief DMJ4310????
 */
void ctrl_motor(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
   uint8_t Data[8];

	Data[0] = (pos_tmp >> 8);
	Data[1] = pos_tmp;
	Data[2] = (vel_tmp >> 4);
	Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	Data[4] = kp_tmp;
	Data[5] = (kd_tmp >> 4);
	Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	Data[7] = tor_tmp;
	
	CANTransmit_U8(hcan, id, Data);
}	

void start_motor(FDCAN_HandleTypeDef* hcan, uint16_t id)
{
	uint8_t  Data[8];
	
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	Data[7] = 0xFC;
	
	CANTransmit_U8(hcan, id, Data);
}	
void clear_error(FDCAN_HandleTypeDef* hcan, uint16_t id)
{
	uint8_t  Data[8];
	
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	Data[7] = 0xFB;
	
	CANTransmit_U8(hcan, id, Data);
}	

void lock_motor(FDCAN_HandleTypeDef* hcan, uint16_t id)
{
    uint8_t  Data[8];

    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFD;

    CANTransmit_U8(hcan, id, Data);
}
	