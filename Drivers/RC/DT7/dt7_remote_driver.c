#include <math.h>
#include <stdlib.h>

#include "dt7_remote_driver.h"

void DT7RemoteRecEnable(UART_HandleTypeDef* huart, uint8_t* uart_rec_buffer)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rec_buffer, MAX_RECEIVE_BUFFER_LENGTH);	
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭DMA半满中断
}

void DT7RawDataUpdate(DT7RecData* rec_data, const uint8_t* rec_buffer)
{  
    rec_data->ch0 = (( rec_buffer[0]       | (rec_buffer[1] << 8)) & 0x07ff);                          // Channel 0
    rec_data->ch1 = (((rec_buffer[1] >> 3) | (rec_buffer[2] << 5)) & 0x07ff);                          // Channel 1
    rec_data->ch2 = (((rec_buffer[2] >> 6) | (rec_buffer[3] << 2) | (rec_buffer[4] << 10)) & 0x07ff); // Channel 2
    rec_data->ch3 = (((rec_buffer[4] >> 1) | (rec_buffer[5] << 7)) & 0x07ff);                          // Channel 3
    rec_data->ch4 = ((rec_buffer[16] | (rec_buffer[17] << 8)) & 0x07ff);
		
    rec_data->switch_left  = ((rec_buffer[5] >> 4) & 0x000C) >> 2;                                      // Switch left
    rec_data->switch_right = ((rec_buffer[5] >> 4) & 0x0003);                                           // Switch right

    rec_data->Mouse.x = rec_buffer[6]  | (rec_buffer[7]  << 8);   																			// Mouse X axis
    rec_data->Mouse.y = rec_buffer[8]  | (rec_buffer[9]  << 8);                                        // Mouse Y axis
    rec_data->Mouse.z = rec_buffer[10] | (rec_buffer[11] << 8);                                        // Mouse Z axis

    rec_data->Mouse.press_left  = rec_buffer[12];                                                       // Mouse Left Pressed
    rec_data->Mouse.press_right = rec_buffer[13];                                                       // Mouse Right Pressed
    rec_data->KeyBoard.key_code = rec_buffer[14] | (rec_buffer[15] << 8);                             // KeyBoard value
}

/**
  * \brief  获取遥控器拨杆值(相对)
  * \param[in]  rec_data 遥控器接收原始数据结构体
  * \param[in]  channel_num  拨杆通道标志位 
  * \return 通道拨杆值  -660~660
  */
static int16_t DT7GetChannelVal(const DT7RecData* rec_data, uint8_t channel_num)
{
	switch(channel_num)
	{
	case DT7_RC_CH0:
		if(abs(((*rec_data).ch0 - DT7_RC_CH_VALUE_OFFSET)) < 10)
			return 0;
		else
			return ((*rec_data).ch0 - DT7_RC_CH_VALUE_OFFSET);
	case DT7_RC_CH1:
		if(abs(((*rec_data).ch1 - DT7_RC_CH_VALUE_OFFSET)) < 10)
			return 0;
		else
			return ((*rec_data).ch1 - DT7_RC_CH_VALUE_OFFSET);
	case DT7_RC_CH2:
		if(abs(((*rec_data).ch2 - DT7_RC_CH_VALUE_OFFSET)) < 10)
			return 0;
		else
			return ((*rec_data).ch2 - DT7_RC_CH_VALUE_OFFSET);
	case DT7_RC_CH3:
		if(abs(((*rec_data).ch3 - DT7_RC_CH_VALUE_OFFSET)) < 10)
			return 0;
		else
			return ((*rec_data).ch3 - DT7_RC_CH_VALUE_OFFSET);
	case DT7_RC_CH4:
		if(abs(((*rec_data).ch4 - DT7_RC_CH_VALUE_OFFSET)) < 10)
			return 0;
		else
			return ((*rec_data).ch4 - DT7_RC_CH_VALUE_OFFSET);
	default:
		return DT7_RC_CH_VALUE_OFFSET;
  }
}

/**
  * \brief  获取遥控器拨码值
  * \param[in]  switch_num  拨码标志位
  * \return 拨码值(上-1 中-3 下-2)
  */
static uint8_t DT7GeSwitchVal(const DT7RecData* rec_data, uint8_t switch_num)
{
	switch (switch_num)
	{
	case DT7_RC_SW_RIGHT:
		return (*rec_data).switch_right;
	case DT7_RC_SW_Left:
		return (*rec_data).switch_left;
	default:
		return DT7_RC_SW_MID;
	}
}

/**
  * \brief  获取鼠标速度值
  * \param[in] xyz  鼠标轴标识位
  * \return 鼠标速度(32767 ~ -32768)
  */
static int16_t DT7GetMouseSpeed(const DT7RecData* rec_data, uint8_t xyz)
{
	switch (xyz)
	{
	case DT7_MOUSE_X:
		return (*rec_data).Mouse.x;
	case DT7_MOUSE_Y:
		return (*rec_data).Mouse.y;
	case DT7_MOUSE_Z:
		return (*rec_data).Mouse.z;
	default:
		return DT7_MOUSE_SPEED_OFFSET;
	}
}

/**
  * \brief  获取鼠标按键
  * \param[in]  button 鼠标按键标识位
  * \return 相应键位是否按下 0-未按下 1-按下
  */
static uint8_t DT7GetMousePressed(const DT7RecData* rec_data, uint8_t button)
{
	switch (button)
	{
	case DT7_MOUSE_LEFT:
		return (*rec_data).Mouse.press_left;
	case DT7_MOUSE_RIGHT:
		return (*rec_data).Mouse.press_right;
	default:
		return DT7_MOUSE_PRESSED_OFFSET;
	}
}

/**
  * \brief  获取键盘按键
  * \param[in]  key 键盘键位标识位
  * \return 相应键位是否按下 0-未按下 1-按下
  */
static uint8_t DT7GetKeyBoardVal(const DT7RecData* rec_data,uint8_t key)
{
	switch (key)
	{
		case DT7_KEY_W:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_W)&&1);
		case DT7_KEY_S:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_S)&&1);
		case DT7_KEY_A:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_A)&&1);
		case DT7_KEY_D:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_D)&&1);  
		case DT7_KEY_SHIFT:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_SHIFT)&&1);
		case DT7_KEY_CTRL:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_CTRL)&&1); 
		case DT7_KEY_Q:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_Q)&&1);
		case DT7_KEY_E:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_E)&&1);	
		case DT7_KEY_R:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_R)&&1);
		case DT7_KEY_F:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_F)&&1);
		case DT7_KEY_G:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_G)&&1);
		case DT7_KEY_Z:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_Z)&&1);
		case DT7_KEY_X:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_X)&&1);
		case DT7_KEY_C:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_C)&&1);
		case DT7_KEY_V:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_V)&&1);
		case DT7_KEY_B:
			return (((*rec_data).KeyBoard.key_code & DT7_KEY_PRESSED_OFFSET_B)&&1);
		default:
			return DT7_KEY_OFFSET;
	}
}

/**
 * \brief 根据DT7遥控器原始数据得到指令
 */
void DT7DataProcess(DT7CmdData* cmd_data, const DT7RecData* rec_data)
{
	cmd_data->ch0 = DT7GetChannelVal(rec_data, DT7_RC_CH0);
	cmd_data->ch1 = DT7GetChannelVal(rec_data, DT7_RC_CH1);
	cmd_data->ch2 = DT7GetChannelVal(rec_data, DT7_RC_CH2);
	cmd_data->ch3 = DT7GetChannelVal(rec_data, DT7_RC_CH3);
	cmd_data->ch4 = DT7GetChannelVal(rec_data, DT7_RC_CH4);
	
	cmd_data->switch_left = DT7GeSwitchVal(rec_data, DT7_RC_SW_Left);
	cmd_data->switch_right = DT7GeSwitchVal(rec_data, DT7_RC_SW_RIGHT);
	
	cmd_data->PCKeyBoard.level_key_W = DT7GetKeyBoardVal(rec_data, DT7_KEY_W);
	cmd_data->PCKeyBoard.level_key_S = DT7GetKeyBoardVal(rec_data, DT7_KEY_S);
	cmd_data->PCKeyBoard.level_key_A = DT7GetKeyBoardVal(rec_data, DT7_KEY_A);
	cmd_data->PCKeyBoard.level_key_D = DT7GetKeyBoardVal(rec_data, DT7_KEY_D);
	cmd_data->PCKeyBoard.level_key_SHIFT = DT7GetKeyBoardVal(rec_data, DT7_KEY_SHIFT);
	cmd_data->PCKeyBoard.level_key_CTRL = DT7GetKeyBoardVal(rec_data, DT7_KEY_CTRL);
	cmd_data->PCKeyBoard.level_key_Q = DT7GetKeyBoardVal(rec_data, DT7_KEY_Q);
	cmd_data->PCKeyBoard.level_key_E = DT7GetKeyBoardVal(rec_data, DT7_KEY_E);
	cmd_data->PCKeyBoard.level_key_R = DT7GetKeyBoardVal(rec_data, DT7_KEY_R);
	cmd_data->PCKeyBoard.level_key_F = DT7GetKeyBoardVal(rec_data, DT7_KEY_F);
	cmd_data->PCKeyBoard.level_key_G = DT7GetKeyBoardVal(rec_data, DT7_KEY_G);
	cmd_data->PCKeyBoard.level_key_Z = DT7GetKeyBoardVal(rec_data, DT7_KEY_Z);
	cmd_data->PCKeyBoard.level_key_X = DT7GetKeyBoardVal(rec_data, DT7_KEY_X);
	cmd_data->PCKeyBoard.level_key_C = DT7GetKeyBoardVal(rec_data, DT7_KEY_C);
	cmd_data->PCKeyBoard.level_key_V = DT7GetKeyBoardVal(rec_data, DT7_KEY_V);
	cmd_data->PCKeyBoard.level_key_B = DT7GetKeyBoardVal(rec_data, DT7_KEY_B);
	
	cmd_data->PCMouse.press_left = DT7GetMousePressed(rec_data, DT7_MOUSE_LEFT);
	cmd_data->PCMouse.press_right = DT7GetMousePressed(rec_data, DT7_MOUSE_RIGHT);
	
	cmd_data->PCMouse.x = DT7GetMouseSpeed(rec_data, DT7_MOUSE_X);
	cmd_data->PCMouse.y = DT7GetMouseSpeed(rec_data, DT7_MOUSE_Y);
	cmd_data->PCMouse.z = DT7GetMouseSpeed(rec_data, DT7_MOUSE_Z);
	
}
