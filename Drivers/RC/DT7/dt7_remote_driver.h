#ifndef _DT7_REMOTE_DRIVER_H_
#define _DT7_REMOTE_DRIVER_H_

#include "usart.h"
#include "stdint.h"

#define MAX_RECEIVE_BUFFER_LENGTH 300 //define maximum lehgth of receive from remote once

#define DT7_RC_FRAME_LEN        18U         //Length of received data per frame
#define DT7_RC_FRAME_LEN_BACK   7U          //Extra length for stability
#define DT7_RC_CH_MAX_RELATIVE 660.0f       
#define VT3_RC_FRAME_LEN  21U

/** DT7_RC Channel Config **/
#define DT7_RC_CH_VALUE_MIN ((uint16_t)364 )
#define DT7_RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define DT7_RC_CH_VALUE_MAX ((uint16_t)1684)
	
#define DT7_RC_CH0    ((uint8_t)0)
#define DT7_RC_CH1    ((uint8_t)1)
#define DT7_RC_CH2    ((uint8_t)2)
#define DT7_RC_CH3    ((uint8_t)3)
#define DT7_RC_CH4    ((uint8_t)4)


/** DT7_RC Switch Config**/	
#define DT7_RC_SW_RIGHT ((uint8_t)0)
#define DT7_RC_SW_Left  ((uint8_t)1)
	
#define DT7_RC_SW_UP    ((uint16_t)1)	
#define DT7_RC_SW_DOWN  ((uint16_t)2)
#define DT7_RC_SW_MID   ((uint16_t)3)


/** Mouse Config**/
#define DT7_MOUSE_X                 ((uint8_t)0)
#define DT7_MOUSE_Y                 ((uint8_t)1)
#define DT7_MOUSE_Z                 ((uint8_t)2)

#define DT7_MOUSE_LEFT              ((uint8_t)3)
#define DT7_MOUSE_RIGHT             ((uint8_t)4)

#define DT7_MOUSE_PRESSED_OFFSET    ((uint8_t)0)
#define DT7_MOUSE_SPEED_OFFSET      ((uint16_t)0)


/** Keyboard Config **/
#define DT7_KEY_W         ((uint8_t)0) 
#define DT7_KEY_S         ((uint8_t)1)
#define DT7_KEY_A         ((uint8_t)2)
#define DT7_KEY_D         ((uint8_t)3)
#define DT7_KEY_SHIFT     ((uint8_t)4)
#define DT7_KEY_CTRL      ((uint8_t)5)
#define DT7_KEY_Q         ((uint8_t)6)
#define DT7_KEY_E         ((uint8_t)7)
#define DT7_KEY_R         ((uint8_t)8)
#define DT7_KEY_F         ((uint8_t)9)
#define DT7_KEY_G         ((uint8_t)10)
#define DT7_KEY_Z         ((uint8_t)11)
#define DT7_KEY_X         ((uint8_t)12)
#define DT7_KEY_C         ((uint8_t)13)
#define DT7_KEY_V         ((uint8_t)14)
#define DT7_KEY_B         ((uint8_t)15)
#define DT7_KEY_OFFSET    ((uint8_t)0)

#define DT7_KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define DT7_KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define DT7_KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define DT7_KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define DT7_KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define DT7_KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
#define DT7_KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
#define DT7_KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)
#define DT7_KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
#define DT7_KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define DT7_KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
#define DT7_KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
#define DT7_KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define DT7_KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define DT7_KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define DT7_KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)


/**
 * \brief 遥控器数据结构体（未处理）
 */
typedef struct
{ 
  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint16_t ch4;
  uint8_t  switch_left;
  uint8_t  switch_right;

  struct
  {
    uint8_t press_left;
    uint8_t press_right;	  
    int16_t x;
    int16_t y;
    int16_t z;
  }Mouse;

  struct
  {
    uint16_t key_code;
  } KeyBoard;
} DT7RecData;

/**
 * \brief 遥控器解读数据结构体（已处理）
 */
typedef struct 
{
	unsigned switch_left  : 2;
	unsigned switch_right  : 2;
	unsigned reserved 	 : 4;
	__packed struct
	{
		unsigned level_key_W : 1;		//四向移动
		unsigned level_key_A : 1;
		unsigned level_key_S : 1;
		unsigned level_key_D : 1;
		unsigned level_key_SHIFT : 1;	//加速
		unsigned level_key_CTRL  : 1;	//陀螺
		unsigned level_key_Q : 1;		//摇车
		unsigned level_key_E : 1;		//飞坡
		
		unsigned level_key_R : 1;		//一键转向
		unsigned level_key_F : 1;		//开摩擦轮
		unsigned level_key_G : 1;		//NULL
		unsigned level_key_Z : 1;		//开镜
		unsigned level_key_X : 1;		//吊射模式
		unsigned level_key_C : 1;		//计算并瞄准
		unsigned level_key_V : 1;		//应急重启
		unsigned level_key_B : 1;		//前哨站模式
	}PCKeyBoard;
	struct
	{
		uint8_t press_left;
		uint8_t press_right;
		float x;
		float y;
		float z;
	}PCMouse;
	
	int16_t ch0, ch1, ch2, ch3, ch4;
}DT7CmdData;


/**
* \brief  DT7遥控器串口初始化
*/
void DT7RemoteRecEnable(UART_HandleTypeDef* huart, uint8_t* uart_rec_buffer);

/**
 * \brief 从串口缓冲中读取DT7遥控器原始数据
 */
void DT7RawDataUpdate(DT7RecData* rec_data, const uint8_t* rec_buffer);

/**
 * \brief 根据DT7遥控器原始数据得到指令
 */
void DT7DataProcess(DT7CmdData* cmd_data, const DT7RecData* rec_data);

/**
  * \brief  获取遥控器拨杆值(相对)
  * \param[in]  rec_data 遥控器接收原始数据结构体
  * \param[in]  channel_num  拨杆通道标志位 
  * \return 通道拨杆值  -660~660
  */
static int16_t DT7GetChannelVal(const DT7RecData* rec_data, uint8_t channel_num);

/**
  * \brief  获取遥控器拨码值
  * \param[in]  switch_num  拨码标志位
  * \return 拨码值(上-1 中-3 下-2)
  */
static uint8_t DT7GeSwitchVal(const DT7RecData* rec_data, uint8_t switch_num);

/**
  * \brief  获取鼠标速度值
  * \param[in] xyz  鼠标轴标识位
  * \return 鼠标速度(32767 ~ -32768)
  */
static int16_t DT7GetMouseSpeed(const DT7RecData* rec_data, uint8_t xyz);

	/**
  * \brief  获取鼠标按键
  * \param[in]  button 鼠标按键标识位
  * \return 相应键位是否按下 0-未按下 1-按下
  */
static uint8_t DT7GetMousePressed(const DT7RecData* rec_data, uint8_t button);

/**
  * \brief  获取键盘按键
  * \param[in]  key 键盘键位标识位
  * \return 相应键位是否按下 0-未按下 1-按下
  */
static uint8_t DT7GetKeyBoardVal(const DT7RecData* rec_data,uint8_t key);
#endif
