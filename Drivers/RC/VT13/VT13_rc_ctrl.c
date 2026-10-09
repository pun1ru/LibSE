#include "VT13_rc_ctrl.h"
#include "main.h"

// remote control data
//遥控器控制变量
VT13_RC_ctrl_t VT13_rc_ctrl;

/**
 * @brief          获取遥控器数据指针
 * @param[in]      none
 * @retval         遥控器数据指针
 */
const VT13_RC_ctrl_t *get_VT13_remote_control_point(void)
{
    return &VT13_rc_ctrl;
}

/**
 * @brief          遥控器协议解析
 * @param[in]      VT13_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */
extern bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
void VT13_to_rc(/*volatile const*/ uint8_t *VT13_buf, VT13_RC_ctrl_t *rc_ctrl)
{
    // 检查输入指针是否为空
    if (VT13_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // 检查VT13_buf的前两个字节是否为特定值，并验证CRC16校验和
	if(VT13_buf[0] == 0xa9 && VT13_buf[1] == 0x53 && verify_crc16_check_sum(VT13_buf, 21)) 
	{			
	// 从VT13_buf中解析通道0的值
		rc_ctrl->rc.ch[0] = (VT13_buf[2] | (VT13_buf[3] << 8)) & 0x07ff;        //!< Channel 0
	// 从VT13_buf中解析通道1的值
		rc_ctrl->rc.ch[1] = ((VT13_buf[3] >> 3) | (VT13_buf[4] << 5)) & 0x07ff; //!< Channel 1
	// 从VT13_buf中解析通道2的值
		rc_ctrl->rc.ch[2] = ((VT13_buf[4] >> 6) | (VT13_buf[5] << 2) |          //!< Channel 2
													(VT13_buf[6] << 10)) & 0x07ff;
	// 从VT13_buf中解析通道3的值
		rc_ctrl->rc.ch[3] = ((VT13_buf[6] >> 1) | (VT13_buf[7] << 7)) & 0x07ff; //!< Channel 3
	// 从VT13_buf中解析模式开关的值
		rc_ctrl->rc.mode_sw = ((VT13_buf[7] >> 4) & 0x0003); 
	// 从VT13_buf中解析停止按钮的值
		rc_ctrl->rc.stop = ((VT13_buf[7] >> 6) & 0x01);
	// 从VT13_buf中解析左按钮的值
		rc_ctrl->rc.left_button = ((VT13_buf[7] >> 7) & 0x01);//fn
	// 从VT13_buf中解析右按钮的值
		rc_ctrl->rc.right_button = ((VT13_buf[8] >> 0) & 0x01);
	// 从VT13_buf中解析滚轮的值
		rc_ctrl->rc.wheel = ((VT13_buf[8] >> 1) | (VT13_buf[9] << 7)) & 0x07FF;
	// 从VT13_buf中解析（扳机）的值
		rc_ctrl->rc.shutter = (VT13_buf[9] >> 4) & 0x01;//扳机
		
	// 从VT13_buf中解析鼠标X轴的值
		rc_ctrl->mouse.x = (VT13_buf[10] | (VT13_buf[11] << 8));
	// 从VT13_buf中解析鼠标Y轴的值
		rc_ctrl->mouse.y = (VT13_buf[12] | (VT13_buf[13] << 8));
	// 从VT13_buf中解析鼠标Z轴的值
		rc_ctrl->mouse.z = (VT13_buf[14] | (VT13_buf[15] << 8));
		
	// 从VT13_buf中解析鼠标左键的状态
		rc_ctrl->mouse.press_l = (VT13_buf[16] >> 0) & 0x03;
	// 从VT13_buf中解析鼠标右键的状态
		rc_ctrl->mouse.press_r = (VT13_buf[16] >> 2) & 0x03;
	// 从VT13_buf中解析鼠标中键的状态
		rc_ctrl->mouse.middle = (VT13_buf[16] >> 4) & 0x03;
		
	// 从VT13_buf中解析键值
		rc_ctrl->key.v = (VT13_buf[17] | (VT13_buf[18] << 8));
		
	// 从VT13_buf中解析CRC16校验和
		rc_ctrl->crc16 = (VT13_buf[19] | (VT13_buf[20] << 8));
		
	// 对解析出的通道值和滚轮值进行偏移调整
		rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.wheel -= RC_CH_VALUE_OFFSET;
	}
   
}

