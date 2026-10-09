#ifndef VT13_RC_CTRL_H
#define VT13_RC_CTRL_H
#include <stdint.h>   // 定义了 uint8_t, uint16_t, int16_t 等精确宽度整数类型
#include <stdbool.h>  // 定义了 bool, true, false
#include <stddef.h>   // 定义了 NULL  
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define VT13_HEADER_SIZE        sizeof(VT13_RC_ctrl_t)
	

/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED_VT13         (  VT13_rc_ctrl.key.v  )
#define    IF_KEY_PRESSED_W_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)     != 0 )
#define    IF_KEY_PRESSED_S_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)     != 0 )
#define    IF_KEY_PRESSED_A_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)     != 0 )
#define    IF_KEY_PRESSED_D_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)     != 0 )
#define    IF_KEY_PRESSED_Q_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)     != 0 )
#define    IF_KEY_PRESSED_E_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)     != 0 )
#define    IF_KEY_PRESSED_G_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)     != 0 )
#define    IF_KEY_PRESSED_X_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)     != 0 )
#define    IF_KEY_PRESSED_Z_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)     != 0 )
#define    IF_KEY_PRESSED_C_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)     != 0 )
#define    IF_KEY_PRESSED_B_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)     != 0 )
#define    IF_KEY_PRESSED_V_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)     != 0 )
#define    IF_KEY_PRESSED_F_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)     != 0 )
#define    IF_KEY_PRESSED_R_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)     != 0 )
#define    IF_KEY_PRESSED_CTRL_VT13    ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)  != 0 )
#define    IF_KEY_PRESSED_SHIFT_VT13   ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT_VT13    (VT13_rc_ctrl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH_VT13    (VT13_rc_ctrl.mouse.press_r == 1)
#define    IF_MOUSE_PRESSED_MID_VT13     (VT13_rc_ctrl.mouse.middle == 1)


/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED_VT13    (VT13_rc_ctrl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED_VT13    (VT13_rc_ctrl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED_VT13    (VT13_rc_ctrl.mouse.z)

typedef struct __attribute__((packed))
{
        struct
        {
                int16_t ch[4];
                uint8_t mode_sw;
                uint8_t stop;
                uint8_t left_button;//fn;
                uint8_t	right_button;
                int16_t wheel;
                uint8_t shutter;
        } rc;
        struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
                uint8_t middle;
        } mouse;
        struct
        {
                uint16_t v;
        } key;
        uint16_t crc16;

} VT13_RC_ctrl_t;


extern VT13_RC_ctrl_t VT13_rc_ctrl;
extern const VT13_RC_ctrl_t *get_VT13_remote_control_point(void);


void VT13_to_rc(uint8_t *VT13_buf, VT13_RC_ctrl_t *rc_ctrl);




#endif

