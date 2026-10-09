#ifndef _UI_DRIVER_H_
#define _UI_DRIVER_H_

#include <stdint.h>
#include <string.h>
#include "judge_receive.h"
#include "usart.h"
#include "general_define.h"
#define LAYER_NUMBER   9
#define COLOR_NUMBER   8


/*
相关结构体定义
*/
//定义在judge_receive.h

/*
数据帧定义开始
*/
#pragma pack(1)
typedef union
{
    ext_robot_interaction_data_t  ext_robot_interaction_data;
	uint8_t  data[119];
}UIframe_t; //UI帧

typedef union
{
    ext_interaction_layer_delete_t  ext_interaction_layer_delete;
	uint8_t  data[2];
}LayerDeleteFrame_t;//层数删除帧

typedef union
{
    ext_interaction_figure_t  ext_interaction_figure;
	uint8_t  data[15];
}InteractionFigureFrame_t;//1个图层帧

typedef union
{
    ext_interaction_figure_2_t  ext_interaction_figure_2;
	uint8_t  data[30];
}InteractionFigure2Frame_t;//2个图层帧

typedef union
{
    ext_interaction_figure_3_t  ext_interaction_figure_3;
	uint8_t  data[75];
}InteractionFigure3Frame_t;//5个图层帧

typedef union
{
    ext_interaction_figure_4_t  ext_interaction_figure_4;
	uint8_t  data[105];
}InteractionFigure4Frame_t;//7个图层帧

typedef union
{
	
	ext_interaction_character_t character_frame;
	uint8_t data[45];
}InteractionCharacterFrame_t;//字符帧

typedef struct
{
		 
		 uint8_t SOF;
         uint16_t data_length;
         uint8_t seq;
         uint8_t CRC8;
}FrameHeader_t;//裁判系统帧头
	 
typedef struct
{
	
	 FrameHeader_t  FrameHeader;
	 uint16_t cmd_id;
	 uint8_t data[119];
	 uint16_t frame_tail;
 }RefereeFrame_t; //裁判系统帧 128位
#pragma pack()
/* UI操作类型 */
 /*删除操作*/
typedef enum	
{
  DeleteNull = 0,//空操作
  DeleteOne,//删除图层
  DeleteAll,//删除所有
} DeleteType;
 /*图形操作*/
typedef enum	
{
  OperateNull = 0,//空操作
  OperateAdd,//增加
  OperateChange,//修改
  OperateDelete,//删除
} OperateType;


/* 图形形状 */
typedef enum
{
  GraphicLine = 0,//直线
  GraphicRect,//长方形
  GraphicCircle,//圆形
  GraphicEllipse,//椭圆
  GraphicArc, //圆弧，我翻译搜的，你懂就行
  GraphicFloat,//浮点
  GraphicInt,//整型
  GraphicChar,//字符型
} GraphicType;

/* 图形颜色 */
typedef enum
{
  ColorTeam = 0,//己方颜色，红与蓝
  ColorYellow,
  ColorGreen,
  ColorOrange,
  ColorAmaranth, //紫红色
  ColorPink,
  ColorCyan,//青色
  ColorBlack,
  Colorwhite,
} ColorType;
 /*发送反馈*/
typedef enum
{
  Success=1,//发送成功
  CoordinateError,//坐标错误
  LayerError,//图层错误
  ColorError,//颜色错误
  NameSizeError,//name大小错误
} ErrorType;

typedef enum
{
	LayerEmpty=6,
	LayerOk,
	LayerMiss,
} LayerType;


void UIframeInit(UIframe_t *const UIframe,uint16_t SubcontentID,uint16_t SenderID,uint16_t ReceiverID);

void LayerDelete(ext_interaction_layer_delete_t *const ext_interaction_layer_delete, DeleteType deletetype,uint8_t layer);
uint8_t DrawLine(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint8_t name[], uint32_t width, uint8_t layer,ColorType color);

uint8_t DrawRect(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint8_t name[], uint32_t width, uint8_t layer,ColorType color);

uint8_t DrawCircle(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t radius, uint8_t name[], uint32_t width, uint8_t layer,ColorType color);

uint8_t DrawEllipse(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t halfshaft_x,uint32_t halfshaft_y, uint8_t name[], uint32_t width, uint8_t layer,ColorType color);

uint8_t DrawArc(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t halfshaft_x, uint32_t halfshaft_y, uint32_t start_angle, \
    uint32_t end_angle, uint8_t name[], uint32_t width, uint8_t layer,ColorType color);

uint8_t DrawFloat(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint8_t name[], uint32_t width, uint32_t size,\
                    uint8_t layer, uint32_t contents,ColorType color);

uint8_t DrawInt(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint8_t name[], uint32_t width, uint32_t size,\
                    uint8_t layer, uint32_t contents,ColorType color);


uint8_t DrawChar(ext_interaction_character_t *const character_frame,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint8_t name[], uint32_t width, uint32_t size,\
                    uint8_t layer, uint8_t* contents, uint8_t len, ColorType color);

void FigureToUIframe(uint8_t *data_figure,uint16_t SubcontentID,uint8_t *data_UIframe);
void CharacterToUIframe(ext_interaction_character_t *data_figure,uint16_t SubcontentID,uint8_t *data_UIframe);
void UIframeTransmit(uint8_t *data_UIframe,uint16_t SubcontentID);
void UIframeClear(uint8_t *data_UIframe);

uint8_t CheckEmpty(ext_interaction_figure_t *const ext_interaction_figure,uint16_t SubcontentID);
uint8_t FigureJudge(ext_interaction_figure_t *const ext_interaction_figure,uint16_t SubcontentID);
#endif