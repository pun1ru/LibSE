#include "UI_driver.h"
#include "judge_receive.h"

/**
 * @brief  UIframe 数据初始化
 * @note   
 * @param  UIframe指针 SubcontentID子内容ID SenderID发送者ID ReceiverID接收者ID
 * @retval None
 */
void UIframeInit(UIframe_t *const UIframe,uint16_t SubcontentID,uint16_t SenderID,uint16_t ReceiverID)
{
	UIframe->ext_robot_interaction_data.sender_id=SenderID;
	UIframe->ext_robot_interaction_data.receiver_id=ReceiverID;
    switch(SubcontentID)
	{
		case 0X0100:
			UIframe->ext_robot_interaction_data.data_cmd_id=0X0100;
		break;
		case 0X0101:
			UIframe->ext_robot_interaction_data.data_cmd_id=0X0101;
		break;
		case 0X0102:
			UIframe->ext_robot_interaction_data.data_cmd_id=0X0102;
		break;
		case 0X0103:
			UIframe->ext_robot_interaction_data.data_cmd_id=0X0103;
		break;
		case 0X0104:
			UIframe->ext_robot_interaction_data.data_cmd_id=0X0104;
		break;
		case 0X0110:
			UIframe->ext_robot_interaction_data.data_cmd_id=0X0110;
		break;
		default:
			break;
	}
}
/**
 * @brief  删除图层
 * @note   
 * @param  *LayerDeleteFrame_t，枚举变量OperateType uint8_t 层数
 * @retval None
 */
 void LayerDelete(ext_interaction_layer_delete_t *const ext_interaction_layer_delete, DeleteType deletetype,uint8_t layer)
{
  ext_interaction_layer_delete->layer=layer;
  ext_interaction_layer_delete->delete_type=deletetype;
}

/**
 * @brief  画直线
 * @note   调用该函数画直线
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 直线起始x坐标
 * @param  start_y: 直线起始y坐标
 * @param  end_x: 直线结束x坐标
 * @param  end_y: 直线结束y坐标
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  layer: 图层
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */
uint8_t DrawLine(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint8_t name[], uint32_t width, uint8_t layer,ColorType color)
{
	if(start_x>1920||end_x>1920||start_y>1080||start_y>1080)
		return CoordinateError;
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	if (0)
    {
		return NameSizeError;
    }
	ext_interaction_figure->figure_name[0]=name[0];
	ext_interaction_figure->figure_name[1]=name[1];
	ext_interaction_figure->figure_name[2]=name[2];
	ext_interaction_figure->operate_tpye=operate;
	ext_interaction_figure->layer=layer;
	ext_interaction_figure->figure_tpye=GraphicLine;
	ext_interaction_figure->color=color;
	ext_interaction_figure->details_a=0;
	ext_interaction_figure->details_b=0;
	ext_interaction_figure->width=width;
	ext_interaction_figure->start_x=start_x;
	ext_interaction_figure->start_y=start_y;
	ext_interaction_figure->details_c=0;
	ext_interaction_figure->details_d=end_x;
	ext_interaction_figure->details_e=end_y;
	return Success;
}
/**
 * @brief  画矩形
 * @note   调用该函数画矩形
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 直线起始x坐标
 * @param  start_y: 直线起始y坐标
 * @param  end_x: 对角顶点 x 坐标
 * @param  end_y: 对角顶点 y 坐标
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  layer: 图层
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */

uint8_t DrawRect(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint8_t name[], uint32_t width, uint8_t layer,ColorType color)
{
	if(start_x>1920||end_x>1920||start_y>1080||start_y>1080)
		return CoordinateError;
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	if (0)
    {
		return NameSizeError;
    }
	ext_interaction_figure->figure_name[0]=name[0];
	ext_interaction_figure->figure_name[1]=name[1];
	ext_interaction_figure->figure_name[2]=name[2];
	ext_interaction_figure->operate_tpye=operate;
	ext_interaction_figure->layer=layer;
	ext_interaction_figure->figure_tpye=GraphicRect;
	ext_interaction_figure->color=color;
	ext_interaction_figure->details_a=0;
	ext_interaction_figure->details_b=0;
	ext_interaction_figure->width=width;
	ext_interaction_figure->start_x=start_x;
	ext_interaction_figure->start_y=start_y;
	ext_interaction_figure->details_c=0;
	ext_interaction_figure->details_d=end_x;
	ext_interaction_figure->details_e=end_y;
	return Success;
}
/**
 * @brief  画圆
 * @note   调用该函数画圆
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 直线起始x坐标
 * @param  start_y: 直线起始y坐标
 * @param  radius: 半径
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  layer: 图层
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */

uint8_t DrawCircle(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t radius, uint8_t name[], uint32_t width, uint8_t layer,ColorType color)
{
	if((start_x+radius)>1920||(int)(start_x-radius)<0\
		||(start_y+radius)>1080||(int)(start_y-radius)<0)
		return CoordinateError;
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	if (0)
    {
		return NameSizeError;
    }
	ext_interaction_figure->figure_name[0]=name[0];
	ext_interaction_figure->figure_name[1]=name[1];
	ext_interaction_figure->figure_name[2]=name[2];
	ext_interaction_figure->operate_tpye=operate;
	ext_interaction_figure->layer=layer;
	ext_interaction_figure->figure_tpye=GraphicCircle;
	ext_interaction_figure->color=color;
	ext_interaction_figure->details_a=0;
	ext_interaction_figure->details_b=0;
	ext_interaction_figure->width=width;
	ext_interaction_figure->start_x=start_x;
	ext_interaction_figure->start_y=start_y;
	ext_interaction_figure->details_c=radius;
	ext_interaction_figure->details_d=0;
	ext_interaction_figure->details_e=0;
	return Success;
}

/**
 * @brief  画椭圆
 * @note   调用该函数画椭圆
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 直线起始x坐标
 * @param  start_y: 直线起始y坐标
 * @param  halfshaft_x: x 半轴长度
 * @param  halfshaft_y: y 半轴长度
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  layer: 图层
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */

uint8_t DrawEllipse(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t halfshaft_x,uint32_t halfshaft_y, uint8_t name[], uint32_t width, uint8_t layer,ColorType color)
{
	if((start_x+halfshaft_x)>1920||(int)(start_x-halfshaft_x)<0\
		||(start_y+halfshaft_y)>1080||(int)(start_y-halfshaft_y)<0)
		return CoordinateError;
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	if (0)
    {
		return NameSizeError;
    }
	ext_interaction_figure->figure_name[0]=name[0];
	ext_interaction_figure->figure_name[1]=name[1];
	ext_interaction_figure->figure_name[2]=name[2];
	ext_interaction_figure->operate_tpye=operate;
	ext_interaction_figure->layer=layer;
	ext_interaction_figure->figure_tpye=GraphicEllipse;
	ext_interaction_figure->color=color;
	ext_interaction_figure->details_a=0;
	ext_interaction_figure->details_b=0;
	ext_interaction_figure->width=width;
	ext_interaction_figure->start_x=start_x;
	ext_interaction_figure->start_y=start_y;
	ext_interaction_figure->details_c=0;
	ext_interaction_figure->details_d=halfshaft_x;
	ext_interaction_figure->details_e=halfshaft_y;
	return Success;
 
}
/**
 * @brief  画圆弧
 * @note   调用该函数画圆弧
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 直线起始x坐标
 * @param  start_y: 直线起始y坐标
 * @param  halfshaft_x: x 半轴长度
 * @param  halfshaft_y: y 半轴长度
 * @param  start_angle: 起始角度
 * @param  end_angle: 终止角度
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  layer: 图层
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */

uint8_t DrawArc(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint32_t halfshaft_x, uint32_t halfshaft_y, uint32_t start_angle, \
    uint32_t end_angle, uint8_t name[], uint32_t width, uint8_t layer,ColorType color)
{
	if(start_x<halfshaft_x||start_x>1920-halfshaft_x\
		||start_y<halfshaft_y||start_y>1080-halfshaft_y)
	    return  CoordinateError;//判断的不是很精确，写解析解太麻烦了
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	if (0)
    {
		return NameSizeError;
    }
	ext_interaction_figure->figure_name[0]=name[0];
	ext_interaction_figure->figure_name[1]=name[1];
	ext_interaction_figure->figure_name[2]=name[2];
	ext_interaction_figure->operate_tpye=operate;
	ext_interaction_figure->layer=layer;
	ext_interaction_figure->figure_tpye=GraphicArc;
	ext_interaction_figure->color=color;
	ext_interaction_figure->details_a=start_angle;
	ext_interaction_figure->details_b=end_angle;
	ext_interaction_figure->width=width;
	ext_interaction_figure->start_x=start_x;
	ext_interaction_figure->start_y=start_y;
	ext_interaction_figure->details_c=0;
	ext_interaction_figure->details_d=halfshaft_x;
	ext_interaction_figure->details_e=halfshaft_y;
	return Success;
}
/**
 * @brief  画浮点
 * @note   调用该函数画浮点
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 起始x坐标
 * @param  start_y: 起始y坐标
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  size：字体大小
 * @param  layer: 图层
 * @param  contents：浮点数*1000
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */

uint8_t DrawFloat(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint8_t name[], uint32_t width, uint32_t size,\
    uint8_t layer, uint32_t contents,ColorType color)
{
	
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	ext_interaction_figure->figure_name[0]=name[0];
	ext_interaction_figure->figure_name[1]=name[1];
	ext_interaction_figure->figure_name[2]=name[2];
	ext_interaction_figure->operate_tpye=operate;
	ext_interaction_figure->layer=layer;
	ext_interaction_figure->figure_tpye=GraphicFloat;
	ext_interaction_figure->color=color;
	ext_interaction_figure->details_a=size;
	ext_interaction_figure->details_b=0;
	ext_interaction_figure->width=width;
	ext_interaction_figure->start_x=start_x;
	ext_interaction_figure->start_y=start_y;
	ext_interaction_figure->details_c=contents;
	ext_interaction_figure->details_d=(contents>>10);
	ext_interaction_figure->details_e=(contents>>21);
	return Success;
}
/**
 * @brief  画整型
 * @note   调用该函数画整型
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 起始x坐标
 * @param  start_y: 起始y坐标
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  size：字体大小
 * @param  layer: 图层
 * @param  contents：整型
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */

uint8_t DrawInt(ext_interaction_figure_t *const ext_interaction_figure,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint8_t name[], uint32_t width, uint32_t size,\
                    uint8_t layer, uint32_t contents,ColorType color)
{
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	if (0)
    {
		return NameSizeError;
    }
	ext_interaction_figure->figure_name[0]=name[0];
	ext_interaction_figure->figure_name[1]=name[1];
	ext_interaction_figure->figure_name[2]=name[2];
	ext_interaction_figure->operate_tpye=operate;
	ext_interaction_figure->layer=layer;
	ext_interaction_figure->figure_tpye=GraphicInt;
	ext_interaction_figure->color=color;
	ext_interaction_figure->details_a=size;
	ext_interaction_figure->details_b=0;
	ext_interaction_figure->width=width;
	ext_interaction_figure->start_x=start_x;
	ext_interaction_figure->start_y=start_y;
	ext_interaction_figure->details_c=contents;
	ext_interaction_figure->details_d=(contents>>10);
	ext_interaction_figure->details_e=(contents>>21);
	return Success;
}


/**
 * @brief  画字符型
 * @note   调用该函数画字符型
 * @param  ext_interaction_figure_t *const ext_interaction_figure
           传入一帧的结构体指针
 * @param  operate: 需要进行的操作类型，0-空操作，1-增加，2-修改，3-删除
 * @param  start_x: 起始x坐标
 * @param  start_y: 起始y坐标
 * @param  name:  图形名字(必须3个uint8_t，不够记得补零)
 * @param  width: 线宽
 * @param  size：字体大小
 * @param  layer: 图层
 * @param  contents:字符串
 * @param  len：字符长度
 * @param  color: 颜色
 * @retval 1:配置正确  2:坐标错误 3:图层错误 4:颜色错误 5:name大小不够
 */

uint8_t DrawChar(ext_interaction_character_t *const character_frame,OperateType operate,\
	uint32_t start_x, uint32_t start_y, uint8_t name[], uint32_t width, uint32_t size,\
                    uint8_t layer, uint8_t* contents, uint8_t len, ColorType color)
{
	if(layer>LAYER_NUMBER)
		return LayerError;
	if(color>COLOR_NUMBER)
		return ColorError;
	if (0)
    {
		return NameSizeError;
    }
	character_frame->character_figure.figure_name[0]=name[0];
	character_frame->character_figure.figure_name[1]=name[1];
	character_frame->character_figure.figure_name[2]=name[2];
	character_frame->character_figure.operate_tpye=operate;
	character_frame->character_figure.layer=layer;
	character_frame->character_figure.figure_tpye=GraphicChar;
	character_frame->character_figure.color=color;
	character_frame->character_figure.details_a=size;
	character_frame->character_figure.details_b=len;
	character_frame->character_figure.width=width;
	character_frame->character_figure.start_x=start_x;
	character_frame->character_figure.start_y=start_y;
	character_frame->character_figure.details_c=0;
	character_frame->character_figure.details_d=0;
	character_frame->character_figure.details_e=0;
	
	//memcpy(character_frame->char_data, contents, len);
		memset(character_frame->char_data,0,30);
	for(int i=0;i<len;i++)
	{
		character_frame->char_data[i] = contents[i];
	}
	character_frame->char_data[len]=0;

		
	
	return Success;
}
/**
 * @brief  非字符帧帧拷贝UI帧
 * @param  非字符帧的数组
 * @param  SubcontentID: 子命令ID
 * @param  UI帧的数组
 * @retval：none
 */
void FigureToUIframe(uint8_t *data_figure,uint16_t SubcontentID,uint8_t *data_UIframe)
{
	switch(SubcontentID)
	{
		case 0X0100:
			memcpy(data_UIframe+6,data_figure,2);
		    break;
		case 0X0101:
			memcpy(data_UIframe+6,data_figure,15);
		    break;
		case 0X0102:
			memcpy(data_UIframe+6,data_figure,30);
		    break;
		case 0X0103:
			memcpy(data_UIframe+6,data_figure,75);
		    break;
		case 0X0104:
			memcpy(data_UIframe+6,data_figure,105);
		    break;
		default:
			break;
	}
}

/**
 * @brief  数据帧拷贝UI帧
 * @param  数据帧的数组
 * @param  SubcontentID: 子命令ID
 * @param  UI帧的数组
 * @retval：none
 */
void CharacterToUIframe(ext_interaction_character_t *data_figure,uint16_t SubcontentID,uint8_t *data_UIframe)
{
	if(SubcontentID == 0x0110)
		memcpy(data_UIframe+6,data_figure,45);
}
/**
 * @brief  UI帧的发送
 * @param  UI的数组
 * @param  SubcontentID: 子命令ID
 * @param  UI帧的数组
 * @retval none
 */

void UIframeTransmit(uint8_t *data_UIframe,uint16_t SubcontentID)
{
    RefereeFrame_t RefereeFrame;
	RefereeFrame.FrameHeader.SOF=0XA5;
	RefereeFrame.FrameHeader.seq=0;
	RefereeFrame.cmd_id=0X0301;
	switch(SubcontentID)
	{
		case 0X0100:
			RefereeFrame.FrameHeader.data_length=8;
		    memcpy(RefereeFrame.data,data_UIframe,8);
		break;
		case 0X0101:
			RefereeFrame.FrameHeader.data_length=21;
		    memcpy(RefereeFrame.data,data_UIframe,21);
		break;
		case 0X0102:
			RefereeFrame.FrameHeader.data_length=36;
		    memcpy(RefereeFrame.data,data_UIframe,36);
		break;
		case 0X0103:
			RefereeFrame.FrameHeader.data_length=81;
		    memcpy(RefereeFrame.data,data_UIframe,81);
		break;
		case 0X0104:
			RefereeFrame.FrameHeader.data_length=111;
		    memcpy(RefereeFrame.data,data_UIframe,111);
		break;
		case 0X0110:
			RefereeFrame.FrameHeader.data_length=51;
		    memcpy(RefereeFrame.data,data_UIframe,51);
		break;
	}
	
	RefereeFrame.FrameHeader.seq++;
	uint8_t transmitdata[128];
	memcpy(transmitdata,&RefereeFrame,128);
    Append_CRC_Check_Sum(transmitdata, transmitdata[1]);
	HAL_UART_Transmit_DMA(&REFEREE_UART,transmitdata,(transmitdata[1]+9));
}
/**
 * @brief  数据帧判断有无图形操作协议之外的数字
 * @param  数据帧的数组
 * @param  SubcontentID: 子命令ID
 * @retval：LayerError-6；LayerOk-7；
 */
uint8_t FigureJudge(ext_interaction_figure_t *const ext_interaction_figure,uint16_t SubcontentID)
{
	switch(SubcontentID)
	{
		case 0X0101:
			for(int i=0;i<1;i++)
		{
			if(ext_interaction_figure[i].operate_tpye != OperateNull)
			   return LayerOk;
		}
		    return LayerEmpty;
		    break;
		case 0X0102:
			for(int i=0;i<2;i++)
		{
			if(ext_interaction_figure[i].operate_tpye != OperateNull)
			   return LayerOk;
		}
		     return LayerEmpty;
		     break;
		case 0X0103:
			for(int i=0;i<5;i++)
		{
			if(ext_interaction_figure[i].operate_tpye != OperateNull)
			   return LayerOk;
		}
		    return LayerEmpty;
	        break;
		case 0X0104:
			for(int i=0;i<7;i++)
		{
			if(ext_interaction_figure[i].operate_tpye != OperateNull)
			   return LayerOk;
		}
		    return LayerEmpty;
		    break;

	}
	return LayerMiss;
}

/**
 * @brief  清空UI帧(发送完清除一次)
 * @param  UI帧的数组
 * @retval：none
 */
void UIframeClear(uint8_t *data_UIframe)
{
	memset(data_UIframe, 0, 119);
}