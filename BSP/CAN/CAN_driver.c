#include "CAN_driver.h"

#include <string.h>

#define CAN_TX_QUEUE_LENGTH 16U

static QueueHandle_t can_tx_queue;

// if can bus is too busy to send frame,  hcan->ErrorCode |= HAL_CAN_ERROR_PARAM will be performed , pay attention 

/* CAN发送监控（在线watch: canMonitor[0/1/2]） */
CANTxMonitor canMonitor[3] = {0};

static uint8_t _fdcan_idx(FDCAN_HandleTypeDef *h)
{
	if (h == &hfdcan1) return 0;
	if (h == &hfdcan2) return 1;
	return 2; // hfdcan3
}

/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	//使能中断
	uint32_t target_interrupts = 0x00038001; // 将所有期望的位组合成一个确定的数值
	HAL_FDCAN_ActivateNotification(&hfdcan1,target_interrupts,NULL);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_TX_EVT_FIFO_NEW_DATA, NULL);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, NULL);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_RANGE;                  
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //???0???FIFO0  
	fdcan_filter.FilterID1 = 0x08;                               //32?ID
	fdcan_filter.FilterID2 = 0x300;                               
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; 
	fdcan_filter.FilterID1 = 0x141;                               //32位ID
	fdcan_filter.FilterID2 = 0x300;            
	HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter);
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; 
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;//
	fdcan_filter.FilterID1 = 0x000; // ID值不重要
	fdcan_filter.FilterID2 = 0x000; // 掩码为0，所有11位都不关心，即接收所有标准ID        
	HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter); 
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);//reject all extid
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	//HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	//发送帧设置
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;																// ??ID 
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// ??? 
    TxHeader.DataLength = len << 16;																		// ?????? 
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										// ???????? 								
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// ???????? 
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// ??CAN?? 
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// ??????FIFO??, ??? 
    TxHeader.MessageMarker = 0x00; 			// ?????TX EVENT FIFO???Maker??????????0?0xFF                
	
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) {
		canMonitor[_fdcan_idx(hfdcan)].tx_fail++;
		canMonitor[_fdcan_idx(hfdcan)].last_fail_id = id;
		return 1;//发送失败
	}
	canMonitor[_fdcan_idx(hfdcan)].tx_ok++;
	return 0;	
}

BaseType_t CANTransmit_QueueInit(void)
{
	if (can_tx_queue == NULL) {
		can_tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxFrame_t));
	}
	return can_tx_queue != NULL ? pdPASS : pdFAIL;
}

BaseType_t CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
							   const uint8_t data[8])
{
	CanTxFrame_t frame;

	if (can_tx_queue == NULL || hcan == NULL || data == NULL || std_id > 0x7FFU) {
		return pdFAIL;
	}
	frame.can = hcan;
	frame.id = std_id;
	memcpy(frame.data, data, sizeof(frame.data));
	return xQueueSend(can_tx_queue, &frame, 0U);
}

void CANTransmit_QueueTask(void *argument)
{
	CanTxFrame_t frame;
	(void)argument;

	for (;;) {
		if (xQueueReceive(can_tx_queue, &frame, portMAX_DELAY) == pdPASS) {
			(void)fdcanx_send_data(frame.can, frame.id, frame.data, sizeof(frame.data));
		}
	}
}

/**
 * \brief CAN邮箱发送，发送数据以字节为单位
 * \param[in] hcan CAN总线控制句柄
 * \param[in] std_id 目标节点标识符 
 * \param[in] aData 数据缓冲数组
 */ 
void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[])
{
    fdcanx_send_data(hcan, std_id, aData, 8);
}

/**
 * \brief CAN邮箱发送，发送数据以uint16_t为单位
 * \param[in] hcan CAN总线控制句柄
 * \param[in] std_id 目标节点标识符 
 * \param[in] uint16_t data1 data2 data3 data4
 */ 
void CANTransmit_I16(FDCAN_HandleTypeDef *hcan, uint32_t std_id, int16_t output1, int16_t output2, int16_t output3, int16_t output4)
{            
	uint8_t aData[8] = {0};
	aData[0] = (uint8_t)(output1 >> 8);		
	aData[1] = (uint8_t)output1;
	aData[2] = (uint8_t)(output2 >> 8);
	aData[3] = (uint8_t)output2;
	aData[4] = (uint8_t)(output3 >> 8);
	aData[5] = (uint8_t)output3;
	aData[6] = (uint8_t)(output4 >> 8);
	aData[7] = (uint8_t)output4;		
	fdcanx_send_data(hcan, std_id, aData, 8);
}


