#include "CAN_driver.h"
#include "task.h"

#include <string.h>

#define CAN_TX_QUEUE_LENGTH 16U

static QueueHandle_t can1_tx_queue;
static QueueHandle_t can2_tx_queue;
static QueueHandle_t can3_tx_queue;
CANTxMonitor canMonitor[3] = {0};

static QueueHandle_t CANTransmit_QueueFor(FDCAN_HandleTypeDef *hcan)
{
	if (hcan == &hfdcan1) return can1_tx_queue;
	if (hcan == &hfdcan2) return can2_tx_queue;
	if (hcan == &hfdcan3) return can3_tx_queue;
	return NULL;
}

static uint8_t _fdcan_idx(FDCAN_HandleTypeDef *h)
{
	if (h == &hfdcan1) return 0;
	if (h == &hfdcan2) return 1;
	if (h == &hfdcan3) return 2;
	return NULL;
}

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

void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;       
	fdcan_filter.FilterID1 = 0x000;                             
	fdcan_filter.FilterID2 = 0x000;                               
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; 
	fdcan_filter.FilterID1 = 0x000;                              
	fdcan_filter.FilterID2 = 0x000;            
	HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter);
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; 
	fdcan_filter.FilterID1 = 0x000; // ID值不重要
	fdcan_filter.FilterID2 = 0x000; // 掩码为0，所有11位都不关心，即接收所有标准ID        
	HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter); 
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	//HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	//发送帧设置
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;															
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;													
    TxHeader.DataLength = len << 16;																	
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;															
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;													 
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;														
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;									
    TxHeader.MessageMarker = 0x00; 	             
	
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
	if (can1_tx_queue == NULL) {
		can1_tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxFrame_t));
	}
	if (can2_tx_queue == NULL) {
		can2_tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxFrame_t));
	}
	if (can3_tx_queue == NULL) {
		can3_tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxFrame_t));
	}
	return can1_tx_queue != NULL && can2_tx_queue != NULL && can3_tx_queue != NULL
			? pdPASS : pdFAIL;
}

BaseType_t CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
							   const uint8_t data[8])
{
	CanTxFrame_t frame;
	QueueHandle_t queue = CANTransmit_QueueFor(hcan);

	if (queue == NULL || data == NULL || std_id > 0x7FFU) {
		return pdFAIL;
	}
	frame.can = hcan;
	frame.id = std_id;
	memcpy(frame.data, data, sizeof(frame.data));
	return xQueueSend(queue, &frame, 0U);
}

void CANTransmit_QueueTask(void *argument)
{
	CanTxFrame_t frame;
	QueueHandle_t queue = CANTransmit_QueueFor((FDCAN_HandleTypeDef *)argument);

	if (queue == NULL) {
		vTaskDelete(NULL);
		return;
	}

	for (;;) {
		if (xQueueReceive(queue, &frame, portMAX_DELAY) == pdPASS) {
			(void)fdcanx_send_data(frame.can, frame.id, frame.data, sizeof(frame.data));
		}
	}
}

void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[])
{
    fdcanx_send_data(hcan, std_id, aData, 8);
}

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


