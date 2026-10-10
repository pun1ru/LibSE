#include "task_init.h"


void CANInit(){
  can_instances[0].can = &hfdcan1;
  can_instances[0].rx_fifo = FDCAN_RX_FIFO0;
  can_instances[1].can = &hfdcan2;
  can_instances[1].rx_fifo = FDCAN_RX_FIFO1;
  can_instances[2].can = &hfdcan3;
  can_instances[2].rx_fifo = FDCAN_RX_FIFO1;
  CAN_RegisterInstances(can_instances, 3U);
  if (CANTransmit_QueueInit() != pdPASS ||
      CANReceive_QueueInit() != pdPASS) {
    Error_Handler();
  }
}


void CommunicateInit(void)
{
  CANInit();
}