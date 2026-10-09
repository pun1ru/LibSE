#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "struct_typedef.h"
#include "stm32h7xx.h"
extern void INS_SPI_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void INS_SPI_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif
