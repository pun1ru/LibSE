#include "bsp_spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
static DMA_Stream_TypeDef *rx_instance, *tx_instance;
void INS_SPI_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    rx_instance = (DMA_Stream_TypeDef *)hdma_spi2_rx.Instance;
    tx_instance = (DMA_Stream_TypeDef *)hdma_spi2_tx.Instance;
    SPI_2LINES(&hspi2);
    SET_BIT(hspi2.Instance->CFG1, SPI_CFG1_TXDMAEN);
    SET_BIT(hspi2.Instance->CFG1, SPI_CFG1_RXDMAEN);
    //endless mode
    MODIFY_REG(hspi2.Instance->CR2, SPI_CR2_TSIZE, 0);
    __HAL_SPI_ENABLE(&hspi2);
    SET_BIT(hspi2.Instance->CR1, SPI_CR1_CSTART);
    // SET_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN);
    // SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi2_rx);
    
    while(rx_instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_rx, DMA_HISR_TCIF4);

    rx_instance->PAR = (uint32_t) & (SPI2->RXDR);
    //memory buffer 1
    //内存缓冲区1
    rx_instance->M0AR = (uint32_t)(rx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi2_rx, num);

    __HAL_DMA_ENABLE_IT(&hdma_spi2_rx, DMA_IT_TC);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi2_tx);
    
    while(tx_instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_tx);
    }


    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_tx, DMA_HISR_TCIF5);

    tx_instance->PAR = (uint32_t) & (SPI2->TXDR);
    //memory buffer 1
    //内存缓冲区1
    tx_instance->M0AR = (uint32_t)(tx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi2_tx, num);


}

void INS_SPI_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi2_rx);
    __HAL_DMA_DISABLE(&hdma_spi2_tx);
    
    while(rx_instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_rx);
    }
    while(tx_instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_tx);
    }
    //clear flag
    //清除标志位
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi2.hdmarx));

    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi2.hdmatx));
    //set memory address
    //设置数据地址
    rx_instance->M0AR = rx_buf;
    tx_instance->M0AR = tx_buf;
    //set data length
    //设置数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi2_rx, ndtr);
    __HAL_DMA_SET_COUNTER(&hdma_spi2_tx, ndtr);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_spi2_rx);
    __HAL_DMA_ENABLE(&hdma_spi2_tx);
    
}




