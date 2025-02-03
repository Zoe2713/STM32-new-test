#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //开启空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
   //DMA失效
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    //当传输结束时，禁用DMA

    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
   //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

   //DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}