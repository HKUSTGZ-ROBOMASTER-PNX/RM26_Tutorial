#include "maix.hpp"
#include "bsp_usart.hpp"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

uint8_t UART1RxBuffer[MAIX_BUFFER_SIZE];

/**
 * @brief  Configures the USART.
 * @param  None
 * @retval None
 */

void USART_Init()
{
  // usart1
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);
  HAL_UART_Receive_DMA(&huart1, UART1RxBuffer, MAIX_BUFFER_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    HAL_UART_Receive_DMA(&huart1, UART1RxBuffer, MAIX_BUFFER_SIZE);
    HAL_UART_Transmit_DMA(&huart1, UART1RxBuffer, MAIX_BUFFER_SIZE);
    memcpy(Maix::Instance()->maix_rx.buffer, UART1RxBuffer, MAIX_BUFFER_SIZE);
    memset(UART1RxBuffer, 0, sizeof(UART1RxBuffer));
  }
}
