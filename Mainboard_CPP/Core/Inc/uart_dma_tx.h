#ifndef UART_DMA_TX_H
#define UART_DMA_TX_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void uart_dma_tx_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_tx);
void uart_dma_tx_send(const uint8_t *data, uint16_t len);
void uart_dma_tx_irq_handler(DMA_HandleTypeDef *hdma);

#ifdef __cplusplus
}
#endif

#endif
