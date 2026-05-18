#include "uart_dma_tx.h"
#include <string.h>
#include "core_cm7.h"

#define TX_BUF_SIZE   1024u
#define TX_BUF_MASK   (TX_BUF_SIZE - 1u)

static struct {
    uint8_t            buf[TX_BUF_SIZE]; // Moved to top for alignment
    volatile uint16_t  head;
    volatile uint16_t  tail;
    volatile uint16_t  bytes_in_dma;
    UART_HandleTypeDef *huart;
} s_tx __attribute__((section(".dma_uart_tx"), aligned(32)));

/* ── Try to launch a new DMA transfer ──────────────────────────────── */
static void start_dma_transfer(void)
{
    if (s_tx.bytes_in_dma > 0) return;

    uint16_t space = (s_tx.head - s_tx.tail) & TX_BUF_MASK;
    if (space == 0) return;

    uint16_t end  = TX_BUF_SIZE - s_tx.tail;
    uint16_t bytes = (space < end) ? space : end;

    /* * H7 CACHE FIX:
     * We need to clean the cache for the range we are about to send.
     * To be safe with the 32-byte cache line, we clean a bit more if needed,
     * or ensure our buffer management doesn't trip over line boundaries.
     */
    uint32_t addr = (uint32_t)&s_tx.buf[s_tx.tail];

    // Clean the specific memory area
    SCB_CleanDCache_by_Addr((uint32_t *)addr, bytes);

    s_tx.bytes_in_dma = bytes;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(s_tx.huart, &s_tx.buf[s_tx.tail], bytes);

    if (status != HAL_OK) {
        s_tx.bytes_in_dma = 0;
        // If it failed (e.g. HAL_BUSY), we'll try again next time send is called
    }
}

/* ── Callback from DMA stream interrupt ─────────────────────────────── */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_tx.huart) {
        /* DMA finished the chunk */
        s_tx.tail = (s_tx.tail + s_tx.bytes_in_dma) & TX_BUF_MASK;
        s_tx.bytes_in_dma = 0;

        /* Start next chunk if buffer isn't empty */
        start_dma_transfer();
    }
}

/* ── Public init ────────────────────────────────────────────────────── */
void uart_dma_tx_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_tx)
{
    s_tx.huart = huart;
    s_tx.head = s_tx.tail = 0;
    s_tx.bytes_in_dma = 0;

    /* Ensure the DMA handle is linked (CubeMX already does this, but safe) */
    __HAL_LINKDMA(huart, hdmatx, *hdma_tx);
}

/* ── Non‑blocking push (called from printf / putchar) ──────────────── */
void uart_dma_tx_send(const uint8_t *data, uint16_t len)
{
    if (len == 0) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t free = (TX_BUF_SIZE - 1) - ((s_tx.head - s_tx.tail) & TX_BUF_MASK);
    if (len > free) len = free;               /* discard on overflow */

    for (uint16_t i = 0; i < len; i++) {
        s_tx.buf[s_tx.head] = data[i];
        s_tx.head = (s_tx.head + 1) & TX_BUF_MASK;
    }

    __set_PRIMASK(primask);

    /* Start DMA only if it is currently idle */
    if (s_tx.bytes_in_dma == 0) {
        start_dma_transfer();
    }
}
