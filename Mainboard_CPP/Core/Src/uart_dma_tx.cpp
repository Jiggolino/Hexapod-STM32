#include "uart_dma_tx.h"
#include <string.h>

#define TX_BUF_SIZE   1024u
#define TX_BUF_MASK   (TX_BUF_SIZE - 1u)

/* Plain .bss placement (no NOLOAD): startup code zeros this struct, so
 * s_tx.huart starts as NULL and the early-return guards below are
 * meaningful even on cold boot. Previously this lived in a (NOLOAD)
 * section that was NOT zeroed → garbage huart pointer → bus fault when
 * touched before uart_dma_tx_init(). 32-byte alignment is preserved for
 * D-cache line cleanness in case caches are enabled later. */
static struct {
    uint8_t            buf[TX_BUF_SIZE];
    volatile uint16_t  head;
    volatile uint16_t  tail;
    volatile uint16_t  bytes_in_dma;
    UART_HandleTypeDef *huart;
} s_tx __attribute__((aligned(32)));

/* ── Try to launch a new DMA transfer ──────────────────────────────── */
static void start_dma_transfer(void)
{
    if (s_tx.huart == nullptr) return;     /* not initialized yet */
    if (s_tx.bytes_in_dma > 0) return;

    uint16_t space = (s_tx.head - s_tx.tail) & TX_BUF_MASK;
    if (space == 0) return;

    uint16_t end  = TX_BUF_SIZE - s_tx.tail;
    uint16_t bytes = (space < end) ? space : end;

    /* D-cache is disabled (no SCB_EnableDCache call anywhere in this build),
     * so the cache-clean-by-address operation that used to be here is both
     * unnecessary AND was throwing imprecise bus errors at the trailing DSB
     * on this Cortex-M7. If D-cache is ever enabled, restore the clean
     * (or place s_tx in a non-cacheable MPU region). */

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
    if (s_tx.huart == nullptr) return;     /* spurious callback before init */
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
    if (s_tx.huart == nullptr) return;     /* drop output before init */

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
