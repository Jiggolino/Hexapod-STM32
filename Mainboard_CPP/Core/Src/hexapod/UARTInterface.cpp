#include "hexapod/UARTInterface.hpp"

/*
 * Initialises the UART protocol module: zeroes the RX ring buffer, enables the
 * RXNE interrupt, retargets printf to the DMA TX ring buffer, and sets stdout
 * unbuffered so command responses go out immediately.
 * Input:  cfg — {huart, pca_right, pca_left, loko} handles used by command handlers
 * Output: void
 */
void UARTInterface::init(const UART_Protocol_Config_t &cfg)
{
    UART_Protocol_Init(&cfg);
}

/*
 * Drains bytes received since the last call, parses complete /CATEGORY/ARG lines,
 * dispatches command handlers, and sends one 50 Hz telemetry frame if streaming
 * is enabled.
 * Output: void
 */
void UARTInterface::update()
{
    UART_Update();
}

/*
 * Returns a reference to the latest controller state decoded from /CONTROLL packets.
 * Stick axes are normalised to [-1, 1]; buttons are bitmask fields.
 * Output: const ref valid until the next update() call
 */
const UART_ControllerState_t& UARTInterface::controller() const
{
    return *UART_GetController();
}
