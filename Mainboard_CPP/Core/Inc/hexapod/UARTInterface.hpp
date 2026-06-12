#pragma once
#include "uart_protocol.h"

class UARTInterface {
public:
    void init(const UART_Protocol_Config_t &cfg);
    void update();

    const UART_ControllerState_t& controller() const;
};
