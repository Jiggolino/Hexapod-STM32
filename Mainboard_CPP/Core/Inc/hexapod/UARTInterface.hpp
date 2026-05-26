#pragma once
#include "uart_protocol.h"

class UARTInterface {
public:
    void init(const UART_Protocol_Config_t &cfg);
    void update();
    void rxCallback();

    const UART_ControllerState_t& controller() const;
    UART_RobotState_t state() const;
    const UART_LED_t* rightLEDs() const;
    const UART_LED_t* leftLEDs()  const;
    const UART_LED_t* controllerLED() const;
};
