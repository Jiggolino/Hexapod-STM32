#include "hexapod/UARTInterface.hpp"

void UARTInterface::init(const UART_Protocol_Config_t &cfg)
{
    UART_Protocol_Init(&cfg);
}

void UARTInterface::update()
{
    UART_Update();
}

void UARTInterface::rxCallback()
{
    UART_Protocol_RX_Callback();
}

const UART_ControllerState_t& UARTInterface::controller() const
{
    return *UART_GetController();
}

UART_RobotState_t UARTInterface::state() const
{
    return UART_GetState();
}

const UART_LED_t* UARTInterface::rightLEDs() const      { return UART_GetRightLEDs(); }
const UART_LED_t* UARTInterface::leftLEDs()  const      { return UART_GetLeftLEDs(); }
const UART_LED_t* UARTInterface::controllerLED() const  { return UART_GetControllerLED(); }
