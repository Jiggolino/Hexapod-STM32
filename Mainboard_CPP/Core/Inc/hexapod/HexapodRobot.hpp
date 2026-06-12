#pragma once
#include "ServoDriver.hpp"
#include "IMUSensor.hpp"
#include "ToFSensor.hpp"
#include "BatteryMonitor.hpp"
#include "LEDController.hpp"
#include "UARTInterface.hpp"
#include "Locomotion.hpp"
#include "Controller.hpp"

class HexapodRobot {
public:
    ServoDriver    servoRight;
    ServoDriver    servoLeft;
    IMUSensor      imu;
    ToFSensor      tof;
    BatteryMonitor battery;
    LEDController  leds;
    UARTInterface  uart;
    Locomotion     loko;
    Controller&    controller;

    HexapodRobot(I2C_HandleTypeDef *hi2c,
                 TIM_HandleTypeDef *htim,
                 UART_HandleTypeDef *huart,
                 ADC_HandleTypeDef *hadc3);

    bool init();
    void update(float dt);

private:
    I2C_HandleTypeDef  *_hi2c;
    TIM_HandleTypeDef  *_htim;
    UART_HandleTypeDef *_huart;
    ADC_HandleTypeDef  *_hadc3;

    uint32_t _errFlags;
    uint16_t _tofDistMm;

    uint32_t _imuLastMs;
    uint32_t _tofLastMs;
    uint32_t _lokoLastMs;
};
