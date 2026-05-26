#include "current.h"
#include "adc.h"
#include "main.h"
#include "stdint.h"

void get_current_servos(float buffer[8][2]){
	for (uint8_t i = 0; i < 8; i++) {
        if (i & (1 << 0)) {
            HAL_GPIO_WritePin(Right_A0_GPIO_Port, Right_A0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Left_A0_GPIO_Port, Left_A0_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(Right_A0_GPIO_Port, Right_A0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Left_A0_GPIO_Port, Left_A0_Pin, GPIO_PIN_RESET);
        }

        if (i & (1 << 1)) {
            HAL_GPIO_WritePin(Right_A1_GPIO_Port, Right_A1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Left_A1_GPIO_Port, Left_A1_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(Right_A1_GPIO_Port, Right_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Left_A1_GPIO_Port, Left_A1_Pin, GPIO_PIN_RESET);
        }

        if (i & (1 << 2)) {
            HAL_GPIO_WritePin(Right_A2_GPIO_Port, Right_A2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Left_A2_GPIO_Port, Left_A2_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(Right_A2_GPIO_Port, Right_A2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Left_A2_GPIO_Port, Left_A2_Pin, GPIO_PIN_RESET);
        }

        buffer[i][0] = ((3.3f*(float)ADC_ReadCurrentRight()/65536.0f) / (0.005f * 50.0f)); //0.005 ohm shunt 16bit adc res 50x opv
        buffer[i][1] = ((3.3f*(float)ADC_ReadCurrentLeft()/65536.0f) / (0.005f * 50.0f)); //0.005 ohm shunt 16bit adc res 50x opv

        HAL_Delay(1);
    }
	return;
}



float get_current_all_servos(){
	float total_current = 0;
	float current[8][2];

	get_current_servos(current);

	for(uint16_t i = 0; i < 8; i++){
		total_current += current[i][0];
		total_current += current[i][1];
	}
	return total_current;
}
