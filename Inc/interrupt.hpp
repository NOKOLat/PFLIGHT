#ifndef INC_INTERRUPT_HPP_
#define INC_INTERRUPT_HPP_

#include <cstdint>
#include "usart.h"

void SbusInit(UART_HandleTypeDef* uart_pin, uint8_t* raw_sbus){

	HAL_UART_Receive_DMA(uart_pin, raw_sbus, 25);
}


void SbusDeInit(UART_HandleTypeDef* uart_pin){

	HAL_UART_DMAStop(uart_pin);
}

void ArmIQRInit(){

	//ArmLoop
	HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_Base_Start_IT(&htim7);

}

void ArmIQRDeinit(){

	//ArmLoop
	HAL_TIM_Base_Stop_IT(&htim6);

	HAL_TIM_Base_Stop_IT(&htim7);

}

#endif
