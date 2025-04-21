#ifndef INC_INTERRUPT_HPP_
#define INC_INTERRUPT_HPP_

#include <cstdint>
#include "flight_data.hpp"

void SbusInit(){

	HAL_UART_Receive_DMA(&huart5, data.raw_sbus, 25);
}

void SbusDeInit(){

	HAL_UART_DMAStop(&huart5);
}

void FailsafeInit(){

	//Failsafe
	HAL_TIM_Base_Start_IT(&htim5);
}

void FailsafeDeInit(){

	//Failsafe
	HAL_TIM_Base_Stop_IT(&htim5);
}

void ArmIQRInit(){

	//ArmLoop
	HAL_TIM_Base_Start_IT(&htim6);

	//IMU
	HAL_TIM_Base_Start_IT(&htim7);
}

void ArmIQRDeinit(){

	//ArmLoop
	HAL_TIM_Base_Stop_IT(&htim6);

	//IMU
	HAL_TIM_Base_Stop_IT(&htim7);
}

#endif
