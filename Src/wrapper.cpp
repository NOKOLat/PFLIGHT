#include "wrapper.hpp"
#include "tim.h"
#include "usart.h"

#include <stdio.h>
#include "flight_manager.hpp"
#include "sbus.h"

FlightManager flight;
nokolat::SBUS sbus;
nokolat::SBUS_DATA sbus_data;

bool loop_wait_frg = false;
uint8_t sbus_timeout_count = 0;

void init(){

	HAL_UART_Receive_DMA(&huart5, sbus.getReceiveBufferPtr(), sbus.getDataLen());

	HAL_TIM_Base_Start_IT(&htim6);
}

void loop(){

	if(loop_wait_frg == false){

		loop_wait_frg = true;

		//状態判定と処理
		flight.UpDate();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//400hz割り込み
	if(htim == &htim6){

		loop_wait_frg = false;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//SBUSの受信処理
	//ライブラリを使用したあと、uint16_t[10]に変換する
    if(huart == &huart5){

        sbus.parse();
        sbus_data = sbus.getData();

		// dataメンバを直接渡す（関数でなければ）
		flight.SbusUpDate(&sbus_data.data[0], sbus_data.failsafe);

        //受信再開
        HAL_UART_Receive_DMA(&huart5, sbus.getReceiveBufferPtr(), sbus.getDataLen());
    }
}
