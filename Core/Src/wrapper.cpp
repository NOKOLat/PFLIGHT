#include "wrapper.hpp"
#include "tim.h"
#include "usart.h"
#include "FlightManager.h"
#include "Sbus\sbus.h"
#include "Utils/SbusDecoder.hpp"

FlightManager flightManager;
FlightLoopManager flightLoopManager;
nokolat::SBUS sbus;
nokolat::SBUS_DATA sbus_data;
SbusChannelData decoded_sbus_data;
uint8_t esp_data_buffer[17];

void init(){

	//UART5(DMA) SBUS受信用
	HAL_UART_Receive_DMA(&huart5, sbus.getReceiveBufferPtr(), sbus.getDataLen());

    //UART3(DMA) ESPからのデータ受信用
    HAL_UART_Receive_DMA(&huart3, esp_data_buffer, sizeof(esp_data_buffer));

	//TIM6(400hz 割り込み） メインループ管理用
	HAL_TIM_Base_Start_IT(&htim6);
}

void loop(){

	// ループ管理フラグのリセット待機
    if(flightLoopManager.isWait() == false) {

    	// ループ管理フラグをセット
        flightLoopManager.setWaitFlag();
        
        // 状態ごとの処理の呼び出し
        flightManager.update();
    }
}

// タイマー割り込み
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	// TIM6(400hz 割り込み） メインループ管理用
    if(htim == &htim6){

    	// ループ管理フラグをセット
        flightLoopManager.clearWaitFlag();
    }
}

// UART割り込み
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	// UART5(DMA) SBUS受信用
    if(huart == &huart5){

    	// 各チャンネルのデータを取得
    	sbus.parse();
        sbus_data = sbus.getData();

        // スイッチ判定などの処理を実行
        decoded_sbus_data = nokolat::Decode(sbus_data);

        // 判定結果をFlightManagerに渡す
        flightManager.sbusUpdate(decoded_sbus_data);

        // 受信の再開
        HAL_UART_Receive_DMA(&huart5, sbus.getReceiveBufferPtr(), sbus.getDataLen());
    }

    // UART3(DMA)
    if(huart == &huart3){
        
    }
}
