#include "wrapper.hpp"
#include "tim.h"
#include "usart.h"
#include "FlightManager.h"
#include "Sbus\sbus.h"
#include "Utils/SbusDecoder.hpp"

#include "Utils/SbusDebug.hpp"
#include "Utils/P2PReceiver.hpp"


FlightManager flightManager;
FlightLoopManager flightLoopManager;
nokolat::SBUS sbus;
nokolat::SBUS_DATA sbus_data;
SbusChannelData decoded_sbus_data;
P2PPackage::P2PReceiver p2p_receiver;


// ESC calibration at startup:
// If you want the firmware to run ESC calibration sequence in init(),
// enable the macro below (uncomment). Alternatively, define ESC_CALIBRATION
// in your build 9flags (e.g. -DESC_CALIBRATION).
#define ESC_CALIBRATION
#include "Utils/PWM.hpp"

void init(){
    #ifdef ESC_CALIBRATION
        // Run ESC calibration sequence if enabled at compile time.

        PwmCalibrateESC();
    #endif

    
    DebugSbus::overrideData.arm = true;
    DebugSbus::overrideData.throttle = 0;
    DebugSbus::overrideData.fly = 0;
    DebugSbus::enableOverride(true);

	//UART5(DMA) SBUS受信用
	HAL_UART_Receive_DMA(&huart5, sbus.getReceiveBufferPtr(), sbus.getDataLen());

    //UART3(DMA) ESPからのデータ受信用
    HAL_UART_Receive_DMA(&huart3, p2p_receiver.getReceiveBufferPtr(), p2p_receiver.getDataLen());

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
    p2p_receiver.Process(flightManager);

    // restart UART3 DMA receive
    HAL_UART_Receive_DMA(&huart3, p2p_receiver.getReceiveBufferPtr(), p2p_receiver.getDataLen());
    }
}
