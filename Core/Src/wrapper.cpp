#include "wrapper.hpp"
#include "tim.h"
#include "usart.h"
#include "FlightManager.h"
#include "Sbus\sbus.h"
#include "Utils/SbusDecoder.hpp"
#include "Utils/SbusDebug.hpp"


#include "ESP32_P2P_Utility/P2PPacketDecoder.hpp"
P2PPacketDecoder decoder;
uint8_t receive_data[22] = {};
bool received = false;


FlightManager flightManager;
FlightLoopManager flightLoopManager;
nokolat::SBUS sbus;
nokolat::SBUS_DATA sbus_data;
SbusChannelData decoded_sbus_data;


static uint32_t time_count = 0;

// ESC calibration at startup:
// If you want the firmware to run ESC calibration sequence in init(),
// enable the macro below (uncomment). Alternatively, define ESC_CALIBRATION
// in your build 9flags (e.g. -DESC_CALIBRATION).
//#define ESC_CALIBRATION
#include "Utils/PWM.hpp"



void init(){
    #ifdef ESC_CALIBRATION
        // Run ESC calibration sequence if enabled at compile time.

        PwmCalibrateESC();
    #endif

    
    DebugSbus::overrideData.arm = true;
    DebugSbus::overrideData.throttle = 0;
    DebugSbus::overrideData.fly = true;
    DebugSbus::overrideData.autofly = true;
    DebugSbus::enableOverride(true);

	//UART5(DMA) SBUS受信用
	HAL_UART_Receive_DMA(&huart5, sbus.getReceiveBufferPtr(), sbus.getDataLen());

    //UART3(DMA) ESPからのデータ受信用
    HAL_UART_Receive_DMA(&huart3, receive_data, 22);

	//TIM6(400hz 割り込み） メインループ管理用
	HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_Base_Start_IT(&htim7);
}

void loop(){

	// ループ管理フラグのリセット待機
    if(flightLoopManager.isWait() == false) {
        
    	// ループ管理フラグをセット
        flightLoopManager.setWaitFlag();
        //printf("Loop Time: %d us\n", time_count*100);

        // 状態ごとの処理の呼び出し
        flightManager.update();
        
        
    }
    if (received) {
//        received = false;
//        for (uint8_t i=0;i<22;i++){
//			printf("%d ",receive_data[i]);
//		}

    	decoder.SetData(receive_data, 22);
		decoder.GetData(PacketDataType::Pitch, flightManager.autopilot_data.pitch);
		decoder.GetData(PacketDataType::Roll, flightManager.autopilot_data.roll);
		decoder.GetData(PacketDataType::Yaw, flightManager.autopilot_data.yaw);
		decoder.GetData(PacketDataType::Throttle, flightManager.autopilot_data.throttle);

        printf("%d %d %d %d\n", flightManager.autopilot_data.pitch, flightManager.autopilot_data.roll, flightManager.autopilot_data.yaw, flightManager.autopilot_data.throttle);
        received = false;
    }
}

// タイマー割り込み
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	// TIM6(400hz 割り込み） メインループ管理用
    if(htim == &htim6){

    	// ループ管理フラグをセット
        flightLoopManager.clearWaitFlag();
        time_count = 0;
    }
    if(htim == &htim7){
		time_count ++;
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
		// 正常に受信できている場合
		if(receive_data[0] == 0x0f && receive_data[21] == 0xf0){

			received = true;
		}

		//割り込み受信の再開
		HAL_UART_Receive_DMA(&huart3, receive_data, 22);

	}

}
