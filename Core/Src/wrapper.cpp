#include "wrapper.hpp"
#include "tim.h"
#include "usart.h"
#include "FlightManager.h"
#include "Sbus\sbus.h"
#include "Utils/SbusDecoder.hpp"
#include "Utils/SbusDebug.hpp"
#include "Utils/LED.hpp"

#include "ESP32_P2P_Utility/P2PPacketDecoder.hpp"
P2PPacketDecoder decoder;
// Packet layout: [start(1)] + N * [type(1) + 4 bytes data] + [end(1)]
// Current PacketDataType defines 2 entries (State, Roll) -> packet size = 1 + 2*5 + 1 = 12
constexpr uint8_t P2P_PACKET_SIZE = 12;
uint8_t receive_data[P2P_PACKET_SIZE] = {};
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

    
    // DebugSbus::overrideData.arm = true;
	// DebugSbus::overrideData.throttle = 0;
    // DebugSbus::overrideData.fly = true;
    // DebugSbus::overrideData.autofly = true;
    // DebugSbus::enableOverride(true);

	//UART5(DMA) SBUS受信用
	HAL_UART_Receive_DMA(&huart5, sbus.getReceiveBufferPtr(), sbus.getDataLen());

	//UART3(DMA) ESPからのデータ受信用
	HAL_UART_Receive_DMA(&huart3, receive_data, P2P_PACKET_SIZE);

	//TIM6(400hz 割り込み） メインループ管理用
	HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_Base_Start_IT(&htim7);

	printf("start\n");
}

void loop(){
	
	//ループ管理フラグのリセット待機
	if(flightLoopManager.isWait() == false) {
		//HAL_Delay(1);

		// ループ管理フラグをセット
		flightLoopManager.setWaitFlag();

		// 状態ごとの処理の呼び出し
		flightManager.update();

//		printf("%d us\n", (int)(time_count*100.0f - 1000));
//		time_count = 0;

		if (received) {
//			for (uint8_t i=0;i<22;i++){
//				printf("%d ",receive_data[i]);
//			}
			//printf("R\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)"0", 1, 100);

			decoder.SetData(receive_data, P2P_PACKET_SIZE);
			decoder.GetData(PacketDataType::State, flightManager.autopilot_data.state);
			decoder.GetData(PacketDataType::Roll, flightManager.autopilot_data.roll);

			//printf("%d %d\n", (int)flightManager.autopilot_data.state, (int)flightManager.autopilot_data.roll);
			received = false;
		}


	}

}

//タイマー割り込み
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

//	if(htim == &htim7){
//		if (flightLoopManager.isWait() == false){
//			time_count ++;
//		}
//
//	}

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
		// 正常に受信できている場合
		// check start and end markers; end marker is at index (received size - 1)
		if(receive_data[0] == 0x0f && receive_data[P2P_PACKET_SIZE - 1] == 0xf0){
			received = true;
		}
		//割り込み受信の再開
		HAL_UART_Receive_DMA(&huart3, receive_data, P2P_PACKET_SIZE);

	}

}
