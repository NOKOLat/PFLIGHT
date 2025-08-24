#include "FlightManager.h"
#include "state/headers/FlightStates.h"
#include <iomanip>
#include <cstring>

// コンストラクタ
FlightManager::FlightManager() {
    
    // IMUユーティリティのインスタンス化
    imuUtil = new ICM42688P_SPI_Util(UserSetting::imuPinSetting.spi_pin, UserSetting::imuPinSetting.gpio_port, UserSetting::imuPinSetting.gpio_pin);

    // 初期状態をInitStateに設定
    current_state = std::make_unique<InitState>();
    current_state->enter(*this);
}

// 状態更新
void FlightManager::changeState(std::unique_ptr<FlightStateInterface> new_state) {

    if (current_state) {

        current_state->exit(*this);
    }
    current_state = std::move(new_state);

    // 状態名をprintfで出力
    if (current_state) {
        printf("[FlightManager] 状態遷移: %s\n", current_state->getStateName());
    }
    current_state->enter(*this);
}

// 状態呼び出し(400hz)
void FlightManager::update() {

    // SBUS接続チェック（Init状態の時以外）
    if (current_state && std::strcmp(current_state->getStateName(), "InitState") != 0) {

        // SBUSのタイムアウトチェック用
        sbus_lost_count ++;
        if (!checkSbusConnect()) {

            changeState(std::make_unique<FailSafeState>());
            return;
        }
    }
    if (current_state) {

        current_state->update(*this);
    }
}

// SBUS更新
void FlightManager::sbusUpdate(SbusChannelData sbus_data){

	this->sbus_data = sbus_data;
	sbus_lost_count = 0;
}

// SBUS接続チェック
bool FlightManager::checkSbusConnect(){

	if(sbus_data.failsafe_bit || (sbus_lost_count > 1000)){

		return false;
	}
    

	return true;
}
