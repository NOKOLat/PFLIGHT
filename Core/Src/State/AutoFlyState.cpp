#include "State/Headers/FlightStates.h"

void AutoFlyState::update(FlightManager& manager) {

//  ループカウント
static uint32_t loop_count = 0;
loop_count++;

    // Armのチェック
	if(!manager.sbus_data.arm){

		manager.changeState(std::make_unique<DisarmingState>());
	}

    // 通常飛行への復帰
    if(!manager.sbus_data.emergency_control){

		manager.changeState(std::make_unique<FlyingState>());
	}
}

void AutoFlyState::enter(FlightManager& manager) {
    
}

void AutoFlyState::exit(FlightManager& manager) {

}

