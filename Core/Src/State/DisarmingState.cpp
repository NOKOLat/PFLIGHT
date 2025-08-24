#include "State/Headers/FlightStates.h"

void DisarmingState::update(FlightManager& manager) {

	// Pwmの停止
	PwmStop();

	// 各PIDインスタンスのリセット
	manager.angle_pitch.reset();
	manager.angle_roll.reset();
	manager.rate_pitch.reset();
	manager.rate_roll.reset();
	manager.rate_yaw.reset();

	// PreArmingへの遷移
	manager.changeState(std::make_unique<PreArmingState>());
}

void DisarmingState::enter(FlightManager& manager) {



}

void DisarmingState::exit(FlightManager& manager) {

	yellowLed(PinState::off);
	greenLed(PinState::off);
}
