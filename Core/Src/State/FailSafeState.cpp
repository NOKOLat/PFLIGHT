#include "State/Headers/FlightStates.h"

void FailSafeState::Update(FlightManager& manager) {

	// LEDの点滅
	while(1){

		// いい感じのdelayだいたい1sくらい
		for(volatile uint32_t i=0; i<1000000; i++);

		// Pwmの停止いっぱいやる
		manager.pwm.MotorStop();

		manager.red_led.Set(PinState::toggle);
		manager.yellow_led.Set(PinState::toggle);
		manager.green_led.Set(PinState::toggle);
	}
}

void FailSafeState::Enter(FlightManager& manager) {

	// Pwmの停止
	manager.pwm.MotorStop();

	// PIDのリセット
	manager.angle_pitch.reset();
	manager.angle_roll.reset();
	manager.rate_pitch.reset();
	manager.rate_roll.reset();
	manager.rate_yaw.reset();
}

void FailSafeState::Exit(FlightManager& manager) {

	// 処理なし
	// 普通に復帰できたらだめ
}