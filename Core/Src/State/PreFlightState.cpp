#include "State/Headers/FlightStates.h"

void PreFlightState::update(FlightManager& manager) {

	// Armスイッチの判定
	if(!manager.sbus_data.arm){

		// Disarm状態に遷移
		manager.changeState(std::make_unique<DisarmingState>());
	}

	// Servo判定とPwm出力(abc_value = 0)
	manager.pwm.CalcServo(manager.sbus_data, 0, manager.control_data.servo_pwm);
	manager.pwm.GenerateServo(manager.control_data.servo_pwm);

	// Flightスイッチの判定
	if(manager.sbus_data.fly){

		//PIDの初期化
		// 各PIDインスタンスのリセット
		manager.angle_pitch.reset();
		manager.angle_roll.reset();
		manager.rate_pitch.reset();
		manager.rate_roll.reset();
		manager.rate_yaw.reset();

		//Madgwickフィルターの初期化(400hz)
		manager.madgwick.begin(UserSetting::MadgwickSampleFreq);

		//飛行用LEDをつける
		manager.green_led.Set(PinState::on);

		// 飛行状態に遷移
		manager.changeState(std::make_unique<FlyingState>());
	}
}
