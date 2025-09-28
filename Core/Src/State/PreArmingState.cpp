#include "State/Headers/FlightStates.h"

void PreArmingState::Enter(FlightManager& manager) {

	// 処理なし
}

void PreArmingState::Update(FlightManager& manager) {

	// Armスイッチをチェック
	if(manager.sbus_data.arm){

		//PreFlightStateに遷移
		manager.changeState(std::make_unique<PreFlightState>());
	}

	// Servo判定とPwm出力(abc_value = 0)
	manager.pwm.CalcServo(manager.sbus_data, 0, manager.control_data.servo_pwm);
	manager.pwm.GenerateServo(manager.control_data.servo_pwm);
}

void PreArmingState::Exit(FlightManager& manager) {

	//ESCの初期化をすませておく
	manager.pwm.InitMotor();

	// センサーのキャリブレーション
	manager.imuUtil->Calibration(UserSetting::calibration_count);

	//黄LEDをつける
	manager.yellow_led.Set(PinState::on);
}