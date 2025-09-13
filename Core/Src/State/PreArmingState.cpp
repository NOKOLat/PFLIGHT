#include "State/Headers/FlightStates.h"

void PreArmingState::update(FlightManager& manager) {

	// Armスイッチをチェック
	if(manager.sbus_data.arm){

		//ESCの初期化をすませておく
		manager.pwm.InitMotor();

	//黄LEDをつける
	manager.yellow_led.Set(PinState::on);

		//CalibrationStateに遷移
		manager.changeState(std::make_unique<CalibrationState>());
		return;
	}

	// Servo判定とPwm出力(abc_value = 0)
	manager.pwm.CalcServo(manager.sbus_data, 0, manager.control_data.servo_pwm);
	manager.pwm.GenerateServo(manager.control_data.servo_pwm);
}
