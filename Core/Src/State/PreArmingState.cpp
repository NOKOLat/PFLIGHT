#include "State/Headers/FlightStates.h"

void PreArmingState::update(FlightManager& manager) {

	// Armスイッチをチェック
	if(manager.sbus_data.arm){

		//ESCの初期化をすませておく
		PwmInitMotor();

		//黄LEDをつける
		yellowLed(PinState::on);

		//CalibrationStateに遷移
		manager.changeState(std::make_unique<CalibrationState>());
		return;
	}

	// Servo判定とPwm出力(abc_value = 0)
	PwmCalcServo(manager.sbus_data, 0, manager.control_data.servo_pwm);
	PwmGenerateServo(manager.control_data.servo_pwm);
}
