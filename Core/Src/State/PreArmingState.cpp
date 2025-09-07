#include "State/Headers/FlightStates.h"

void PreArmingState::update(FlightManager& manager) {

	// Armスイッチをチェック
	if(manager.sbus_data.arm){

		//ESCの初期化をすませておく
		manager.pwm.initMotor();

		// センサーのキャリブレーション
		manager.imuUtil->calibration(UserSetting::calibration_count);

		//黄LEDをつける
		yellowLed(PinState::on);

		//PreFlightStateに遷移
		manager.changeState(std::make_unique<PreFlightState>());
	}

	// Servo判定とPwm出力(abc_value = 0)
	manager.pwm.calcServo(manager.sbus_data, 0, manager.control_data.servo_pwm);
	manager.pwm.generateServo(manager.control_data.servo_pwm);
}
