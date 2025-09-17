#include "State/Headers/FlightStates.h"

void PreArmingState::update(FlightManager& manager) {

	// リポ電圧チェック
	if(!manager.lipo_check.CheckVoltage()){

		// 電圧が足りない場合はエラーで終了
		printf("LIPO_VOLTAGE_ERROR\n");
		return;
	}

	// Armスイッチをチェック
	if(manager.sbus_data.arm){

		//ESCの初期化をすませておく
		manager.pwm.InitMotor();

		// センサーのキャリブレーション
		manager.imuUtil->Calibration(UserSetting::calibration_count);

		//黄LEDをつける
		manager.yellow_led.Set(PinState::on);

		//PreFlightStateに遷移
		manager.changeState(std::make_unique<PreFlightState>());
	}

	// Servo判定とPwm出力(abc_value = 0)
	manager.pwm.CalcServo(manager.sbus_data, 0, manager.control_data.servo_pwm);
	manager.pwm.GenerateServo(manager.control_data.servo_pwm);
}
