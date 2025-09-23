#include "State/Headers/FlightStates.h"

void PreArmingState::update(FlightManager& manager) {

	// Armスイッチをチェック
	if(manager.sbus_data.arm){

		//ESCの初期化をすませておく
		manager.pwm.InitMotor();

		if (manager.sbus_data.autofly){
			static uint8_t autofly_count = 0;
			autofly_count ++;
			if (autofly_count % 50 == 0){
				manager.yellow_led.Set(PinState::toggle);
			}

			if (autofly_count >= 2000){
				manager.yellow_led.Set(PinState::on);

				//CalibrationStateに遷移
				manager.changeState(std::make_unique<CalibrationState>());
				autofly_count = 0;
			}
			
		}
		else if(manager.sbus_data.emergency_control){
			static uint8_t emergency_count = 0;
			emergency_count++;

			//対故障で飛行することの確認

			if (emergency_count % 50 == 0) {
				manager.red_led.Set(PinState::toggle);
			}

			if (emergency_count >= 2000) {
				manager.red_led.Set(PinState::on);

				manager.changeState(std::make_unique<EmergencyControlState>());
				emergency_count = 0;
			}

		}
		else{
			//黄LEDをつける
			manager.yellow_led.Set(PinState::on);

			manager.changeState(std::make_unique<PreFlightState>());
		}
		
		return;
	}

	// Servo判定とPwm出力(abc_value = 0)
	manager.pwm.CalcServo(manager.sbus_data, 0, manager.control_data.servo_pwm);
	manager.pwm.GenerateServo(manager.control_data.servo_pwm);
}
