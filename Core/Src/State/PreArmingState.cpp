#include "State/Headers/FlightStates.h"

void PreArmingState::update(FlightManager& manager) {

	// Armスイッチをチェック
	if(manager.sbus_data.arm){
		static uint16_t prearm_count=0;

		if (prearm_count == 0){
			//ESCの初期化をすませておく
			manager.pwm.InitMotor();
		}


		if (manager.sbus_data.autofly){
			prearm_count ++;
			if (prearm_count % 50 == 0){
				manager.yellow_led.Set(PinState::toggle);
			}

			if (prearm_count >= 2000){
				manager.yellow_led.Set(PinState::on);

				//CalibrationStateに遷移
				manager.changeState(std::make_unique<CalibrationState>());
				prearm_count = 0;
			}
			
		}
		else if(manager.sbus_data.emergency_control){
			prearm_count++;

			//対故障で飛行することの確認

			if (prearm_count % 50 == 0) {
				manager.red_led.Set(PinState::toggle);
			}

			if (prearm_count >= 2000) {
				manager.red_led.Set(PinState::on);

				manager.changeState(std::make_unique<PreFlightState>());
				prearm_count = 0;
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
