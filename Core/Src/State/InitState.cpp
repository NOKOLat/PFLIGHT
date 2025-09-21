#include "State/Headers/FlightStates.h"
#include "UserSetting/PIDSetting.hpp"
#include "UserSetting/LEDSetting.hpp"

void InitLED(FlightManager& manager);
void InitPIDFromUserSetting(FlightManager& manager);

void InitState::Update(FlightManager& manager) {

	// SBUSの受信チェック
	if(!manager.sbus_data.is_receive){

		printf("SBUS_ERROR \n");
		return;
	}

	// IMUの通信チェック
	if (manager.imuUtil) {

		if (manager.imuUtil->Init() != 0) {

			printf("IMU_ERROR \n");
			return;
		}
	}
	
	// Motorの設定チェック
	if(manager.pwm.CheckMotorSetting(motor_count)){

		printf("MotorSetting_Error\n");
		return;
	}

	// Servoの初期化
	manager.pwm.InitServo();

	// 赤LEDをつける
	manager.red_led.Set(PinState::on);

	// PreArmStateへの遷移
	manager.changeState(std::make_unique<PreArmingState>());
}

void InitState::Enter(FlightManager& manager) {

	printf("FC start \n");

	//PWMの停止（安全のため）
	manager.pwm.MotorStop();

	// LED初期化
	InitLED(manager);

	// PIDの初期化（FlightManager の PID インスタンスに設定を反映）
	InitPIDFromUserSetting(manager);
}

// LEDの初期化関数
void InitLED(FlightManager& manager) {

	using namespace UserSetting;

	manager.red_led.LEDInit(redLedSetting.gpio_port, redLedSetting.gpio_pin);
	manager.yellow_led.LEDInit(yellowLedSetting.gpio_port, yellowLedSetting.gpio_pin);
	manager.green_led.LEDInit(greenLedSetting.gpio_port, greenLedSetting.gpio_pin);
}


// PIDの初期化関数
void InitPIDFromUserSetting(FlightManager& manager) {

	using namespace UserSetting;

    manager.angle_pitch.setGain(angle_pitch_gain.kp, angle_pitch_gain.ki, angle_pitch_gain.kd);
    manager.angle_pitch.setLimit(angle_pitch_limit.i_max, angle_pitch_limit.d_max);
    manager.angle_pitch.setTime(angle_pitch_dt.dt);

    manager.angle_roll.setGain(angle_roll_gain.kp, angle_roll_gain.ki, angle_roll_gain.kd);
    manager.angle_roll.setLimit(angle_roll_limit.i_max, angle_roll_limit.d_max);
    manager.angle_roll.setTime(angle_roll_dt.dt);

    manager.rate_pitch.setGain(rate_pitch_gain.kp, rate_pitch_gain.ki, rate_pitch_gain.kd);
    manager.rate_pitch.setLimit(rate_pitch_limit.i_max, rate_pitch_limit.d_max);
    manager.rate_pitch.setTime(rate_pitch_dt.dt);

    manager.rate_roll.setGain(rate_roll_gain.kp, rate_roll_gain.ki, rate_roll_gain.kd);
    manager.rate_roll.setLimit(rate_roll_limit.i_max, rate_roll_limit.d_max);
    manager.rate_roll.setTime(rate_roll_dt.dt);

    manager.rate_yaw.setGain(rate_yaw_gain.kp, rate_yaw_gain.ki, rate_yaw_gain.kd);
    manager.rate_yaw.setLimit(rate_yaw_limit.i_max, rate_yaw_limit.d_max);
    manager.rate_yaw.setTime(rate_yaw_dt.dt);
}
