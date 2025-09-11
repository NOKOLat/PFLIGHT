#include "State/Headers/FlightStates.h"
#include "Utils/SbusDebug.hpp"
#include "UserSetting/PIDSetting.hpp"

void InitPIDFromUserSetting(FlightManager& manager);

void InitState::update(FlightManager& manager) {

	// SBUSの受信チェック
	if(!manager.sbus_data.is_receive){
		// If a debug override is enabled, apply it so initialization can continue
		if (DebugSbus::isOverrideEnabled()) {
			DebugSbus::applyOverride(manager.sbus_data);
		} else {
			printf("SBUS_ERROR \n");
			return;
		}
	}

	// IMUの通信チェック
	if (manager.imuUtil) {

		if (manager.imuUtil->init() != 0) {

			printf("IMU_ERROR \n");
			return;
		}
	}

	if (manager.dps368) {

		if (manager.dps368->init() != 0) {

			printf("BAROMETER_ERROR \n");
			return;
		}
	}

	// Servoの初期化
	PwmInitServo();

	// 赤LEDをつける
	redLed(PinState::on);

	// PreArmStateへの遷移
	manager.changeState(std::make_unique<PreArmingState>());
}

void InitState::enter(FlightManager& manager) {

	printf("FC start \n");

	//PWMの停止（安全のため）
	PwmStop();

	// PIDの初期化（FlightManager の PID インスタンスに設定を反映）
	InitPIDFromUserSetting(manager);
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
