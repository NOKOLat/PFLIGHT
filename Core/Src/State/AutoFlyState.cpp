#include "State/Headers/FlightStates.h"

void AutoFlyState::update(FlightManager& manager) {

    // ループカウント
	static uint32_t loop_count = 0;
	loop_count++;

	float target_value[3] = {}; // pitch, roll, yaw
	static float altitude_value = 0.0f;

    // Armのチェック
	if(!manager.sbus_data.arm){

		manager.changeState(std::make_unique<DisarmingState>());
		return;
	}

    // 通常飛行への復帰
    if(!manager.sbus_data.autofly){

		manager.changeState(std::make_unique<FlyingState>());
		return;
	}
		// scale
		
		target_value[0] = (manager.autopilot_data.pitch / 127.0f) * 10.0f; // degrees
		target_value[1] = (manager.autopilot_data.roll / 127.0f) * 10.0f; // degrees
		target_value[2] = (manager.autopilot_data.yaw / 127.0f) * 30.0f; // dps

		// throttle_assist used as altitude target proxy (meters)
		altitude_value = (manager.autopilot_data.throttle / 255.0f) * 1.0f; // meters
		printf("%f m",altitude_value);


	// Madgwickフィルターでの姿勢推定
	manager.madgwick.updateIMU(

		manager.sensor_data.gyro[0], manager.sensor_data.gyro[1], manager.sensor_data.gyro[2],
		manager.sensor_data.accel[0], manager.sensor_data.accel[1], manager.sensor_data.accel[2]
	);

	// 推定データの取得
	manager.sensor_data.angle[0] = manager.madgwick.getPitch();
	manager.sensor_data.angle[1] = manager.madgwick.getRoll();
	manager.sensor_data.angle[2] = manager.madgwick.getYaw();

    // センサー向きの調整
    float buf = manager.sensor_data.gyro[0];
    manager.sensor_data.gyro[0]  = manager.sensor_data.gyro[1];
    manager.sensor_data.gyro[1]  = buf;
    manager.sensor_data.gyro[2] *= -1;


	// 100hz 角度制御(pitch, roll)
    if(loop_count % 4 == 0){

    	// 目標角と現在角から目標角速度を計算
		// pitch, rollの目標角速度計算: calc() を呼んだ後 getData() で結果を取得
		manager.angle_pitch.calc(target_value[0], manager.sensor_data.angle[0]);
		manager.angle_pitch.getData(&manager.control_data.target_rate[0]);
		manager.angle_roll.calc(target_value[1], manager.sensor_data.angle[1]);
		manager.angle_roll.getData(&manager.control_data.target_rate[1]);

    	// yaw軸はセンサーデータを使用
        manager.control_data.target_rate[2] = target_value[2];


		// --- 気圧の取得 ---
		float pressure_Pa = 0.0f;
		float temperature_C = 0.0f;
		// 可能ならバーオメータから取得（FlightManager に保持される dps368 を使用）
		if (manager.dps368) {
			if (manager.dps368->getData(&pressure_Pa, &temperature_C) == 0) {
				// legacy の uint16_t フィールドに収める場合は切り詰め
				manager.sensor_data.pressure = pressure_Pa;
			}
		}

		altitude.Update(pressure_Pa, manager.sensor_data.accel.data(), manager.sensor_data.angle.data(), 0.01f);
		

    }

    // 400hz 角速度制御
	//目標角速度と現在角速度(センサーデータ）から制御量を計算
	// pitch, roll, yawの制御量計算: calc() を呼んだ後 getData() で結果を取得
	manager.rate_pitch.calc(manager.control_data.target_rate[0], manager.sensor_data.gyro[0]);
	manager.rate_pitch.getData(&manager.control_data.pid_result[0]);
	manager.rate_roll.calc(manager.control_data.target_rate[1], manager.sensor_data.gyro[1]);
	manager.rate_roll.getData(&manager.control_data.pid_result[1]);
	manager.rate_yaw.calc(manager.control_data.target_rate[2], manager.sensor_data.gyro[2]);
	manager.rate_yaw.getData(&manager.control_data.pid_result[2]);

	// 上のモーターはスロットル + 制御出力をミキシング
	PwmCalcMainMotor(manager.sbus_data.throttle, manager.control_data.pid_result, manager.control_data.upper_motor_pwm);

	// 下のモーターはスロットルのみに比例
	PwmCalcSubMotor(manager.sbus_data.throttle, manager.control_data.lower_motor_pwm);


	PwmGenerate(manager.control_data.upper_motor_pwm, manager.control_data.lower_motor_pwm, manager.control_data.servo_pwm);

}

void AutoFlyState::enter(FlightManager& manager) {
    
}

void AutoFlyState::exit(FlightManager& manager) {

}
