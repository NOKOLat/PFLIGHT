#include "State/Headers/FlightStates.h"

void AutoFlyState::update(FlightManager& manager) {

    // ループカウント
	static uint32_t loop_count = 0;
	loop_count++;

	float target_value[3] = {}; // pitch, roll, yaw
	float altitude_value = 0.0f; // meters
	float estimated_data[3] = {};
	static float throttle = 0.0f;

	static float sum_accel[3] = {};
	float average_accel[3] = {};

	static float sum_angle[3] = {};
	float average_angle[3] = {};

	static float sum_pressure = 0.0f;
	float average_pressure = 0.0f;

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
	//target_value[2] = (manager.autopilot_data.yaw / 127.0f) * 30.0f; // dps
	target_value[2] = 0;

	// throttle_assist used as altitude target proxy (meters)
	altitude_value = (manager.autopilot_data.throttle / 255.0f) * 2.0f; // meters

	//IMUデータの取得
	manager.imuUtil->getData(manager.sensor_data.accel, manager.sensor_data.gyro);

	// Madgwickフィルターでの姿勢推定
	manager.madgwick.updateIMU(

		manager.sensor_data.gyro[0], manager.sensor_data.gyro[1], manager.sensor_data.gyro[2],
		manager.sensor_data.accel[0], manager.sensor_data.accel[1], manager.sensor_data.accel[2]
	);

	for (uint8_t i=0; i<3; i++){
		sum_accel[i] += manager.sensor_data.accel[i];
		sum_angle[i] += manager.sensor_data.angle[i];
	}

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

			// --- 気圧の取得と高度推定 ---
		// 8回に1回、50Hzで実行	
		float pressure_Pa = 0.0f;
		float temperature_C = 0.0f;
		// 可能ならバーオメータから取得（FlightManager に保持される dps368 を使用）
		if (manager.dps368) {
			if (manager.dps368->getData(&pressure_Pa, &temperature_C) == 0) {
				manager.sensor_data.pressure = pressure_Pa;
				// printf("P: %.4f Pa, T: %.2f C\n", pressure_Pa, temperature_C);
				
			}
		}
		// collect pressure (Pa) for averaging
		sum_pressure += manager.sensor_data.pressure;

    }

	if(loop_count %8 == 0){
		// 8回に1回、50Hzで実行
		for (uint8_t i=0; i<3; i++){
			average_accel[i] = sum_accel[i] / 8.0f;
			average_angle[i] = sum_angle[i] / 8.0f;
			average_pressure = sum_pressure / 2.0f;

			sum_accel[i] = 0.0f;
			sum_angle[i] = 0.0f;
			
		}
		sum_pressure = 0.0f;

		//printf("%+2.2f, %+2.2f, %.4f\n", average_accel[2], average_angle[0], average_pressure);
		
		altitude.Update(average_pressure, average_accel, average_angle, 0.02f);

		altitude.GetData(estimated_data);
		printf("%.1f cm\n", estimated_data[0]*100.0f); // cm 単位で出力

		float throttle_error = altitude_value - estimated_data[0]*10.0f;
		if (throttle_error > 10.0f){
			throttle += 10.0f;
		} 
		else if (throttle_error < -10.0f){
			throttle -= 10.0f;
		}
		else{
			throttle += throttle_error;
		}
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

	PwmCalcMainMotor(throttle, manager.control_data.pid_result, manager.control_data.upper_motor_pwm);

	PwmGenerate(manager.control_data.upper_motor_pwm, manager.control_data.lower_motor_pwm, manager.control_data.servo_pwm);

}

void AutoFlyState::enter(FlightManager& manager) {
	// Overwrite PID parameters for autopilot mode using UserSetting values
	// Angle PIDs
	manager.angle_pitch.reset();
	manager.angle_roll.reset();
	
	manager.rate_pitch.reset();
	manager.rate_roll.reset();
	manager.rate_yaw.reset();

	// manager.angle_pitch.setGain(UserSetting::angle_pitch_gain.kp, UserSetting::angle_pitch_gain.ki, UserSetting::angle_pitch_gain.kd);
	// manager.angle_pitch.setLimit(UserSetting::angle_pitch_limit.i_max, UserSetting::angle_pitch_limit.d_max);
	// manager.angle_pitch.setTime(UserSetting::angle_pitch_dt.dt);
	

	// manager.angle_roll.setGain(UserSetting::angle_roll_gain.kp, UserSetting::angle_roll_gain.ki, UserSetting::angle_roll_gain.kd);
	// manager.angle_roll.setLimit(UserSetting::angle_roll_limit.i_max, UserSetting::angle_roll_limit.d_max);
	// manager.angle_roll.setTime(UserSetting::angle_roll_dt.dt);
	

	// // Rate PIDs
	// manager.rate_pitch.setGain(UserSetting::rate_pitch_gain.kp, UserSetting::rate_pitch_gain.ki, UserSetting::rate_pitch_gain.kd);
	// manager.rate_pitch.setLimit(UserSetting::rate_pitch_limit.i_max, UserSetting::rate_pitch_limit.d_max);
	// manager.rate_pitch.setTime(UserSetting::rate_pitch_dt.dt);
	

	// manager.rate_roll.setGain(UserSetting::rate_roll_gain.kp, UserSetting::rate_roll_gain.ki, UserSetting::rate_roll_gain.kd);
	// manager.rate_roll.setLimit(UserSetting::rate_roll_limit.i_max, UserSetting::rate_roll_limit.d_max);
	// manager.rate_roll.setTime(UserSetting::rate_roll_dt.dt);
	

	manager.rate_yaw.setGain(UserSetting::rate_yaw_gain.kp*1.2f, UserSetting::rate_yaw_gain.ki, UserSetting::rate_yaw_gain.kd*2.0f);
	// manager.rate_yaw.setLimit(UserSetting::rate_yaw_limit.i_max, UserSetting::rate_yaw_limit.d_max);
	// manager.rate_yaw.setTime(UserSetting::rate_yaw_dt.dt);
	

}

void AutoFlyState::exit(FlightManager& manager) {
	// Restore PID parameters from UserSetting, then reset internal states once.

	// Angle PIDs
	manager.angle_pitch.setGain(UserSetting::angle_pitch_gain.kp, UserSetting::angle_pitch_gain.ki, UserSetting::angle_pitch_gain.kd);
	manager.angle_pitch.setLimit(UserSetting::angle_pitch_limit.i_max, UserSetting::angle_pitch_limit.d_max);
	manager.angle_pitch.setTime(UserSetting::angle_pitch_dt.dt);

	manager.angle_roll.setGain(UserSetting::angle_roll_gain.kp, UserSetting::angle_roll_gain.ki, UserSetting::angle_roll_gain.kd);
	manager.angle_roll.setLimit(UserSetting::angle_roll_limit.i_max, UserSetting::angle_roll_limit.d_max);
	manager.angle_roll.setTime(UserSetting::angle_roll_dt.dt);

	// Rate PIDs
	manager.rate_pitch.setGain(UserSetting::rate_pitch_gain.kp, UserSetting::rate_pitch_gain.ki, UserSetting::rate_pitch_gain.kd);
	manager.rate_pitch.setLimit(UserSetting::rate_pitch_limit.i_max, UserSetting::rate_pitch_limit.d_max);
	manager.rate_pitch.setTime(UserSetting::rate_pitch_dt.dt);

	manager.rate_roll.setGain(UserSetting::rate_roll_gain.kp, UserSetting::rate_roll_gain.ki, UserSetting::rate_roll_gain.kd);
	manager.rate_roll.setLimit(UserSetting::rate_roll_limit.i_max, UserSetting::rate_roll_limit.d_max);
	manager.rate_roll.setTime(UserSetting::rate_roll_dt.dt);

	manager.rate_yaw.setGain(UserSetting::rate_yaw_gain.kp, UserSetting::rate_yaw_gain.ki, UserSetting::rate_yaw_gain.kd);
	manager.rate_yaw.setLimit(UserSetting::rate_yaw_limit.i_max, UserSetting::rate_yaw_limit.d_max);
	manager.rate_yaw.setTime(UserSetting::rate_yaw_dt.dt);

	// Reset internals once after restoring gains
	manager.angle_pitch.reset();
	manager.angle_roll.reset();
	manager.rate_pitch.reset();
	manager.rate_roll.reset();
	manager.rate_yaw.reset();
}
