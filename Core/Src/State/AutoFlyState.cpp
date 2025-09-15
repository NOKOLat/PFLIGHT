#include "State/Headers/FlightStates.h"

bool entered = false;

float target_value[3] = {}; // pitch, roll, yaw
float target_altitude = 0.0f;
float estimated_altitude = 0.0f;
float estimated_velocity = 0.0f;
static float throttle = 0.0f;

static float sum_accel[3] = {};
float average_accel[3] = {};

static float sum_angle[3] = {};
float average_angle[3] = {};

static float sum_altitude = 0.0f;
static float sum_velocity = 0.0f;

// ループカウント
static uint32_t loop_count = 0;

// プロトタイプ宣言: グローバル関数は FlightManager を引数に受け取る
static void AltitudeControl(FlightManager& manager);
static void AngularControl(FlightManager& manager);
static void AngularVelocityControl(FlightManager& manager);

void AutoFlyState::update(FlightManager& manager) {

	loop_count++;

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

	if (!entered){

		altitude.offset();
		
		if(loop_count > 400){
			entered = true;
		}
	}

	// 自動操縦用目標値: AutopilotData をスケールして使用
	// controller から送られる値にはトリムが含まれているため、sbus_data.trim を加算して補正する
	// pitch, roll: 角度 (deg)、 yaw: 角速度 (dps)
	target_value[0] = 5;
	target_value[1] = (manager.autopilot_data.roll / 127.0f) * 30;
	target_value[2] = 0;

	// trim を反映（trim は正規化値なので各軸の最大値でスケール）
	target_value[0] += manager.sbus_data.trim[0] * manager.sbus_data.angle_pitch_max;
	target_value[1] += manager.sbus_data.trim[1] * manager.sbus_data.angle_roll_max;
	target_value[2] += manager.sbus_data.trim[2] * manager.sbus_data.rate_yaw_max;

	// throttle_assist used as altitude target proxy (cm)
	target_altitude = 1.00f;

	//IMUデータの取得
	manager.imuUtil->GetData(manager.sensor_data.accel, manager.sensor_data.gyro);

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

	//100Hz 
	if (loop_count %4 == 0){
		
		AngularControl(manager);

		for (uint8_t i=0; i<3; i++){
			average_accel[i] = sum_accel[i] / 4.0f;
			average_angle[i] = sum_angle[i] / 4.0f;

			sum_accel[i] = 0.0f;
			sum_angle[i] = 0.0f;
			
		}

		// --- 気圧の取得と高度推定 ---
		float pressure_Pa = 0.0f;
		float temperature_C = 0.0f;
		// 可能ならバーオメータから取得（FlightManager に保持される dps368 を使用）
		if (manager.dps368->getData(&pressure_Pa, &temperature_C) == 0) {
			manager.sensor_data.pressure = pressure_Pa;
		}

		//printf("%+2.2f, %+2.2f, %.4f\n", average_accel[2], average_angle[0], average_pressure);
	

		altitude.Update(manager.sensor_data.pressure, average_accel, average_angle, 0.01f);

		float getdata[3];
		altitude.GetData(getdata);
		sum_altitude += getdata[0];
		sum_velocity += getdata[1];
	}

	//12.5hz
	if(loop_count %32 == 0){

		estimated_altitude = sum_altitude / 8.0f;
		estimated_velocity = sum_velocity / 8.0f;

		sum_altitude = 0.0f;
		sum_velocity = 0.0f;

		if (entered){
			AltitudeControl(manager);
		}
	}

	//400Hz
	AngularVelocityControl(manager);

	if (entered){
		manager.pwm.CalcMotor(throttle, manager.control_data.pid_result, manager.control_data.motor_pwm.data());
		manager.pwm.GenerateMotor(manager.control_data.motor_pwm.data());
	}
}


// 12.5hz 高度制御
void AltitudeControl(FlightManager& manager){

	static float prev_altitude = 0.0f;
	const float dt_alt = 0.08f;
	const float mixing = 0.5f;

	// PDゲイン: velocity_errorとその時間微分を throttle 単位にマッピング
	const float Kp_vel = 10.0f; // throttle per 
	const float Kd_vel = 10.0f; // throttle per 

	//printf("%.3f m ", estimated_altitude);

	float velocity = ((estimated_altitude - prev_altitude) / dt_alt ) * mixing 
					+ estimated_velocity * (1-mixing);
	prev_altitude = estimated_altitude;

	// 目標速度
	float target_velocity = target_altitude - estimated_altitude;
	//printf("v:%+.2fm/s tgt:%+.2fm/s ", velocity, target_velocity);


	// 速度誤差に基づく制御
	float velocity_error = (target_velocity - velocity);

	// 静的変数: 前回速度誤差を保持
	static float prev_velocity_error = 0.0f;

	// 微分項の計算 (dt = dt_alt)
	float derivative = (velocity_error - prev_velocity_error) / dt_alt;
	prev_velocity_error = velocity_error;

	// P/D を throttle 単位に変換
	float p_contrib = Kp_vel * velocity_error; // throttle 単位
	float d_term = Kd_vel * derivative; // throttle 単位

	if(target_altitude < 0.0f){
		if (estimated_altitude < 0.20f){
			throttle = 0.0f;
		}
		else {
			p_contrib *= 0.1;
			if (p_contrib < -5.0f){
				p_contrib = -5.0f;
			}

			throttle += p_contrib + d_term;
		}
			
	} else {
		// P項のクリップは throttle 単位で評価
		if (p_contrib > 10.0f){
			p_contrib = 10.0f;
		}
		if (p_contrib < -10.0f){
			p_contrib = -10.0f;
		}

		if(p_contrib < 0.0f && throttle <= 50.0f){
			throttle = 50.0f;
			p_contrib = 0.0f;
			d_term = 0.0f;
		}

		if (d_term > 20.0f){
			d_term = 20.0f;
		}
		if (d_term < -20.0f){
			d_term = -20.0f;
		}


		throttle += p_contrib + d_term;

		if(throttle >600.0f){
			throttle = 600.0f;
		}
	}
	printf("%.1f\n", throttle);
	printf("Motor[8]: %4u, %4u, %4u, %4u %4u, %4u, %4u, %4u \n", manager.control_data.motor_pwm[0], manager.control_data.motor_pwm[1], manager.control_data.motor_pwm[2], manager.control_data.motor_pwm[3], manager.control_data.motor_pwm[4], manager.control_data.motor_pwm[5], manager.control_data.motor_pwm[6], manager.control_data.motor_pwm[7]);
	

}


// 100hz 角度制御(pitch, roll)
void AngularControl(FlightManager& manager){
	
	// 目標角と現在角から目標角速度を計算
	// pitch, rollの目標角速度計算: calc() を呼んだ後 getData() で結果を取得
	manager.angle_pitch.calc(target_value[0], manager.sensor_data.angle[0]);
	manager.angle_pitch.getData(&manager.control_data.target_rate[0]);
	manager.angle_roll.calc(target_value[1], manager.sensor_data.angle[1]);
	manager.angle_roll.getData(&manager.control_data.target_rate[1]);

	// yaw軸はセンサーデータを使用
	manager.control_data.target_rate[2] = target_value[2];

}

void AngularVelocityControl(FlightManager& manager){

	// 400hz 角速度制御
	//目標角速度と現在角速度(センサーデータ）から制御量を計算
	// pitch, roll, yawの制御量計算: calc() を呼んだ後 getData() で結果を取得
	manager.rate_pitch.calc(manager.control_data.target_rate[0], manager.sensor_data.gyro[0]);
	manager.rate_pitch.getData(&manager.control_data.pid_result[0]);
	manager.rate_roll.calc(manager.control_data.target_rate[1], manager.sensor_data.gyro[1]);
	manager.rate_roll.getData(&manager.control_data.pid_result[1]);
	manager.rate_yaw.calc(manager.control_data.target_rate[2], manager.sensor_data.gyro[2]);
	manager.rate_yaw.getData(&manager.control_data.pid_result[2]);

}


void AutoFlyState::enter(FlightManager& manager) {
	entered = false;
	loop_count = 0;
	
	// Overwrite PID parameters for autopilot mode using UserSetting values
	// Angle PIDs
	manager.angle_pitch.reset();
	manager.angle_roll.reset();
	
	manager.rate_pitch.reset();
	manager.rate_roll.reset();
	manager.rate_yaw.reset();

	//manager.angle_pitch.setGain(UserSetting::angle_pitch_gain.kp*1.5f, UserSetting::angle_pitch_gain.ki, UserSetting::angle_pitch_gain.kd);
	// manager.angle_pitch.setLimit(UserSetting::angle_pitch_limit.i_max, UserSetting::angle_pitch_limit.d_max);
	// manager.angle_pitch.setTime(UserSetting::angle_pitch_dt.dt);
	

	//manager.angle_roll.setGain(UserSetting::angle_roll_gain.kp*1.5f, UserSetting::angle_roll_gain.ki, UserSetting::angle_roll_gain.kd);
	// manager.angle_roll.setLimit(UserSetting::angle_roll_limit.i_max, UserSetting::angle_roll_limit.d_max);
	// manager.angle_roll.setTime(UserSetting::angle_roll_dt.dt);
	

	// // Rate PIDs
	// manager.rate_pitch.setGain(UserSetting::rate_pitch_gain.kp, UserSetting::rate_pitch_gain.ki, UserSetting::rate_pitch_gain.kd);
	// manager.rate_pitch.setLimit(UserSetting::rate_pitch_limit.i_max, UserSetting::rate_pitch_limit.d_max);
	// manager.rate_pitch.setTime(UserSetting::rate_pitch_dt.dt);
	

	// manager.rate_roll.setGain(UserSetting::rate_roll_gain.kp, UserSetting::rate_roll_gain.ki, UserSetting::rate_roll_gain.kd);
	// manager.rate_roll.setLimit(UserSetting::rate_roll_limit.i_max, UserSetting::rate_roll_limit.d_max);
	// manager.rate_roll.setTime(UserSetting::rate_roll_dt.dt);
	

	manager.rate_yaw.setGain(UserSetting::rate_yaw_gain.kp*1.5f, UserSetting::rate_yaw_gain.ki, UserSetting::rate_yaw_gain.kd);
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

	throttle = 0.0f;
	altitude.Reset();
	
}
