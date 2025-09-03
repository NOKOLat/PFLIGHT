#include "State/Headers/FlightStates.h"

void EmergencyControlState::update(FlightManager& manager) {

    // ループカウント
	static uint32_t loop_count = 0;
	loop_count++;

    // Armのチェック
	if(!manager.sbus_data.arm){

		manager.changeState(std::make_unique<DisarmingState>());
	}

    // 通常飛行への復帰
    if(!manager.sbus_data.emergency_control){

		manager.changeState(std::make_unique<FlyingState>());
	}

    // センサーデータの取得
	if (manager.imuUtil){

		manager.imuUtil->getData(manager.sensor_data.accel, manager.sensor_data.gyro);
	}

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
		manager.angle_pitch.calc(manager.sbus_data.target_value[0], manager.sensor_data.angle[0]);
		manager.angle_pitch.getData(&manager.control_data.target_rate[0]);
		manager.angle_roll.calc(manager.sbus_data.target_value[1], manager.sensor_data.angle[1]);
		manager.angle_roll.getData(&manager.control_data.target_rate[1]);

    	// yaw軸はセンサーデータを使用
		manager.control_data.target_rate[2] = manager.sbus_data.target_value[2];
    }

    // 400hz 角速度制御
	//目標角速度と現在角速度(センサーデータ）から制御量を計算
	manager.rate_pitch.calc(manager.control_data.target_rate[0], manager.sensor_data.gyro[0]);
	manager.rate_pitch.getData(&manager.control_data.pid_result[0]);

	manager.rate_roll.calc(manager.control_data.target_rate[1], manager.sensor_data.gyro[1]);
	manager.rate_roll.getData(&manager.control_data.pid_result[1]);

	manager.rate_yaw.calc(manager.control_data.target_rate[2], manager.sensor_data.gyro[2]);
	manager.rate_yaw.getData(&manager.control_data.pid_result[2]);

	// 上モーターを止める場合
	if(manager.sbus_data.stop_motor_side == 2 && manager.sbus_data.emergency_control){

		// メインモーターの計算式で、下モーターのスロットル + 制御の計算を行う
		PwmCalcMainMotor(manager.sbus_data.throttle, manager.control_data.pid_result, manager.control_data.lower_motor_pwm);

		// 計算値を2倍にする
		for(uint8_t i=0; i<4; i++){

			manager.control_data.lower_motor_pwm[i] *= 2;

			// todo: 最大値を超えていないかの判定と処理
		}

		// 上モーターの出力を0にする
		for(uint8_t i=0; i<4; i++){

			manager.control_data.upper_motor_pwm[i] = 0;
		}
	}
	//　下モーターを止める場合
	else if(manager.sbus_data.stop_motor_side == 1 && manager.sbus_data.emergency_control){

		// スロットルの値を少し上げる（定数倍）
		manager.sbus_data.throttle *= 1.1f;

		// メインモーターの計算式で、上モーターの計算を行う
		PwmCalcMainMotor(manager.sbus_data.throttle, manager.control_data.pid_result, manager.control_data.upper_motor_pwm);

		// 下モーターの出力を0にする
		for(uint8_t i=0; i<4; i++){

			manager.control_data.lower_motor_pwm[i] = 0;
		}
	}
	// どれも満たさない場合?
	else{

		// 通常状態に戻る
		manager.changeState(std::make_unique<FlyingState>());
		return;
	}

	// Pwmの出力をする
	PwmGenerate(manager.control_data.upper_motor_pwm, manager.control_data.lower_motor_pwm, manager.control_data.servo_pwm);
}
