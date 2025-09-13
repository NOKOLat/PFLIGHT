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

		manager.imuUtil->GetData(manager.sensor_data.accel, manager.sensor_data.gyro);
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

	// サーボがcloseの時→下モーターを止める
	if(manager.sbus_data.drop == 0){

		// 上モーターのみを計算
		manager.pwm.CalcMotorUpperOnly(manager.sbus_data.throttle, manager.control_data.pid_result, manager.control_data.motor_pwm.data());

	}
	else{

		// 下モータのみを計算
		manager.pwm.CalcMotorLowerOnly(manager.sbus_data.throttle, manager.control_data.pid_result, manager.control_data.motor_pwm.data());
	}

	//  サーボのpwmは計算しない

	// adcは計算しない

	// PWMを生成
	manager.pwm.GenerateMotor(manager.control_data.motor_pwm.data());
	manager.pwm.GenerateServo(manager.control_data.servo_pwm);

	printf("Motor[8]: %4u, %4u, %4u, %4u %4u, %4u, %4u, %4u \n", manager.control_data.motor_pwm[0], manager.control_data.motor_pwm[1], manager.control_data.motor_pwm[2], manager.control_data.motor_pwm[3], manager.control_data.motor_pwm[4], manager.control_data.motor_pwm[5], manager.control_data.motor_pwm[6], manager.control_data.motor_pwm[7]);
}
