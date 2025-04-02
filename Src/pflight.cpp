/*
 * pflight.cpp
 *
 *  Created on: Mar 31, 2025
 *      Author: Sezakiaoi
 */

#include <pflight.hpp>
#include "ICM42688P_HAL_SPI.h"
#include "pose_estimation.h"
#include "SBUS.h"
#include "ring_buffer.h"
#include "PID.h"

ICM42688P_HAL_SPI icm(&hspi1, GPIOB, GPIO_PIN_6);
SBUS sbus;
PID pitch, roll, yaw;

//----------構造体----------//
struct ArmLoop{

	uint8_t wait 	   	   = 0;
	uint8_t loop_count 	   = 0;
	uint8_t data_index 	   = 0;
	uint8_t max_index 	   = 8;
	uint8_t failsafe_count = 0;
}armloop;

struct Data{

	uint16_t sbus[10] = {};
	RINGBUFFER accel;
	RINGBUFFER gyro;
	RINGBUFFER mag;
	float angle[3] = {};
	float speed[3] = {};
	float position[3] = {};
}data;

struct Channel{

	const uint8_t pitch    = 0;
	const uint8_t roll     = 1;
	const uint8_t yaw      = 3;
	const uint8_t throttle = 2;

	const uint8_t arm = 5;
	const uint8_t motor = 6;
}channel;

struct PWM{

	uint16_t motor[8] = {};
	uint16_t servo[2] = {};

	uint16_t motor_max  = 2500 * 0.55;
	uint16_t motor_min  = 2500 * 0.45;
	uint16_t motor_init = 2500 * 0.40;

	uint16_t servo_open = 1800;
	uint16_t servo_close = 400;
}pwm;

struct Control{

	float target[3] = {};//pitch,rollは角度 yawは角速度
	float throttle = 0;
	float pid_output[3] = {};//PIDの出力
}control;

//---------------Arm---------------//

/* @brief Armの判定
 *
 * @return 0:arm 1:disarm
 */
uint8_t is_arm(){

	return data.sbus[channel.arm] > 1000;
}

/* @brief 自動操縦の判定
 *
 * @return 0:有効 1:無効
 */
uint8_t is_auto(){

	return 1;
}

/* @brief モーター始動
 *
 * @return 0:有効 1:無効
 */
uint8_t is_rotate(){

	return data.sbus[channel.motor] > 1000;
}
//---------------メインループ管理---------------//

/* @brief ループ開始処理
 *
 * ループ待機フラグの有効化
 * ループカウントのインクリメント
 * データ取得回数のリセット
 */
void init_armloop(){

	armloop.wait 	   = 0;
	armloop.loop_count ++;
}

void set_wait_flag(){

	armloop.wait 	   = 1;
}
/* @brief アームループ待機 */
uint8_t is_wait_armloop(){

	return armloop.wait;
}

//---------------タイマー割り込み管理---------------//

/* @brief タイマー割り込み開始
 *
 * 2ループ分空の実行を行います(100ms)
 *
 */
void init_timer_interrupt(){

	//ArmLoop
	HAL_TIM_Base_Start_IT(&htim6);

	//IMU
	HAL_Delay(3);
	HAL_TIM_Base_Start_IT(&htim7);

	//failsafe
	HAL_Delay(7);
	HAL_TIM_Base_Start_IT(&htim5);

	//初期データの取得
	HAL_Delay(50);
}

/* @brief タイマー割り込み停止
 *
 * ArmLoop(Tim6)
 * IMU(Tim7)
 * を停止します
 */
void deinit_timer_interrupt(){

	//ArmLoop
	HAL_TIM_Base_Stop_IT(&htim6);

	//IMU
	HAL_TIM_Base_Stop_IT(&htim7);
}

//---------------SBUS管理とフェイルセーフ---------------//

/* @brief SBUS用の割り込み開始
 *
 * 初回実行と受信再開時の両方で使います
 */
void init_sbus(){

	HAL_UART_Receive_DMA(&huart2, sbus.GetBufferPointer(), sbus.DataLen);
}

void deinit_sbus(){

	HAL_UART_DMAStop(&huart2);
}

void encode_sbus(){

	if(sbus.IsSBUS() == 0){

		sbus.Encode();
		sbus.GetData(data.sbus);
		armloop.failsafe_count = 0;
	}
}

//SBUS 1000 ~ 2000
void sbus_to_angle(){

	//pitch(angle)
	control.target[0] = float(data.sbus[channel.pitch] - 1500) * 45 / 500;

	//roll(angle)
	control.target[1] = float(data.sbus[channel.pitch] - 1500) * 45 / 500;

	//yaw(speed)
	control.target[2] = float(data.sbus[channel.pitch] - 1500) * 25 / 500;

	//throttle(500~2000)
	control.throttle = float(data.sbus[channel.throttle] - 500) / 1500;

}
/* @brief フェイルセーフのチェック
 *
 * 10カウント(1000msの通信停止）でフェイルセーフします
 */
uint8_t check_failsafe(){

	if(armloop.failsafe_count > 10){

		return 1;
	}

	armloop.failsafe_count ++;
	return 0;
}

//---------------IMU---------------//

/* @brief imuの初期化
 *
 * 通信チェックとキャリブレーション
 *
 * @return 0: 初期化成功 1: 通信失敗
 */
uint8_t init_imu(){

	uint8_t debug = icm.Connection();
	icm.AccelConfig(icm.ACCEL_Mode::LowNoize, icm.ACCEL_SCALE::SCALE02g, icm.ACCEL_ODR::ODR01000hz, icm.ACCEL_DLPF::ODR40);
	icm.GyroConfig(icm.GYRO_MODE::LowNoize, icm.GYRO_SCALE::Dps0250, icm.GYRO_ODR::ODR01000hz, icm.GYRO_DLPF::ODR40);
	HAL_Delay(10);
	icm.Calibration(2000);

	return debug;
}

/* @brief 6軸のセンサーデータ取得
 *
 * 16要素のBufferに対して、以下の書き込みを行っています
 * 偶数ループ時: 0~7に書き込み
 * 奇数ループ時: 8~15に書き込み
 *
 * データ取得ごとにデータ数をカウントしています
 */
void get_imu_data(){

	float tmp_accel[3] = {};
	float tmp_gyro[3]  = {};

	icm.GetData(tmp_accel, tmp_gyro);

	data.accel.set_value(tmp_accel);
	data.gyro.set_value(tmp_gyro);
}

//---------------姿勢推定---------------//

/* @brief 姿勢推定にデータを入力
 *
 * 偶数ループ時: 8~15のデータを書き込み
 * 奇数ループ時: 0~7のデータを書き込み
 */
void calc_pose(){

	uint8_t index_accel = data.accel.get_index();
	uint8_t index_gyro = data.gyro.get_index();
	uint8_t index_mag = data.mag.get_index();

	while(PoseEstimation(data.accel.get_value(index_accel % 16), data.gyro.get_value(index_gyro % 16), data.mag.get_value(index_mag % 16), data.angle, data.speed, data.position) == 1){

		index_accel --;
		index_gyro --;
		index_mag --;
	}
}

void clear_value(){

}

//---------------PID---------------//


void pid_setup(){

	pitch.Setup(0.1, 0, 0, 0);
	roll.Setup(0.1, 0, 0, 0);
	yaw.Setup(0.1, 0, 0, 0);
}

void pid_calc(){

	control.pid_output[0] = pitch.Calc(control.target[0], data.angle[0]);
	control.pid_output[1] =	roll.Calc(control.target[1], data.angle[1]);
	control.pid_output[2] =	yaw.Calc(control.target[2], data.angle[2]);
}

void pid_to_pwm(){

	pwm.motor[0] = control.throttle + control.pid_output[0] + control.pid_output[1] + control.pid_output[2];
	pwm.motor[1] = control.throttle + control.pid_output[0] + control.pid_output[1] + control.pid_output[2];
	pwm.motor[2] = control.throttle + control.pid_output[0] + control.pid_output[1] + control.pid_output[2];
	pwm.motor[3] = control.throttle + control.pid_output[0] + control.pid_output[1] + control.pid_output[2];
}
//---------------PWM---------------//

/* @brief ESCに最小回転数を送信
 *
 *
 */
void init_pwm(){

	//Motor_Start
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//Servo
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	//Motor_Init
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm.motor_init);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm.motor_init);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm.motor_init);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm.motor_init);

	//Delay
	HAL_Delay(4000);
}

void idel_pwm(){

	//Motor_idel
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm.motor_min);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm.motor_min);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm.motor_min);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm.motor_min);

	//Servo_idel
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm.servo_close);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm.servo_close);
}

void generate_pwm(){

	//Motor
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm.motor[0]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm.motor[1]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm.motor[2]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm.motor[3]);

	//Servo
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm.servo[0]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm.servo[1]);
}

//---------------Debug---------------//

void printvalue(){

	printf("%3.3lf, %3.3lf, %3.3lf\n", data.angle[0], data.angle[1], data.angle[2]);
}

void debug_mode(){

	data.sbus[5] = 1500;
	data.sbus[6] = 1500;
}
