/*
 * flight_data.hpp
 *
 *  Created on: Jun 30, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_FLIGHT_DATA_HPP_
#define INC_FLIGHT_DATA_HPP_

#include <cstdint>

struct SensorData{

	float accel[3] = {};
	float gyro[3] = {};
	float angle[3] = {};
};

struct SbusData{

	uint16_t max[4]    = {1600, 1600, 1680, 1600};
	uint16_t min[4]    = {400, 400, 368, 400};
	uint16_t center[4] = {1000, 1000, 1000, 1000};

	//受信判定（起動時に使用）
	bool is_receive = false;

	//目標値データ
	int16_t throttle         = 0;
	float target_pitch_angle = 0;
	float target_roll_angle  = 0;
	float target_yaw_rate    = 0;

	//3軸をまとめた配列 PIDの計算時に使用
	float target_angle[3] = {};
	float target_rate[3] = {};

	//スイッチデータ
	bool arm        = false;
	bool fly        = false;
	uint8_t drop    = 0;
	bool autodrop   = false;
	bool autofly = false;

	//failsafe
	bool failsafe_bit = false;
	uint16_t failsafe_count = 0;
};

struct ControlData{

	float pid_control[3] = {};
	uint16_t motor_pwm[4] = {};
	uint16_t servo_pwm[2] = {};
};

enum class Channel: uint8_t{

	//channel = index - 1;
	throttle = 3 - 1,
	pitch = 2 - 1,
	roll = 1 - 1,
	yaw = 4 - 1,
	arm = 6 - 1,
	fly = 5 - 1,
	drop = 7 - 1,
	autodrop = 8 - 1,
	autofly = 9 - 1,
};
#endif /* INC_FLIGHT_DATA_HPP_ */
