/*
 * flight_manager.hpp
 *
 *  Created on: Jun 30, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_FLIGHT_MANAGER_HPP_
#define INC_FLIGHT_MANAGER_HPP_

#include "flight_data.hpp"
#include <cstdint>

enum class state: uint8_t{

	init = 0,
	wait_arm,
	arm,
	wait_fly,
	fly,
	disarm,
	automation,
	failsafe,
	twin,
};

enum class error_state: uint8_t{

	NO_ERROR = 0,
	SBUS_NOT_DETECTED,
	IMU_NOT_DETECTED,
	IMU_CALIBRATION_FALUT,
	ARM_NOT_DETECTED,
	FLY_NOT_DETECTED,
	DELAY_NOW,
};

//各状態ごとの処理の戻り値
struct StateResult{

	bool state_changed = false;
	state next_state = state::init;
	error_state error = error_state::NO_ERROR;
};

class FlightManager{

	public:

	void UpDate();
	void SbusUpDate(uint16_t sbus_data[10], bool failsafe_bit);
	state GetCurrentState();

	private:

	//現在の状態
	state current_state = state::init;

	//状態ごとの処理
	StateResult Init();
	StateResult WaitArm();

	StateResult Arm();
	StateResult WaitFly();
	StateResult Fly();
	StateResult DisArm();
	StateResult Automation();
	StateResult FailSafe();
	StateResult Twin();

	//データ構造体のインスタンス
	SensorData sensor_data;
	SbusData sbus_data;
	ControlData control_data;
};



#endif /* INC_FLIGHT_MANAGER_HPP_ */
