/*
 * PWM.hpp
 *
 *  Created on: May 4, 2025
 *      Author: aoi25
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include <cstdint>
#include <array>
#include "tim.h"
#include "FlightData/SbusData.hpp"
#include "UserSetting/MotorSetting.hpp"

// Pwm値の計算
void PwmCalcMainMotor(float throttle, std::array<float,3>& control, std::array<uint16_t,4>& motor);
void PwmCalcSubMotor(float throttle, std::array<uint16_t,4>& motor);
void PwmCalcServo(SbusChannelData sbus_data, uint16_t adc_value, std::array<uint16_t,2>& servo);

// 初期化
void PwmInitMotor();
void PwmInitServo();

// ESC calibration routine: runs a simple max->min pulse sequence for all motors.
// Call only when motors are connected and safe to spin. Minimal implementation.
void PwmCalibrateESC();

// Pwmの出力
void PwmGenerate(std::array<uint16_t,4>& upper_motor, std::array<uint16_t,4>& lower_motor, std::array<uint16_t,2>& servo);
void PwmGenerateUpperMotor(std::array<uint16_t,4>& upper_motor);
void PwmGenerateLowerMotor(std::array<uint16_t,4>& lower_motor);
void PwmGenerateServo(std::array<uint16_t,2>& servo);

// モーターの停止
void PwmStop();

#endif /* INC_PWM_HPP_ */