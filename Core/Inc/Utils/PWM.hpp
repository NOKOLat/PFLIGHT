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

// PWM制御クラス（元のグローバル関数をメンバー関数化）
class PWM {

public:

    PWM() = default;
    ~PWM() = default;

    // Servo
    void InitServo();
    void CalcServo(SbusChannelData sbus_data, uint16_t adc_value, std::array<uint16_t,2>& servo);
    void GenerateServo(std::array<uint16_t,2>& servo);

    // Motor
    void InitMotor();
    void CalcMotor(float throttle, std::array<float,3>& control, std::array<uint16_t,4>& motor);
    void GenerateMotor(std::array<uint16_t,4>& motor);
    void MotorStop();

    // Common
    void Generate(std::array<uint16_t,4>& motor, std::array<uint16_t,2>& servo);

    // Config
    void SetMotorConfig(const MotorTim& tim, const MotorChannel& channel, const MotorPWM& pwm);
    void SetServoConfig(const ServoTim& tim, const ServoChannel& channel, const ServoPWM& pwm);

private:

    MotorTim motor_tim;
    MotorChannel motor_channel;
    MotorPWM motor_pwm;
    ServoTim servo_tim;
    ServoChannel servo_channel;
    ServoPWM servo_pwm;
};


#endif /* INC_PWM_HPP_ */
