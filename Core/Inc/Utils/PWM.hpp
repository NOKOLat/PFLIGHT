/*
 * PWM.hpp
 * 基底PWMクラス: Servo系のデフォルト実装を提供し、Motor系は純粋仮想
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include <cstdint>
#include <array>
#include "tim.h"
#include "FlightData/SbusData.hpp"
#include "UserSetting/MotorSetting.hpp"

// 基底PWMクラス
class PWM {
public:

    PWM() = default;
    virtual ~PWM() = default;

    virtual uint8_t CheckMotorSetting(uint8_t motor_num) = 0;

    // -------- Servo (仮想: 既定実装あり) --------
    virtual void InitServo();
    virtual void CalcServo(SbusChannelData sbus_data, uint16_t adc_value, std::array<uint16_t,2>& servo);
    virtual void GenerateServo(std::array<uint16_t,2>& servo);
    virtual void SetServoConfig(const ServoTim& tim, const ServoChannel& channel, const ServoPWM& pwm);

    // -------- Motor (純粋仮想) --------
    virtual void InitMotor() = 0;
    virtual void CalcMotor(float throttle, std::array<float,4>& control, uint16_t* motor_pwm) = 0;
    virtual void GenerateMotor(uint16_t* motor_pwm) = 0;
    virtual void MotorStop() = 0;
    virtual void SetMotorConfig(const MotorTim& tim, const MotorChannel& channel, const MotorPWM& pwm) = 0;

protected:
    // Servo設定のみ基底が保持
    ServoTim servo_tim{};
    ServoChannel servo_channel{};
    ServoPWM servo_pwm{};
};

#endif /* INC_PWM_HPP_ */
