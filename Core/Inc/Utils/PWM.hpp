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
    void initServo();
    void calcServo(SbusChannelData sbus_data, uint16_t adc_value, std::array<uint16_t,2>& servo);
    void generateServo(std::array<uint16_t,2>& servo);

    // Motor
    void initMotor();
    void calcMotor(float throttle, std::array<float,3>& control, std::array<uint16_t,4>& motor);
    void generateMotor(std::array<uint16_t,4>& motor);
    void motorStop();

    // Both
    void generate(std::array<uint16_t,4>& motor, std::array<uint16_t,2>& servo);
    void generate(std::array<uint16_t,4>& /*upper_motor*/, std::array<uint16_t,4>& /*lower_motor*/, std::array<uint16_t,2>& /*servo*/); // 予備（未使用）

    // 旧PwmStop互換 呼び出し側は stop() を使用
    void stop();

private:

    MotorTim motor_tim;
    MotorChannel motor_channel;
    MotorPWM motor_pwm;
    ServoTim servo_tim;
    ServoChannel servo_channel;
    ServoPWM servo_pwm;
};


#endif /* INC_PWM_HPP_ */
