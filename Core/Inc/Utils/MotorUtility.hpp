/*
 * MotorUtility.hpp
 *
 *  Created on: Sep 8, 2025
 *      Author: aoi25
 */

#ifndef INC_UTILS_MOTORUTILITY_HPP_
#define INC_UTILS_MOTORUTILITY_HPP_

#include "Utils/PWM.hpp"

class PWM_Quad : public PWM {

    public:

        PWM_Quad() = default;
        virtual ~PWM_Quad() = default;

        // Motor関連実装
        void InitMotor() override;
        void CalcMotor(float throttle, std::array<float,4>& control, uint16_t* motor_pwm) override;
        void GenerateMotor(uint16_t* motor_pwm) override;
        void MotorStop() override;
        void SetMotorConfig(const MotorTim& tim, const MotorChannel& channel, const MotorPWM& pwm) override;

    private:
        MotorTim motor_tim{};
        MotorChannel motor_channel{};
        MotorPWM motor_pwm{};
};

class PWM_Coaxial_Octa : public PWM {

    public:
        PWM_Coaxial_Octa() = default;
        virtual ~PWM_Coaxial_Octa() = default;

        void InitMotor() override;
        void CalcMotor(float throttle, std::array<float,4>& control, uint16_t* motor_pwm) override;
        void GenerateMotor(uint16_t* motor_pwm) override;
        void MotorStop() override;
        void SetMotorConfig(const MotorTim& tim, const MotorChannel& channel, const MotorPWM& pwm) override;

    private:
        MotorTim motor_tim{};
        MotorChannel motor_channel{};
        MotorPWM motor_pwm{};
};



#endif /* INC_UTILS_MOTORUTILITY_HPP_ */
