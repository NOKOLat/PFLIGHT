/*
 * MotorUtility.hpp
 *
 * モーター制御ユーティリティ
 * PWMクラスを継承して、モーターの数や配置に対応した派生クラスを定義している
 * 
 * 各モータークラスの実装はヘッダーファイルを分けてあるので、追加する場合は基底クラスを継承して実装すること
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
        uint8_t CheckMotorSetting(uint8_t motor_num) override;
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

        // Motor関連実装
        uint8_t CheckMotorSetting(uint8_t motor_num) override;
        void InitMotor() override;
        void CalcMotor(float throttle, std::array<float,4>& control, uint16_t* motor_pwm) override;
        void GenerateMotor(uint16_t* motor_pwm) override;
        void MotorStop() override;
        void SetMotorConfig(const MotorTim& tim, const MotorChannel& channel, const MotorPWM& pwm) override;

        // 耐故障制御用
        void CalcMotorUpperOnly(float throttle, std::array<float,4>& control, uint16_t* motor);
        void CalcMotorLowerOnly(float throttle, std::array<float,4>& control, uint16_t* motor);

    private:
        MotorTim motor_tim{};
        MotorChannel motor_channel{};
        MotorPWM motor_pwm{};
};



#endif /* INC_UTILS_MOTORUTILITY_HPP_ */
