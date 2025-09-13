/*
 * PWM_Quad.cpp
 */
#include "Utils/MotorUtility.hpp"

uint8_t PWM_Quad::CheckMotorSetting(uint8_t motor_num){

    if(motor_num != 4){

        return 1; // error
    }

    return 0; // ok
}

// Motor初期化
void PWM_Quad::InitMotor(){

    HAL_TIM_PWM_Start(motor_tim.motor1, motor_channel.motor1);
    HAL_TIM_PWM_Start(motor_tim.motor2, motor_channel.motor2);
    HAL_TIM_PWM_Start(motor_tim.motor3, motor_channel.motor3);
    HAL_TIM_PWM_Start(motor_tim.motor4, motor_channel.motor4);

    __HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor_pwm.init);

    HAL_Delay(2500);
}

// Motor計算 (PID出力分配)
void PWM_Quad::CalcMotor(float throttle, std::array<float,4>& control, uint16_t* motor){

    motor[0] = motor_pwm.min + (throttle + control[0] - control[1] - control[2]);
    motor[1] = motor_pwm.min + (throttle + control[0] + control[1] + control[2]);
    motor[2] = motor_pwm.min + (throttle - control[0] - control[1] + control[2]);
    motor[3] = motor_pwm.min + (throttle - control[0] + control[1] - control[2]);

    for(uint8_t i=0; i<4; i++){

        if(motor[i] >= motor_pwm.max){

            motor[i] = motor_pwm.max;
        }

        if(motor[i] <= motor_pwm.min){

            motor[i] = motor_pwm.min;
        }
    }
}

// Motor出力
void PWM_Quad::GenerateMotor(uint16_t* motor){

    __HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor[0]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor[1]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor[2]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor[3]);
}

// Motor停止
void PWM_Quad::MotorStop(){

    HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor1);
    HAL_TIM_PWM_Stop(motor_tim.motor2, motor_channel.motor2);
    HAL_TIM_PWM_Stop(motor_tim.motor3, motor_channel.motor3);
    HAL_TIM_PWM_Stop(motor_tim.motor4, motor_channel.motor4);
}

// Motor設定
void PWM_Quad::SetMotorConfig(const MotorTim& tim, const MotorChannel& channel, const MotorPWM& pwm){

    motor_tim = tim;
    motor_channel = channel;
    motor_pwm = pwm;
}
