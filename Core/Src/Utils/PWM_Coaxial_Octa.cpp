/*
 * PWM_Coaxial_Octa.cpp
 * Motor系はユーザー実装予定の空定義
 */
#include "Utils/MotorUtility.hpp"

uint8_t PWM_Coaxial_Octa::CheckMotorSetting(uint8_t motor_num){

    if(motor_num != 8){

        return 1; // error
    }

    return 0; // ok
}

void PWM_Coaxial_Octa::InitMotor(){

    HAL_TIM_PWM_Start(motor_tim.motor1, motor_channel.motor1);
    HAL_TIM_PWM_Start(motor_tim.motor2, motor_channel.motor2);
    HAL_TIM_PWM_Start(motor_tim.motor3, motor_channel.motor3);
    HAL_TIM_PWM_Start(motor_tim.motor4, motor_channel.motor4);
    HAL_TIM_PWM_Start(motor_tim.motor5, motor_channel.motor5);
    HAL_TIM_PWM_Start(motor_tim.motor6, motor_channel.motor6);
    HAL_TIM_PWM_Start(motor_tim.motor7, motor_channel.motor7);
    HAL_TIM_PWM_Start(motor_tim.motor8, motor_channel.motor8);

    __HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor5 , motor_channel.motor5, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor6 , motor_channel.motor6, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor7 , motor_channel.motor7, motor_pwm.init);
    __HAL_TIM_SET_COMPARE(motor_tim.motor8 , motor_channel.motor8, motor_pwm.init);

    HAL_Delay(2500);

}

void PWM_Coaxial_Octa::CalcMotor(float throttle, std::array<float,4>& control, uint16_t* motor){

    // motor[0] = motor_pwm.min + (throttle + control[0] - control[1] - control[2]);
    // motor[1] = motor_pwm.min + (throttle + control[0] + control[1] + control[2]);
    // motor[2] = motor_pwm.min + (throttle - control[0] - control[1] + control[2]);
    // motor[3] = motor_pwm.min + (throttle - control[0] + control[1] - control[2]);
    motor[0] = 1100;
    motor[1] = 1100;
    motor[2] = 1100;
    motor[3] = 1100;
    
    motor[4] = motor_pwm.min + (throttle + control[0] - control[1] + control[2]);
    motor[5] = motor_pwm.min + (throttle + control[0] + control[1] - control[2]);
    motor[6] = motor_pwm.min + (throttle - control[0] - control[1] - control[2]);
    motor[7] = motor_pwm.min + (throttle - control[0] + control[1] + control[2]);
    

    for(uint8_t i=0; i<4; i++){

        if(motor[i] >= motor_pwm.max){

            motor[i] = motor_pwm.max;
        }

        if(motor[i] <= 1100){

			motor[i] = 1100;
		}
        
    }
    for(uint8_t i=4; i<8; i++){

		if(motor[i] >= motor_pwm.max){

			motor[i] = motor_pwm.max;
		}

        if(motor[i] <= motor_pwm.min){

            motor[i] = motor_pwm.min;
        }
		
	}

}

void PWM_Coaxial_Octa::CalcMotorUpperOnly(float throttle, std::array<float,4>& control, uint16_t* motor){

    motor[0] = motor_pwm.min + (throttle + control[0] - control[1] - control[2]);
    motor[1] = motor_pwm.min + (throttle + control[0] + control[1] + control[2]);
    motor[2] = motor_pwm.min + (throttle - control[0] - control[1] + control[2]);
    motor[3] = motor_pwm.min + (throttle - control[0] + control[1] - control[2]);
    motor[4] = motor_pwm.init;
    motor[5] = motor_pwm.init;
    motor[6] = motor_pwm.init;
    motor[7] = motor_pwm.init;

    for(uint8_t i=0; i<4; i++){

        if(motor[i] >= motor_pwm.max){

            motor[i] = motor_pwm.max;
        }

        if(motor[i] <= motor_pwm.min){

            motor[i] = motor_pwm.min;
        }
    }
}

void PWM_Coaxial_Octa::CalcMotorLowerOnly(float throttle, std::array<float,4>& control, uint16_t* motor){

    motor[0] = motor_pwm.init;
    motor[1] = motor_pwm.init;
    motor[2] = motor_pwm.init;
    motor[3] = motor_pwm.init;
    motor[4] = motor_pwm.min + (throttle + control[0] - control[1] + control[2]);
    motor[5] = motor_pwm.min + (throttle + control[0] + control[1] - control[2]);
    motor[6] = motor_pwm.min + (throttle - control[0] - control[1] - control[2]);
    motor[7] = motor_pwm.min + (throttle - control[0] + control[1] + control[2]);

    for(uint8_t i=5; i<8; i++){

        if(motor[i] >= motor_pwm.max){

            motor[i] = motor_pwm.max;
        }

        if(motor[i] <= motor_pwm.min){

            motor[i] = motor_pwm.min;
        }
    }
}

void PWM_Coaxial_Octa::GenerateMotor(uint16_t* motor){

    __HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor[0]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor[1]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor[2]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor[3]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor5 , motor_channel.motor5, motor[4]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor6 , motor_channel.motor6, motor[5]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor7 , motor_channel.motor7, motor[6]);
    __HAL_TIM_SET_COMPARE(motor_tim.motor8 , motor_channel.motor8, motor[7]);
}

void PWM_Coaxial_Octa::MotorStop(){

    HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor1);
    HAL_TIM_PWM_Stop(motor_tim.motor2, motor_channel.motor2);
    HAL_TIM_PWM_Stop(motor_tim.motor3, motor_channel.motor3);
    HAL_TIM_PWM_Stop(motor_tim.motor4, motor_channel.motor4);
    HAL_TIM_PWM_Stop(motor_tim.motor5, motor_channel.motor5);
    HAL_TIM_PWM_Stop(motor_tim.motor6, motor_channel.motor6);
    HAL_TIM_PWM_Stop(motor_tim.motor7, motor_channel.motor7);
    HAL_TIM_PWM_Stop(motor_tim.motor8, motor_channel.motor8);

}

void PWM_Coaxial_Octa::SetMotorConfig(const MotorTim& tim, const MotorChannel& channel, const MotorPWM& pwm){

    motor_tim = tim;
    motor_channel = channel;
    motor_pwm = pwm;
}