#include "PWM.hpp"
#include "math.h"

MotorTim motor_tim;
MotorChannel motor_channel;
MotorPWM motor_pwm;

ServoTim servo_tim;
ServoChannel servo_channel;
ServoPWM servo_pwm;

//PIDの制御量をモータに分配
void PidtoPwm(float throttle, float control[3], uint16_t motor[4]){

	motor[0] = motor_pwm.min + throttle + control[0] - control[1] - control[2];
	motor[1] = motor_pwm.min + throttle + control[0] + control[1] + control[2];
	motor[2] = motor_pwm.min + throttle - control[0] - control[1] + control[2];
	motor[3] = motor_pwm.min + throttle - control[0] + control[1] - control[2];

	uint32_t pwm_min = motor_pwm.min;
	uint32_t pwm_max = motor_pwm.max;
	uint32_t pwm_max2 = motor_pwm.max * motor_pwm.max;
	uint32_t pwm_min2 = motor_pwm.min * motor_pwm.min;
	uint32_t pwm_range = pwm_max - pwm_min;
	uint32_t pwm_range2 = pwm_max2 - pwm_min2;

	for(uint8_t i=0; i<4; i++){

		//線形的になるように処理
		if(motor[i] >= motor_pwm.max){

			motor[i] = motor_pwm.max;
		}
		if(motor[i] <= motor_pwm.min){

			motor[i] = motor_pwm.min;
		}

		motor[i] = sqrt(pwm_min2 + pwm_range2 * (motor[i] - pwm_min) / pwm_range);
	}
}

//ESCの初期化PWMを
void MotorInit(){

	//motor start
	HAL_TIM_PWM_Start(motor_tim.motor1, motor_channel.motor1);
	HAL_TIM_PWM_Start(motor_tim.motor2, motor_channel.motor2);
	HAL_TIM_PWM_Start(motor_tim.motor3, motor_channel.motor3);
	HAL_TIM_PWM_Start(motor_tim.motor4, motor_channel.motor4);

	//servo start
	HAL_TIM_PWM_Start(servo_tim.servo1, servo_channel.servo1);
	HAL_TIM_PWM_Start(servo_tim.servo2, servo_channel.servo2);

	//motor init
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor_pwm.init);

	//servo init
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo_pwm.close);
	__HAL_TIM_SET_COMPARE(servo_tim.servo2 , servo_channel.servo2, servo_pwm.close);

	//初期化待機
	HAL_Delay(2500);
}

void MotorIdel(){

	//motor idel
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor_pwm.min);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor2, motor_pwm.min);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor3, motor_pwm.min);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor4, motor_pwm.min);

	//servo idel
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo_pwm.close);
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo2, servo_pwm.close);
}

void MotorGenerate(uint16_t* motor, uint16_t* servo){

	//motor
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor[0]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor2, motor[1]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor3, motor[2]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor4, motor[3]);

	//servo
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo[0]);
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo2, servo[1]);
}

void MotorStop(){

	//motor
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor1);
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor2);
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor3);
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor4);

	//servo
	HAL_TIM_PWM_Stop(servo_tim.servo1, servo_channel.servo1);
	HAL_TIM_PWM_Stop(servo_tim.servo2, servo_channel.servo2);
}
