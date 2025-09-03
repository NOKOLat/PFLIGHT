#include "Utils/PWM.hpp"
#include <array>

MotorTim motor_tim;
MotorChannel motor_channel;
MotorPWM motor_pwm;

ServoTim servo_tim;
ServoChannel servo_channel;
ServoPWM servo_pwm;

//PIDの制御量をモータに分配
void PwmCalcMainMotor(float throttle, std::array<float,3>& control, std::array<uint16_t,4>& motor){

	//モーターの値を計算
	motor[0] = motor_pwm.min + (throttle + control[0] - control[1] - control[2]);
	motor[1] = motor_pwm.min + (throttle + control[0] + control[1] + control[2]);
	motor[2] = motor_pwm.min + (throttle - control[0] - control[1] + control[2]);
	motor[3] = motor_pwm.min + (throttle - control[0] + control[1] - control[2]);

	//　最大値と最小値を超えた場合の処理
	for(uint8_t i=0; i<4; i++){

		if(motor[i] >= motor_pwm.max){

			motor[i] = motor_pwm.max;
		}
		if(motor[i] <= motor_pwm.min){

			motor[i] = motor_pwm.min;
		}
	}
}

void PwmCalcSubMotor(float throttle, std::array<uint16_t,4>& motor){

	//モーターの値を計算
	motor[0] = motor_pwm.min + throttle / 10.0;
	motor[1] = motor_pwm.min + throttle / 10.0;
	motor[2] = motor_pwm.min + throttle / 10.0;
	motor[3] = motor_pwm.min + throttle / 10.0;

	//　最大値と最小値を超えた場合の処理
	for(uint8_t i=0; i<4; i++){

		if(motor[i] >= motor_pwm.max){

			motor[i] = motor_pwm.max;
		}
		if(motor[i] <= motor_pwm.min){

			motor[i] = motor_pwm.min;
		}
	}
}

void PwmCalcServo(SbusChannelData sbus_data, uint16_t adc_value, std::array<uint16_t, 2>& servo){

	//投下条件
	//autodrop == 0 かつ 赤外線が閾値以上
	//drop == 1

	for(uint8_t i=0; i<2; i++){

		if((sbus_data.autodrop && (adc_value > 2000)) || sbus_data.drop == 2){
		//if(sbus_data.drop == 2){

			servo[i] = servo_pwm.open;
		}
		else if(sbus_data.drop == 1){

			servo[i] = servo_pwm.center;
		}
		else{

			servo[i] = servo_pwm.close;
		}
	}
}

void PwmInitMotor(){

	//motor start
	HAL_TIM_PWM_Start(motor_tim.motor1, motor_channel.motor1);
	HAL_TIM_PWM_Start(motor_tim.motor2, motor_channel.motor2);
	HAL_TIM_PWM_Start(motor_tim.motor3, motor_channel.motor3);
	HAL_TIM_PWM_Start(motor_tim.motor4, motor_channel.motor4);

	HAL_TIM_PWM_Start(motor_tim.motor5, motor_channel.motor5);
	HAL_TIM_PWM_Start(motor_tim.motor6, motor_channel.motor6);
	HAL_TIM_PWM_Start(motor_tim.motor7, motor_channel.motor7);
	HAL_TIM_PWM_Start(motor_tim.motor8, motor_channel.motor8);

	//motor init
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor_pwm.init);

	__HAL_TIM_SET_COMPARE(motor_tim.motor5 , motor_channel.motor5, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor6 , motor_channel.motor6, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor7 , motor_channel.motor7, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor8 , motor_channel.motor8, motor_pwm.init);

	//初期化待機
	HAL_Delay(2500);
}

void PwmInitServo(){

	//servo start
	HAL_TIM_PWM_Start(servo_tim.servo1, servo_channel.servo1);
	HAL_TIM_PWM_Start(servo_tim.servo2, servo_channel.servo2);

	//servo init
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo_pwm.close);
	__HAL_TIM_SET_COMPARE(servo_tim.servo2 , servo_channel.servo2, servo_pwm.close);
}

void PwmGenerateUpperMotor(std::array<uint16_t,4>& motor){

	//motor
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor[0]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor[1]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor[2]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor[3]);
}

void PwmGenerateLowerMotor(std::array<uint16_t,4>& motor){

	//motor
	__HAL_TIM_SET_COMPARE(motor_tim.motor5 , motor_channel.motor5, motor[0]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor6 , motor_channel.motor6, motor[1]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor7 , motor_channel.motor7, motor[2]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor8 , motor_channel.motor8, motor[3]);
}

void PwmGenerateServo(std::array<uint16_t,2>& servo){

	//servo
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo[0]);
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo2, servo[1]);
}

void PwmGenerate(std::array<uint16_t,4>& upper_motor, std::array<uint16_t,4>& lower_motor, std::array<uint16_t,2>& servo){

	//motor
	PwmGenerateUpperMotor(upper_motor);
	PwmGenerateLowerMotor(lower_motor);

	//servo
	PwmGenerateServo(servo);
}

void PwmStop(){

	//motor
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor1);
	HAL_TIM_PWM_Stop(motor_tim.motor2, motor_channel.motor2);
	HAL_TIM_PWM_Stop(motor_tim.motor3, motor_channel.motor3);
	HAL_TIM_PWM_Stop(motor_tim.motor4, motor_channel.motor4);

	HAL_TIM_PWM_Stop(motor_tim.motor5, motor_channel.motor5);
	HAL_TIM_PWM_Stop(motor_tim.motor6, motor_channel.motor6);
	HAL_TIM_PWM_Stop(motor_tim.motor7, motor_channel.motor7);
	HAL_TIM_PWM_Stop(motor_tim.motor8, motor_channel.motor8);
}
