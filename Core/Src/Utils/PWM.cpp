#include "Utils/PWM.hpp"
#include <array>

// PIDの制御量をモータに分配
void PWM::calcMotor(float throttle, std::array<float,3>& control, std::array<uint16_t,4>& motor){
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

void PWM::calcServo(SbusChannelData sbus_data, uint16_t adc_value, std::array<uint16_t, 2>& servo){
	for(uint8_t i=0; i<2; i++){
		if((sbus_data.autodrop && (adc_value > 2000)) || sbus_data.drop == 2){
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

void PWM::initMotor(){
	//motor start
	HAL_TIM_PWM_Start(motor_tim.motor1, motor_channel.motor1);
	HAL_TIM_PWM_Start(motor_tim.motor2, motor_channel.motor2);
	HAL_TIM_PWM_Start(motor_tim.motor3, motor_channel.motor3);
	HAL_TIM_PWM_Start(motor_tim.motor4, motor_channel.motor4);

	//motor init
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor2 , motor_channel.motor2, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor3 , motor_channel.motor3, motor_pwm.init);
	__HAL_TIM_SET_COMPARE(motor_tim.motor4 , motor_channel.motor4, motor_pwm.init);

	//初期化待機
	HAL_Delay(2500);
}

void PWM::initServo(){
	//servo start
	HAL_TIM_PWM_Start(servo_tim.servo1, servo_channel.servo1);
	HAL_TIM_PWM_Start(servo_tim.servo2, servo_channel.servo2);

	//servo init
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo_pwm.close);
	__HAL_TIM_SET_COMPARE(servo_tim.servo2 , servo_channel.servo2, servo_pwm.close);
}

void PWM::generateMotor(std::array<uint16_t,4>& motor){
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor1, motor[0]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor2, motor[1]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor3, motor[2]);
	__HAL_TIM_SET_COMPARE(motor_tim.motor1 , motor_channel.motor4, motor[3]);
}

void PWM::generateServo(std::array<uint16_t,2>& servo){
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo[0]);
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo2, servo[1]);
}

void PWM::generate(std::array<uint16_t,4>& motor, std::array<uint16_t,2>& servo){
	generateMotor(motor);
	generateServo(servo);
}

void PWM::generate(std::array<uint16_t,4>& /*upper_motor*/, std::array<uint16_t,4>& /*lower_motor*/, std::array<uint16_t,2>& /*servo*/){
	// 未使用オーバーロード（将来拡張用）
}

void PWM::motorStop(){
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor1);
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor2);
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor3);
	HAL_TIM_PWM_Stop(motor_tim.motor1, motor_channel.motor4);
}

void PWM::stop(){
	motorStop();
}
