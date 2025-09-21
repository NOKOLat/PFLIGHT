#include "Utils/PWM.hpp"
#include <array>

// Servoの開閉判定
void PWM::CalcServo(SbusChannelData sbus_data, uint16_t adc_value, std::array<uint16_t, 2>& servo){

	for(uint8_t i=0; i<2; i++){

		// 自動投下状態で赤外線センサーの値が閾値を超えている、またはスイッチが2段階目に入っている場合に開く
		if((sbus_data.autodrop && (adc_value > UserSetting::ir_threshold)) || sbus_data.drop == 2){

			servo[i] = servo_pwm.open;
		} 
		else if(sbus_data.drop == 1){

			servo[i] = servo_pwm.center;
		} 
		else {
			
			servo[i] = servo_pwm.close;
		}
	}
}

// Servo初期化
void PWM::InitServo(){

	HAL_TIM_PWM_Start(servo_tim.servo1, servo_channel.servo1);
	HAL_TIM_PWM_Start(servo_tim.servo2, servo_channel.servo2);

	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo_pwm.close);
	__HAL_TIM_SET_COMPARE(servo_tim.servo2 , servo_channel.servo2, servo_pwm.close);
}

// Servo出力
void PWM::GenerateServo(std::array<uint16_t,2>& servo){

	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo1, servo[0]);
	__HAL_TIM_SET_COMPARE(servo_tim.servo1 , servo_channel.servo2, servo[1]);
}

// Servo設定
void PWM::SetServoConfig(const ServoTim& tim, const ServoChannel& channel, const ServoPWM& pwm){
	
	servo_tim = tim;
	servo_channel = channel;
	servo_pwm = pwm;
}
