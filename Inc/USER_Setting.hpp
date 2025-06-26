/*
 * USER_Setting.hpp
 *
 *  Created on: Jun 21, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_USER_SETTING_HPP_
#define INC_USER_SETTING_HPP_

//---------------PID---------------//

//角度PIDの設定
struct AnglePID{

	//Kp, Ki, Kd
	float pitch[3] = {1.0, 0, 0};
	float roll[3] = {1.0, 0, 0};
	float yaw[3] = {1.0, 0, 0};

	//time(s)
	float time = 0.010;
};

//角速度PIDの設定
struct RatePID{

	//Kp, Ki, Kd
	float pitch[3] = {1.0, 0, 0};
	float roll[3] = {1.0, 0, 0};
	float yaw[3] = {1.0, 0, 0};

	//time(s)
	float time = 0.0025;
};

//--------------データ管理------------------//

struct SbusValue{

	uint16_t max[4] = {};
	uint16_t center[4] = {};
	uint16_t min[4] = {};
};
//---------------プロポの設定---------------//

struct Channel{

	//index = channel - 1
	uint8_t pitch = 2 - 1;
	uint8_t roll  = 1 - 1;
	uint8_t throttle = 3 - 1;
	uint8_t yaw = 4 - 1;
	uint8_t servo = 7 - 1;
	uint8_t arm = 6 - 1;
	uint8_t start = 5 - 1;
	uint8_t zidou = 8 - 1;
	uint8_t kosyou = 9 - 1;

};

//---------------Motor---------------//

/* モーター配置（上が前）
 * 	1	2
 *
 * 	3	4
 */

//モーターのタイマー番号
struct MotorTim{

	TIM_HandleTypeDef* motor1 = &htim1;
	TIM_HandleTypeDef* motor2 = &htim1;
	TIM_HandleTypeDef* motor3 = &htim1;
	TIM_HandleTypeDef* motor4 = &htim1;
};

//モーターのチャンネル番号
struct MotorChannel{

	uint32_t motor1 = TIM_CHANNEL_1;
	uint32_t motor2 = TIM_CHANNEL_2;
	uint32_t motor3 = TIM_CHANNEL_3;
	uint32_t motor4 = TIM_CHANNEL_4;
};

//---------------Servo---------------//

//サーボのタイマー番号
struct ServoTim{

	TIM_HandleTypeDef* servo1 = &htim12;
	TIM_HandleTypeDef* servo2 = &htim12;
};

//モーターチャンネル番号
struct ServoChannel{

	uint32_t servo1 = TIM_CHANNEL_1;
	uint32_t servo2 = TIM_CHANNEL_2;
};


#endif /* INC_USER_SETTING_HPP_ */
