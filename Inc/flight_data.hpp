#ifndef INC_FLIGHT_DATA_HPP_
#define INC_FLIGHT_DATA_HPP_

struct FlightData{

	float accel[16][3] = {};
	float gyro[16][3] = {};
	float mag[16][3] = {};

	float angle[3] = {};//-90~90
	float position[3] = {};
	float speed[3] = {};
	float control[3] = {};
	uint8_t raw_sbus[25] = {};
    uint16_t sbus[10] = {};//0~2047
    uint16_t sbus_max[4] = {1662, 1662, 1680, 1662};
    uint16_t sbus_center[4] = {1000, 1000, 1000, 1000};
    uint16_t sbus_min[4] = {348, 348, 360, 348};
    uint16_t motor[4] = {};//PWM
    uint16_t servo[2] = {};//PWM
    float target_angle[3] = {};//-90~90
    uint16_t throttle = 0;

}data;

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

}channel;

struct Motor{

	uint16_t max  = 2500 * 0.95; //1500
	uint16_t min  = 2500 * 0.50; //1250
	uint16_t init = 2500 * 0.40; //1000
}motor;

struct Servo{

	uint16_t open = 400;
	uint16_t center = 1000;
	uint16_t close = 1800;
}servo;


//フラグ判定は 成功:0 失敗:1
uint8_t IsArm(){

	return data.sbus[channel.arm] < 1500;
}

uint8_t IsStart(){

	return data.sbus[channel.start] < 1500;
}

uint8_t IsAuto(){

	return data.sbus[channel.zidou] < 1500;
}

//----------単位換算----------//

void SbustoAngle(){

	//pitch(angle)
	data.target_angle[0] = ((float)(data.sbus[channel.pitch] - data.sbus_center[channel.pitch]) / (float)(data.sbus_max[channel.pitch] - data.sbus_center[channel.pitch])) * 45.0;

	//roll(angle)
	data.target_angle[1] = ((float)(data.sbus[channel.roll] - data.sbus_center[channel.roll]) / (float)(data.sbus_max[channel.roll] - data.sbus_center[channel.roll])) * 45.0;

	//yaw(speed)
	data.target_angle[2] = ((float)(data.sbus[channel.yaw] - data.sbus_center[channel.yaw]) / (float)(data.sbus_max[channel.yaw] - data.sbus_center[channel.yaw])) * 60;

	//throttle(500~2000)
	data.throttle = (float)(data.sbus[channel.throttle] - data.sbus_min[channel.throttle]) / (data.sbus_max[channel.throttle] - data.sbus_min[channel.throttle])* 750.0;

}

void PidtoPwm(){

	data.motor[0] = (motor.min) + data.throttle + data.control[0] + data.control[1] - data.control[2];
	data.motor[1] = (motor.min) + data.throttle + data.control[0] - data.control[1] + data.control[2];
	data.motor[2] = (motor.min) + data.throttle - data.control[0] + data.control[1] - data.control[2];
	data.motor[3] = (motor.min) + data.throttle - data.control[0] - data.control[1] + data.control[2];

	//PWMが最大値を超えないように処理
	for(uint8_t i=0; i<8; i++){

		if(data.motor[i] > motor.max){

			data.motor[i] = motor.max;
		}
	}
}

#endif
