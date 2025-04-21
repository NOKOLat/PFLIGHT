#ifndef INC_FLIGHT_DATA_HPP_
#define INC_FLIGHT_DATA_HPP_

#include "RingBuffer.hpp"

struct FlightData{

	RingBuffer<float, 2> accel;
	RingBuffer<float, 2> gyro;
	RingBuffer<float, 2> mag;

	float angle[3] = {};//-90~90
	float position[3] = {};
	float speed[3] = {};
	float control[3] = {};
	uint8_t raw_sbus[25] = {};
    uint16_t sbus[10] = {};//0~2047
    uint16_t sbus_max[4] = {1500, 1500, 1680, 1500};
    uint16_t sbus_center[4] = {1000, 1000, 1000, 1000};
    uint16_t sbus_min[4] = {500, 500, 360, 500};
    uint16_t motor[4] = {};//PWM
    uint16_t servo[2] = {};//PWM
    float target_angle[3] = {};//-90~90
    uint16_t throttle = 0;


    //RingBufferの初期化
    FlightData():accel(16,3), gyro(16,3), mag(16,3){}
}data;


struct Channel{

	//index = channel - 1
	uint8_t pitch = 2 - 1;
	uint8_t roll  = 1 - 1;
	uint8_t throttle = 3 - 1;
	uint8_t yaw = 4 - 1;
	uint8_t servo = 5 - 1;
	uint8_t arm = 6 - 1;
	uint8_t start = 7 - 1;
	uint8_t zidou = 8 - 1;
	uint8_t kosyou = 9 - 1;

}channel;

struct Motor{

	uint16_t max  = 2500 * 0.55; //1375
	uint16_t min  = 2500 * 0.45; //1125
	uint16_t start = 2500 * 0.40; //1000
}motor;

struct Servo{

	uint16_t open = 400;
	uint16_t center = 1000;
	uint16_t close = 1800;
}servo;


//フラグ判定は 成功:0 失敗:1
uint8_t IsArm(){

	return data.sbus[channel.arm] < 1000;
}

uint8_t IsStart(){

	return data.sbus[channel.start] < 1000;
}

uint8_t IsAuto(){

	return data.sbus[channel.zidou] < 1000;
}

//----------単位換算----------//

void SbustoAngle(){

	//pitch(angle)
	data.target_angle[0] = (data.sbus[channel.pitch] - data.sbus_center[channel.pitch]) * 45.0 / (data.sbus_max[channel.pitch] - data.sbus_center[channel.pitch]) / 45.0 * 100;

	//roll(angle)
	data.target_angle[1] = (data.sbus[channel.roll] - data.sbus_center[channel.roll]) * 45.0 / (data.sbus_max[channel.roll] - data.sbus_center[channel.roll]) / 45.0 * 100;

	//yaw(speed)
	data.target_angle[2] = (data.sbus[channel.yaw] - data.sbus_center[channel.yaw]) * 45.0 / (data.sbus_max[channel.yaw] - data.sbus_center[channel.yaw]) / 45.0 * 100;

	//throttle(500~2000)
	data.throttle = (data.sbus[channel.throttle] - data.sbus_min[channel.throttle])*1.0 / (data.sbus_max[channel.throttle] - data.sbus_min[channel.throttle])* 100.0;
}

void PidtoPwm(){

	data.motor[0] = (2500 * 0.45) + (data.sbus[channel.throttle] - 360) / (1680-360) * 250 + (data.control[0] + data.control[1] - data.control[2]);
	data.motor[1] = (2500 * 0.45) + (data.sbus[channel.throttle] - 360) / (1680-360) * 250 + (data.control[0] - data.control[1] + data.control[2]);
	data.motor[2] = (2500 * 0.45) + (data.sbus[channel.throttle] - 360) / (1680-360) * 250 - (data.control[0] + data.control[1] - data.control[2]);
	data.motor[3] = (2500 * 0.45) + (data.sbus[channel.throttle] - 360) / (1680-360) * 250 - (data.control[0] - data.control[1] + data.control[2]);
}

#endif
