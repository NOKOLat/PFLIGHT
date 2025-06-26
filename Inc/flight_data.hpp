#ifndef INC_FLIGHT_DATA_HPP_
#define INC_FLIGHT_DATA_HPP_

#include "USER_Setting.hpp"

SbusValue sbus_value;//USER_Setting.hppから値をもらうための構造体

struct FlightData{

	//Sensor
	float accel[3] = {};
	float gyro[3] = {};
	float mag[3] = {};
	float angle[3] = {};
	float rate[3] = {};

	//SBUS
	uint8_t raw_sbus[25] = {};
    uint16_t sbus[10] = {};//0~2047
    uint16_t sbus_max[4] = {};
    uint16_t sbus_center[4] = {};
    uint16_t sbus_min[4] = {};

    //PWN
    uint16_t motor[4] = {};
    uint16_t servo[2] = {};

    //PID
    float target_angle[3] = {};//pitch, roll
    float target_rate[3] = {};//pitch, roll, yaw
    float control[3] = {};//pitch, roll, yaw
    uint16_t throttle = 0;

    FlightData(uint16_t max[4], uint16_t center[4], uint16_t min[4]){

    	for(uint8_t i=0; i<4; i++){

    	    sbus_max[i] = max[i];
    	    sbus_center[i] = center[i];
    	    sbus_min[i] = min[i];
    	}
    }

};

FlightData data(sbus_value.max, sbus_value.center, sbus_value.min);
Channel channel;


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
	data.target_angle[0] = ((float)(data.sbus[channel.pitch] - data.sbus_center[channel.pitch]) / (float)(data.sbus_max[channel.pitch] - data.sbus_center[channel.pitch])) * 30.0;

	//roll(angle)
	data.target_angle[1] = ((float)(data.sbus[channel.roll] - data.sbus_center[channel.roll]) / (float)(data.sbus_max[channel.roll] - data.sbus_center[channel.roll])) * 30.0;

	//yaw(speed)
	data.target_angle[2] = ((float)(data.sbus[channel.yaw] - data.sbus_center[channel.yaw]) / (float)(data.sbus_max[channel.yaw] - data.sbus_center[channel.yaw])) * 60.0;

	//throttle(500~2000)
	data.throttle = (float)(data.sbus[channel.throttle] - data.sbus_min[channel.throttle]) / (data.sbus_max[channel.throttle] - data.sbus_min[channel.throttle])* 750.0;

}

#endif
