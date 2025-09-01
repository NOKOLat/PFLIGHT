#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <cstdint>
#include <array>

struct SensorData {

	std::array<float, 3> accel = {};
	std::array<float, 3> gyro = {};
	std::array<float, 3> angle = {};
    float altitude = 0.0f;
    uint16_t adc_value = 0;
    uint16_t temperature = 0;
    uint16_t pressure = 0;  
};

#endif // SENSOR_DATA_HPP