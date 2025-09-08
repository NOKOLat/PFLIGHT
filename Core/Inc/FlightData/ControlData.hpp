#ifndef CONTROL_DATA_HPP
#define CONTROL_DATA_HPP

#include <cstdint>
#include <array>

struct ControlData {

    std::array<float, 3> target_rate = {};
    std::array<float, 3> pid_result = {};
	std::array<uint16_t, 4> motor_pwm = {};
	std::array<uint16_t, 2> servo_pwm = {};
};

#endif // CONTROL_DATA_HPP
