#ifndef SBUS_DATA_HPP
#define SBUS_DATA_HPP

#include <cstdint>
#include <array>

struct SbusChannelData{

    // 中立の値
    std::array<uint16_t, 4> max = {1600, 1600, 1680, 1600};
	std::array<uint16_t, 4> min = {400, 400, 368, 400};
	std::array<uint16_t, 4> center = {1000, 1000, 1000, 1000};

    // 最大値
    float angle_pitch_max = 30.0f;
    float angle_roll_max = 30.0f;
    float rate_yaw_max = 60.0f;
    float throttle_max = 750.0f; // PWMの最大値を指定(PWMは0~1000の範囲 値を大きくしすぎると高回転時に姿勢制御が難しくなる)

    // 目標値
	std::array<float, 3> target_value;
    // トリム値（送られてきたSBUSのニュートラル偏差を保持）
    std::array<float, 3> trim = {0.0f, 0.0f, 0.0f};
    float throttle = 0.0;

    // スイッチ
    bool arm = false;
    bool fly = false;
    uint8_t drop = false;
    bool autodrop = false;
    bool autofly = false;
    bool emergency_control = false;
    uint8_t stop_motor_side = 0;
    // 受信判定
    bool is_receive = false;

    // failsafe
    bool failsafe_bit = false;
    bool timeout = false;
};

enum class SbusChannel: uint8_t{

    //channel = propo_index - 1;
	throttle = 3 - 1,
	pitch = 2 - 1,
	roll = 1 - 1,
	yaw = 4 - 1,
	arm = 6 - 1,
	fly = 5 - 1,
	drop = 7 - 1,
	autodrop = 8 - 1,
	autofly = 9 - 1,
    emergency_control = 10 - 1,
};

#endif // SBUS_DATA_HPP
