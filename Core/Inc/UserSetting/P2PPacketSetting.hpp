#pragma once

#include <cstdint>

// P2P packet data type definitions moved from Core/Lib/ESP32_P2P_Utility/PacketDataType.hpp
namespace UserSetting {

// 単一の enum class にまとめた PacketDataType
enum class PacketDataType : uint8_t {
	// int32_t (上位2ビット = 00 -> 0x00〜0x3F)
	Pitch = 0x00,
    Roll = 0x01,
    Yaw = 0x02,
    Throttle = 0x03,

    // uint32_t (上位2ビット = 01 -> 0x40〜0x7F)

    // float (上位2ビット = 10 -> 0x80〜0xBF)


};

} // namespace UserSetting

// P2P packet fixed receive size (update if your packet length changes)
namespace UserSetting {
    constexpr size_t P2P_PACKET_SIZE = 17;
}

