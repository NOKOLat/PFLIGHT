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

        // Packet format used by ESP32 P2P utility:
        //  - Start byte: 1 byte (0x0F)
        //  - For each field: 1 byte (type) + 4 bytes (value)
        //    -> each field entry consumes 5 bytes on the wire
        //  - End byte: 1 byte (0xF0)
        // Examples of field value types and their packet cost:
        //  - float         : 4 bytes (value) -> 1(type)+4(value) = 5 bytes per field
        //  - int32_t       : 4 bytes (value) -> 1(type)+4(value) = 5 bytes per field
        //  - uint32_t      : 4 bytes (value) -> 1(type)+4(value) = 5 bytes per field
        // Calculation (compile-time): total = 1(start) + N*(1+4) + 1(end) = 2 + 5*N
        // Current PacketDataType entries used by receiver:
        //  - Pitch  (int32_t)
        //  - Roll   (int32_t)
        //  - Yaw    (int32_t)
        //  - Throttle (int32_t)
        // Therefore N = 4 -> P2P_PACKET_SIZE = 2 + 5*4 = 22
        constexpr uint16_t P2P_PACKET_SIZE = 22;

}

