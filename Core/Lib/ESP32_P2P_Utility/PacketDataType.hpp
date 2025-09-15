#ifndef PACKET_DATA_TYPE_HPP
#define PACKET_DATA_TYPE_HPP

#include <cstdint>

// 単一の enum class にまとめた PacketDataType
    enum class PacketDataType : uint8_t {

        // int32_t (上位2ビット = 00 -> 0x00〜0x3F)
        State = 0x00,
        Roll = 0x01,
        

        // uint32_t (上位2ビット = 01 -> 0x40〜0x7F)

        // float (上位2ビット = 10 -> 0x80〜0xBF)

    };

#endif // PACKET_DATA_TYPE_HPP
