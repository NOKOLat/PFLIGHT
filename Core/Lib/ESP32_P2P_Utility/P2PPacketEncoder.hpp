#ifndef P2P_PACKET_ENCODER_HPP
#define P2P_PACKET_ENCODER_HPP

#include <array>
#include <cstdint>
#include <cstring>

#include "PacketDataType.hpp"
#include "PacketUtility.hpp"

struct P2PEncodedPacket {
    std::array<uint8_t, 256> data{};
    uint8_t current_packet_size = 0;
};

class P2PPacketEncoder {
public:
    P2PPacketEncoder() = default;

    PacketError Begin();
    PacketError End();

    PacketError AddData(PacketDataType type, float data);
    PacketError AddData(PacketDataType type, uint32_t data);
    PacketError AddData(PacketDataType type, int32_t data);

    // Get the internal encoded buffer pointer and size. After this call
    // the caller may use the pointer to send the bytes. A subsequent Begin()
    // will discard the internal buffer.
    PacketError GetData(uint8_t** outPtr, uint8_t* outSize);
    uint8_t GetPacketSize() const;

private:
    P2PEncodedPacket packet;
    bool has_begin_ = false;
    bool has_end_ = false;
    bool ptr_given_ = false; // whether GetData has been called and ptr handed out
};

#endif // P2P_PACKET_ENCODER_HPP
