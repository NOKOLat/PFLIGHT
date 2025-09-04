#include "P2PPacketDecoder.hpp"

PacketError P2PPacketDecoder::SetData(const uint8_t* buffer, uint8_t size) {
    if (buffer == nullptr || size == 0) return PacketError::TYPE_MISMATCH;

    // basic validation: start 0x0F and end 0xF0
    if (buffer[0] != 0x0F) return PacketError::TYPE_MISMATCH;
    if (buffer[size - 1] != 0xF0) return PacketError::TYPE_MISMATCH;

    if (size > packet.data.size()) return PacketError::PACKET_OVERFLOW;

    // copy to internal buffer
    std::memcpy(packet.data.data(), buffer, size);
    packet.current_packet_size = size;
    has_data_ = true;
    return PacketError::SUCCESS;
}

// helper: find type byte in packet and copy 4 bytes into outPtr. returns SUCCESS or TYPE_MISMATCH
static PacketError FindAndCopy(const P2PDecodedPacket &packet, uint8_t type, void* outPtr) {
    // entries start at index 1 and end before last byte
    size_t idx = 1;
    while (idx + 5 <= packet.current_packet_size) {
        uint8_t t = packet.data[idx];
        if (t == type) {
            const uint8_t* bytes = packet.data.data() + idx + 1;
            std::memcpy(outPtr, bytes, 4);
            return PacketError::SUCCESS;
        }
        idx += 5;
    }
    return PacketError::TYPE_MISMATCH;
}

PacketError P2PPacketDecoder::GetData(PacketDataType type, float &out) {
    if (!has_data_) return PacketError::PACKET_NOT_INITIALIZED;
    return FindAndCopy(packet, static_cast<uint8_t>(type), &out);
}

PacketError P2PPacketDecoder::GetData(PacketDataType type, uint32_t &out) {
    if (!has_data_) return PacketError::PACKET_NOT_INITIALIZED;
    return FindAndCopy(packet, static_cast<uint8_t>(type), &out);
}

PacketError P2PPacketDecoder::GetData(PacketDataType type, int32_t &out) {
    if (!has_data_) return PacketError::PACKET_NOT_INITIALIZED;
    return FindAndCopy(packet, static_cast<uint8_t>(type), &out);
}
