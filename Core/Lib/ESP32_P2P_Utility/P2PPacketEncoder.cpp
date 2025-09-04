#include "P2PPacketEncoder.hpp"

// helper: append 1 type + 4 bytes value
static PacketError AddValue(P2PEncodedPacket &packet, uint8_t type, const void* valPtr) {
    const size_t need = 5;
    if (static_cast<size_t>(packet.current_packet_size) + need > packet.data.size()) {
        return PacketError::PACKET_OVERFLOW;
    }

    size_t idx = packet.current_packet_size;
    packet.data[idx] = type;
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(valPtr);
    // store 4 bytes
    packet.data[idx + 1] = bytes[0];
    packet.data[idx + 2] = bytes[1];
    packet.data[idx + 3] = bytes[2];
    packet.data[idx + 4] = bytes[3];

    packet.current_packet_size = static_cast<uint8_t>(idx + 5);
    return PacketError::SUCCESS;
}

PacketError P2PPacketEncoder::Begin() {
    if (has_begin_) return PacketError::TYPE_MISMATCH;
    packet.current_packet_size = 0;
    // start byte per spec
    packet.data[0] = 0x0F;
    packet.current_packet_size = 1;
    has_begin_ = true;
    has_end_ = false;
    ptr_given_ = false;
    return PacketError::SUCCESS;
}

PacketError P2PPacketEncoder::End() {
    if (!has_begin_) return PacketError::TYPE_MISMATCH;
    if (has_end_) return PacketError::TYPE_MISMATCH;
    if (static_cast<size_t>(packet.current_packet_size) + 1 > packet.data.size()) return PacketError::PACKET_OVERFLOW;
    packet.data[packet.current_packet_size] = 0xF0;
    packet.current_packet_size = static_cast<uint8_t>(packet.current_packet_size + 1);
    has_end_ = true;
    return PacketError::SUCCESS;
}

PacketError P2PPacketEncoder::AddData(PacketDataType type, float data) {
    if (!has_begin_) return PacketError::PACKET_NOT_INITIALIZED;
    if (has_end_) return PacketError::TYPE_MISMATCH;
    return AddValue(packet, static_cast<uint8_t>(type), &data);
}

PacketError P2PPacketEncoder::AddData(PacketDataType type, uint32_t data) {
    if (!has_begin_) return PacketError::PACKET_NOT_INITIALIZED;
    if (has_end_) return PacketError::TYPE_MISMATCH;
    return AddValue(packet, static_cast<uint8_t>(type), &data);
}

PacketError P2PPacketEncoder::AddData(PacketDataType type, int32_t data) {
    if (!has_begin_) return PacketError::PACKET_NOT_INITIALIZED;
    if (has_end_) return PacketError::TYPE_MISMATCH;
    return AddValue(packet, static_cast<uint8_t>(type), &data);
}

PacketError P2PPacketEncoder::GetData(uint8_t** outPtr, uint8_t* outSize) {
    if (outPtr == nullptr || outSize == nullptr) return PacketError::TYPE_MISMATCH;
    if (!has_begin_) return PacketError::PACKET_NOT_INITIALIZED;
    if (!has_end_) return PacketError::PACKET_NOT_ENDED;
    if (ptr_given_) return PacketError::PACKET_ALREADY_RETRIEVED;

    *outPtr = packet.data.data();
    *outSize = packet.current_packet_size;
    ptr_given_ = true;

    // reset internal state for next Begin()
    has_begin_ = false;
    has_end_ = false;
    packet.current_packet_size = 0;

    return PacketError::SUCCESS;
}

uint8_t P2PPacketEncoder::GetPacketSize() const {
    return packet.current_packet_size;
}
