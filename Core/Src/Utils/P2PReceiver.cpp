#include "Utils/P2PReceiver.hpp"
#include "ESP32_P2P_Utility/P2PPacketDecoder.hpp"

using namespace Utils;

void P2PReceiver::Process(const uint8_t* buf, size_t len, FlightManager& manager) {

    if (buf == nullptr || len == 0) return;

    // Ensure fixed length
    if (len != UserSetting::P2P_PACKET_SIZE) return;

    P2PPacketDecoder decoder;
    if (decoder.SetData(buf, static_cast<uint8_t>(len)) != PacketError::SUCCESS) return;

    // Extract fields according to UserSetting::PacketDataType mapping
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    uint32_t throttle = 0;

    // Attempt to read each field; ignore failures
    decoder.GetData(static_cast<UserSetting::PacketDataType>(UserSetting::PacketDataType::Pitch), pitch);
    decoder.GetData(static_cast<UserSetting::PacketDataType>(UserSetting::PacketDataType::Roll), roll);
    decoder.GetData(static_cast<UserSetting::PacketDataType>(UserSetting::PacketDataType::Yaw), yaw);
    decoder.GetData(static_cast<UserSetting::PacketDataType>(UserSetting::PacketDataType::Throttle), throttle);


}
