#include "Utils/P2PReceiver.hpp"
#include "ESP32_P2P_Utility/P2PPacketDecoder.hpp"
#include <cstdint>

namespace P2PPackage {

    P2PReceiver::P2PReceiver() {
        // Initialize buffer to zero
        for (size_t i = 0; i < UserSetting::P2P_PACKET_SIZE; ++i) {
            receive_buffer[i] = 0;
        }
    }

    uint8_t* P2PReceiver::getReceiveBufferPtr() {
        return receive_buffer;
    }

    size_t P2PReceiver::getDataLen() const {
        return static_cast<size_t>(UserSetting::P2P_PACKET_SIZE);
    }

    void P2PReceiver::Process(FlightManager& manager) {
        // Process data stored in internal buffer
        P2PReceiver::Process(receive_buffer, getDataLen(), manager);
    }

    void P2PReceiver::Process(const uint8_t* buf, size_t len, FlightManager& manager) {

        if (buf == nullptr || len == 0) return;

        // Ensure fixed length
        if (len != UserSetting::P2P_PACKET_SIZE) return;

        P2PPacketDecoder decoder;
        if (decoder.SetData(buf, static_cast<uint8_t>(len)) != PacketError::SUCCESS) return;

        // Extract fields according to UserSetting::PacketDataType mapping
        // Per spec these are stored/provisioned as int32_t on the wire.
        int32_t get_data[4] = {};

        // Attempt to read each field; ignore failures
        decoder.GetData(UserSetting::PacketDataType::Pitch, get_data[0]);
        decoder.GetData(UserSetting::PacketDataType::Roll, get_data[1]);
        decoder.GetData(UserSetting::PacketDataType::Yaw, get_data[2]);
        decoder.GetData(UserSetting::PacketDataType::Throttle, get_data[3]);

        // Sender transmits int8_t promoted to int32_t. Recover original int8_t
        // by clamping to int8_t range. Loop over first three axis fields.
        for (uint8_t i = 0; i < 3; ++i) {
            if (get_data[i] > 127) get_data[i] = 127;
            if (get_data[i] < -127) get_data[i] = -127;
        }

        manager.autopilot_data.pitch = static_cast<int8_t>(get_data[0]);
        manager.autopilot_data.roll = static_cast<int8_t>(get_data[1]);
        manager.autopilot_data.yaw = static_cast<int8_t>(get_data[2]);

        // For throttle, clamp negative to 0 and cap at 255
        int32_t throttle = get_data[3];
        if (throttle < 0) throttle = 0;
        if (throttle > 255) throttle = 255;
        manager.autopilot_data.throttle = static_cast<uint8_t>(throttle);
    }

}

