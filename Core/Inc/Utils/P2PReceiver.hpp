#pragma once

#include "FlightManager.h"
#include "UserSetting/P2PPacketSetting.hpp"
#include "ESP32_P2P_Utility/P2PPacketDecoder.hpp"
#include <cstdint>

namespace P2PPackage {

class P2PReceiver {
public:
    // Process raw fixed-length buffer received from UART3 and update FlightManager
    static void Process(const uint8_t* buf, size_t len, FlightManager& manager);

    // Instance API: hold an internal receive buffer so caller can pass
    // its pointer to HAL_UART_Receive_DMA.
    P2PReceiver();

    // Returns pointer to internal buffer suitable for DMA receive
    uint8_t* getReceiveBufferPtr();

    // Returns the number of bytes the DMA should receive into the buffer
    size_t getDataLen() const;

    // Process data currently stored in the internal buffer
    void Process(FlightManager& manager);

private:
    // Internal aligned buffer sized to P2P packet length
    alignas(uint32_t) uint8_t receive_buffer[UserSetting::P2P_PACKET_SIZE];
};

}
