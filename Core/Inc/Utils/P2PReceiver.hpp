#pragma once

#include "FlightManager.h"
#include "UserSetting/P2PPacketSetting.hpp"
#include "ESP32_P2P_Utility/P2PPacketDecoder.hpp"
#include <cstdint>

namespace Utils {

class P2PReceiver {
public:
    // Process raw fixed-length buffer received from UART3 and update FlightManager
    static void Process(const uint8_t* buf, size_t len, FlightManager& manager);
};

}
