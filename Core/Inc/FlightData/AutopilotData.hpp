#ifndef AUTOPILOT_DATA_HPP
#define AUTOPILOT_DATA_HPP

#include <cstdint>

// Autopilot data populated from P2P packets (stored as int32_t per packet spec)
struct AutopilotData {
    int32_t pitch = 0;     // mapped parameter, -127..127
    int32_t roll = 0;      // mapped parameter, -127..127
    int32_t yaw = 0;       // mapped parameter, -127..127
    int32_t throttle = 0;  // mapped parameter, 0..255 (used as altitude proxy; 255 -> 1.0 m)

};

#endif // AUTOPILOT_DATA_HPP
