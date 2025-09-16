#ifndef AUTOPILOT_DATA_HPP
#define AUTOPILOT_DATA_HPP

#include <cstdint>

// Autopilot data populated from P2P packets (stored as int32_t per packet spec)
struct AutopilotData {
    int32_t state = 0;
    // state: (送信する値は TYPE_STATE = 0x00 で送ります)
    // 0 = 起動 / ホバリング
    // 1 = 横移動
    // 2 = 前進
    // 3 = 着陸
    int32_t roll = 0;      // mapped parameter, -127..127
    
};

#endif // AUTOPILOT_DATA_HPP
