#include "Utils/SbusDebug.hpp"

namespace DebugSbus {

// Storage for the override values. Default constructed matches SbusChannelData defaults.
SbusChannelData overrideData;

// simple volatile flag for embedded use (no dynamic allocation / atomics required).
static volatile bool enabled = false;

void enableOverride(bool en) {
    enabled = en;
}

bool isOverrideEnabled() {
    return enabled;
}

void applyOverride(SbusChannelData& dst) {
    if (!enabled) return;
    // copy entire struct - simple and predictable for debug use
    dst = overrideData;
}

} // namespace DebugSbus
