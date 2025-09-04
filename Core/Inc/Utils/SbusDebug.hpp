#pragma once

#include "FlightData/SbusData.hpp"

// Debug API for overriding SBUS channel data from anywhere without
// modifying existing code. Include this header and set
// DebugSbus::overrideData.<field> = ...; DebugSbus::enableOverride(true);

namespace DebugSbus {

extern SbusChannelData overrideData;

// Enable or disable global override. When enabled, call applyOverride
// where you have a local SbusChannelData instance to copy the override into it.
void enableOverride(bool en);

bool isOverrideEnabled();

// If override is enabled, this copies the override into dst.
// Call this from any place that receives or reads SBUS data (e.g. in
// a debug hook or in user code). This library intentionally does not
// modify existing project files.
void applyOverride(SbusChannelData& dst);

} // namespace DebugSbus
