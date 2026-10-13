#pragma once
#include <cstdint>

// Time differences on free-running unsigned 32-bit clocks (no ESPHome/ESP-IDF dependencies).
namespace usbip {

// Time elapsed from `since` to `now`, both readings of the same 32-bit clock (ms, us or ticks).
// Correct across a wrap of the clock. A `since` that is newer than `now` gives 0 instead of a
// huge value: the two readings may come from different tasks, and the one taken as "now" may in
// fact be the older one (the other task stored a fresh time right after `now` was read). Plain
// `now - since` would then be close to 2^32 and look like an age of 49 days.
// Valid for real differences below 2^31 (24.8 days in ms, 35.7 min in us).
inline uint32_t elapsed32(uint32_t now, uint32_t since) {
  const int32_t d = static_cast<int32_t>(now - since);
  return d < 0 ? 0 : static_cast<uint32_t>(d);
}

}  // namespace usbip
