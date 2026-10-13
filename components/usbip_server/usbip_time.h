#pragma once
#include <cstdint>

// Time differences on free-running unsigned 32-bit clocks (no ESPHome/ESP-IDF dependencies).
namespace usbip {

// Time elapsed from `since` to `now`, both readings of the same 32-bit clock (ms, us or ticks).
// Correct across a wrap of the clock, for the full unsigned range.
//
// The two readings may come from different tasks, and the one taken as "now" may in fact be the
// older one (the other task stored a fresh time right after `now` was read). Plain `now - since`
// is then just below 2^32 and looks like an age of 49 days. A `since` that is newer than `now` by
// up to kElapsedSkew therefore counts as 0. Only that narrow band is treated this way: a signed
// difference would turn every real age above 2^31 (24.8 days in ms) into 0 as well, and something
// that happened long ago would look as if it had just happened.
static constexpr uint32_t kElapsedSkew = 0x10000;  // 65.5 s in ms, 65.5 ms in us

inline uint32_t elapsed32(uint32_t now, uint32_t since) {
  const uint32_t d = now - since;
  return d > UINT32_MAX - kElapsedSkew ? 0 : d;
}

}  // namespace usbip
