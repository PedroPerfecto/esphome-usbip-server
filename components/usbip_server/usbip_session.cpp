#include "usbip_session.h"
#include <cstring>

namespace usbip {

bool takeover_allowed(bool device_ready, const char *requested, const char *exported) {
  if (!device_ready || requested == nullptr || exported == nullptr) return false;
  return std::strcmp(requested, exported) == 0;
}

bool recv_timeout_before(int64_t now_us, int64_t deadline_us, uint32_t *timeout_ms) {
  if (now_us >= deadline_us) return false;
  const int64_t left_us = deadline_us - now_us;
  const int64_t ms = left_us / 1000 + (left_us % 1000 != 0 ? 1 : 0);
  *timeout_ms = ms > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(ms);
  return true;
}

}  // namespace usbip
