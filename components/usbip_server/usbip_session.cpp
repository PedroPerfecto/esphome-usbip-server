#include "usbip_session.h"
#include <cstring>

namespace usbip {

bool takeover_allowed(bool device_ready, const char *requested, const char *exported) {
  if (!device_ready || requested == nullptr || exported == nullptr) return false;
  return std::strcmp(requested, exported) == 0;
}

}  // namespace usbip
