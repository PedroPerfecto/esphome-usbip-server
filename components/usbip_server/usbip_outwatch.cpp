#include "usbip_outwatch.h"
#include <cstdio>

namespace usbip {

OutStuck out_stuck_check(const OutWatch &w, uint32_t now_ms, uint32_t max_age_ms, uint32_t orphan_grace_ms) {
  if (w.in_flight == 0) return OutStuck::kNone;
  if (w.have_oldest && static_cast<uint32_t>(now_ms - w.oldest_submit_ms) > max_age_ms) return OutStuck::kAge;
  if (w.have_orphan && static_cast<uint32_t>(now_ms - w.orphan_since_ms) >= orphan_grace_ms) return OutStuck::kOrphan;
  return OutStuck::kNone;
}

const char *ep0_probe_str(Ep0Probe r) {
  switch (r) {
    case Ep0Probe::kOk:
      return "ok";
    case Ep0Probe::kError:
      return "err";
    case Ep0Probe::kTimeout:
      return "timeout";
    default:
      return "not_sent";
  }
}

uint16_t out_stuck_pack(const OutStuckInfo &info) {
  const uint16_t pf = info.prefetch_in_flight > 15 ? 15 : info.prefetch_in_flight;
  return static_cast<uint16_t>(pf | (static_cast<uint16_t>(info.reason) << 4) |
                               (static_cast<uint16_t>(info.ep0) << 8));
}

bool out_stuck_unpack(uint16_t packed, OutStuckInfo *info) {
  const uint8_t reason = (packed >> 4) & 0x0F;
  const uint8_t ep0 = static_cast<uint8_t>(packed >> 8);
  if (reason != static_cast<uint8_t>(OutStuck::kAge) && reason != static_cast<uint8_t>(OutStuck::kOrphan)) return false;
  if (ep0 > static_cast<uint8_t>(Ep0Probe::kTimeout)) return false;
  info->reason = static_cast<OutStuck>(reason);
  info->ep0 = static_cast<Ep0Probe>(ep0);
  info->prefetch_in_flight = packed & 0x0F;
  return true;
}

size_t out_stuck_format(const OutStuckInfo &info, char *out, size_t cap) {
  if (cap == 0) return 0;
  const int n = snprintf(out, cap, "OUT_STUCK ep=%02X age=%u ms reason=%s ep0=%s prefetch=%u", info.ep,
                         static_cast<unsigned>(info.age_ms), info.reason == OutStuck::kOrphan ? "orphan" : "age",
                         ep0_probe_str(info.ep0), static_cast<unsigned>(info.prefetch_in_flight));
  if (n < 0) {
    out[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1;
}

}  // namespace usbip
