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

size_t out_stuck_format(const OutStuckInfo &info, char *out, size_t cap, bool suppressed) {
  if (cap == 0) return 0;
  const int n = snprintf(out, cap, "%s ep=%02X age=%u ms reason=%s ep0=%s prefetch=%u",
                         suppressed ? "OUT_STUCK_SUPPRESSED" : "OUT_STUCK", info.ep,
                         static_cast<unsigned>(info.age_ms), info.reason == OutStuck::kOrphan ? "orphan" : "age",
                         ep0_probe_str(info.ep0), static_cast<unsigned>(info.prefetch_in_flight));
  if (n < 0) {
    out[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1;
}

void guard_reset(RebootGuard &g) {
  g.boots = 0;
  g.run_s = 0;
  g.base_s = 0;
  g.last_stop_s = 0;
  for (uint32_t &t : g.reboot_s) t = 0;
  g.reboots = 0;
}

bool guard_valid(const RebootGuard &g) {
  if (g.reboots > kGuardMaxReboots || g.base_s > g.run_s || g.last_stop_s > g.run_s) return false;
  for (uint8_t i = 0; i < g.reboots; ++i) {
    if (g.reboot_s[i] > g.run_s || (i > 0 && g.reboot_s[i] < g.reboot_s[i - 1])) return false;
  }
  return true;
}

void guard_boot(RebootGuard &g) {
  if (g.boots != UINT32_MAX) ++g.boots;
  g.base_s = g.run_s;
}

void guard_tick(RebootGuard &g, uint32_t uptime_s) {
  const uint32_t run = uptime_s > UINT32_MAX - g.base_s ? UINT32_MAX : g.base_s + uptime_s;
  if (run > g.run_s) g.run_s = run;  // never backwards
  if (g.reboots > 0 && g.run_s - g.last_stop_s >= kGuardWindowS) g.reboots = 0;
}

void guard_note_stop(RebootGuard &g) { g.last_stop_s = g.run_s; }

uint8_t guard_recent_reboots(const RebootGuard &g) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < g.reboots && i < kGuardMaxReboots; ++i) {
    if (g.run_s - g.reboot_s[i] < kGuardWindowS) ++n;
  }
  return n;
}

GuardAction guard_on_stuck(RebootGuard &g) {
  guard_note_stop(g);
  if (guard_recent_reboots(g) >= kGuardMaxReboots) return GuardAction::kSuppress;
  if (g.reboots >= kGuardMaxReboots) {  // drop the oldest
    for (uint8_t i = 1; i < kGuardMaxReboots; ++i) g.reboot_s[i - 1] = g.reboot_s[i];
    g.reboots = kGuardMaxReboots - 1;
  }
  g.reboot_s[g.reboots++] = g.run_s;
  return GuardAction::kReboot;
}

}  // namespace usbip
