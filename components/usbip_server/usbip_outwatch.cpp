#include "usbip_outwatch.h"
#include "usbip_time.h"
#include <cstdio>

namespace usbip {

OutStuck xfer_stuck(const WatchedXfer &x, uint32_t now_ms, uint32_t timeout_ms) {
  if (!x.in_flight) return OutStuck::kNone;
  if (elapsed32(now_ms, x.since_ms) <= timeout_ms) return OutStuck::kNone;
  if (x.orphan) return OutStuck::kOrphan;
  return x.intr_out ? OutStuck::kAge : OutStuck::kNone;
}

bool watch_same(const WatchSample &first, const WatchSample &second) {
  return first.in_flight && second.in_flight && first.flags == second.flags && first.since_ms == second.since_ms;
}

OutStuck out_stuck_scan(const WatchedXfer *items, size_t count, uint32_t now_ms, uint32_t timeout_ms, size_t *index) {
  OutStuck found = OutStuck::kNone;
  uint32_t worst = 0;
  for (size_t i = 0; i < count; ++i) {
    const OutStuck r = xfer_stuck(items[i], now_ms, timeout_ms);
    if (r == OutStuck::kNone) continue;
    const uint32_t age = elapsed32(now_ms, items[i].since_ms);
    if (found == OutStuck::kNone || age > worst) {
      found = r;
      worst = age;
      if (index != nullptr) *index = i;
    }
  }
  return found;
}

StallAction stall_step(const WatchedXfer &x, bool soft_reported, uint32_t now_ms, uint32_t hard_ms, uint32_t soft_ms) {
  if (xfer_stuck(x, now_ms, hard_ms) != OutStuck::kNone) return StallAction::kHard;
  if (!x.in_flight || !x.intr_out || soft_reported) return StallAction::kNone;
  return elapsed32(now_ms, x.since_ms) > soft_ms ? StallAction::kSoft : StallAction::kNone;
}

bool stall_on_done(StallStats &stats, bool soft_reported, uint32_t since_ms, uint32_t done_ms, uint32_t *duration_ms,
                   uint32_t soft_ms) {
  if (!soft_reported) return false;
  const uint32_t duration = elapsed32(done_ms, since_ms);
  if (duration <= soft_ms) return false;
  ++stats.recovered;
  if (duration > stats.max_ms) stats.max_ms = duration;
  if (duration_ms != nullptr) *duration_ms = duration;
  return true;
}

const char *ep0_probe_str(Ep0Probe r) {
  switch (r) {
    case Ep0Probe::kOk:
      return "ok";
    case Ep0Probe::kError:
      return "err";
    case Ep0Probe::kTimeout:
      return "timeout";
    case Ep0Probe::kSkipped:
      return "skipped";
    default:
      return "not_sent";
  }
}

bool ep0_probe_wanted(uint8_t ep, bool reboot_follows) { return ep != 0 && reboot_follows; }

uint16_t out_stuck_pack(const OutStuckInfo &info) {
  const uint16_t pf = info.prefetch_in_flight > 15 ? 15 : info.prefetch_in_flight;
  return static_cast<uint16_t>(pf | (static_cast<uint16_t>(info.reason) << 4) |
                               (static_cast<uint16_t>(info.ep0) << 8));
}

bool out_stuck_unpack(uint16_t packed, OutStuckInfo *info) {
  const uint8_t reason = (packed >> 4) & 0x0F;
  const uint8_t ep0 = static_cast<uint8_t>(packed >> 8);
  if (reason != static_cast<uint8_t>(OutStuck::kAge) && reason != static_cast<uint8_t>(OutStuck::kOrphan)) return false;
  if (ep0 > static_cast<uint8_t>(Ep0Probe::kSkipped)) return false;
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

StuckStep stuck_step(StuckLatch &latch, RebootGuard &g, bool stuck) {
  if (!stuck) {
    latch.suppressed = false;
    return StuckStep::kIdle;
  }
  if (latch.suppressed) {
    guard_note_stop(g);
    return StuckStep::kQuiet;
  }
  if (guard_on_stuck(g) == GuardAction::kSuppress) {
    latch.suppressed = true;
    return StuckStep::kReportSuppressed;
  }
  return StuckStep::kReboot;
}

}  // namespace usbip
