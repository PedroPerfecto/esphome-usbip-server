#pragma once
#include <cstddef>
#include <cstdint>

// Detection of interrupt OUT transfers the USB stack never completes (no ESPHome/ESP-IDF
// dependencies). Such a transfer keeps its slot for good (a periodic pipe is never halted, see
// usbip_intrq.h); once all slots are taken every later transfer is refused. The server cannot
// recover the pipe, so it leaves a record and restarts.
namespace usbip {

static constexpr uint32_t kOutStuckAgeMs = 3000;     // in flight this long without a completion
static constexpr uint32_t kOutOrphanGraceMs = 1000;  // orphaned slot still in flight this long

enum class OutStuck : uint8_t {
  kNone = 0,
  kAge = 1,     // the oldest transfer in flight is older than kOutStuckAgeMs
  kOrphan = 2,  // an orphaned slot (answered with RET_UNLINK / session ended) is still in flight
};

// Snapshot of the interrupt OUT transfers. Times in ms, unsigned 32-bit (differences stay valid
// across a wrap).
struct OutWatch {
  uint32_t in_flight{0};         // submitted, completion callback not run yet (USB stack view)
  bool have_oldest{false};       // a submitted transfer has not been released yet
  uint32_t oldest_submit_ms{0};  // when the oldest of them was submitted
  uint8_t oldest_ep{0};
  bool have_orphan{false};       // an orphaned slot has not been released yet
  uint32_t orphan_since_ms{0};   // when the oldest of them was orphaned
  uint8_t orphan_ep{0};
};

// in_flight == 0 means every callback has run (slots only wait to be released): never stuck.
// The orphan rule has a short grace so that a transfer completing right after the session ended
// is not reported. kAge wins if both apply.
OutStuck out_stuck_check(const OutWatch &w, uint32_t now_ms, uint32_t max_age_ms = kOutStuckAgeMs,
                         uint32_t orphan_grace_ms = kOutOrphanGraceMs);

// Result of the control transfer (GET_STATUS) sent to ep0 when a stuck transfer is detected.
enum class Ep0Probe : uint8_t {
  kNotSent = 0,  // no device, lock busy, allocation or submit failed
  kOk = 1,       // completed successfully within the limit
  kError = 2,    // completed with an error status within the limit
  kTimeout = 3,  // no completion within the limit
};
const char *ep0_probe_str(Ep0Probe r);

// What the record keeps about one detection.
struct OutStuckInfo {
  OutStuck reason{OutStuck::kNone};
  uint8_t ep{0};                  // endpoint address of the transfer that triggered it
  uint32_t age_ms{0};             // time since its submission
  Ep0Probe ep0{Ep0Probe::kNotSent};
  uint8_t prefetch_in_flight{0};  // interrupt IN prefetch transfers in flight (saturates at 15)
};

// reason, ep0 result and prefetch count share the 16-bit field of the RTC record:
// bits 0-3 prefetch_in_flight, bits 4-7 reason, bits 8-15 ep0 result.
uint16_t out_stuck_pack(const OutStuckInfo &info);
// Fills reason/ep0/prefetch_in_flight; false if the field holds values that are not valid.
bool out_stuck_unpack(uint16_t packed, OutStuckInfo *info);

// "OUT_STUCK ep=01 age=5012 ms reason=age ep0=timeout prefetch=2"; with suppressed the text starts
// with "OUT_STUCK_SUPPRESSED". Returns the length written (as snprintf, without the terminator),
// always terminated.
size_t out_stuck_format(const OutStuckInfo &info, char *out, size_t cap, bool suppressed = false);

// Protection against a reboot loop: if the transfer gets stuck again right after every restart,
// restarting does not help and only keeps the device (and its log) unavailable.
//
// There is no wall clock, and uptime starts from 0 at every boot, so time is the run time summed
// over all boots (run_s) and kept, with the rest of this state, in memory that survives a restart.
// Plain data without initializers (it lives in RTC_NOINIT memory): start from RebootGuard{} or
// guard_reset().
static constexpr uint8_t kGuardMaxReboots = 3;      // this many OUT_STUCK reboots ...
static constexpr uint32_t kGuardWindowS = 3600;     // ... within this much run time: no further one

struct RebootGuard {
  uint32_t boots;        // boots since the state was last reset (any reset reason)
  uint32_t run_s;        // run time summed over those boots [s]
  uint32_t base_s;       // run_s when this boot started
  uint32_t last_stop_s;  // run_s when a stuck transfer was last seen
  uint32_t reboot_s[kGuardMaxReboots];  // run_s of the last OUT_STUCK reboots, oldest first
  uint8_t reboots;       // valid entries in reboot_s
};

enum class GuardAction : uint8_t { kReboot, kSuppress };

void guard_reset(RebootGuard &g);
// False if the fields cannot have been written by this code (e.g. RTC memory after a power loss).
bool guard_valid(const RebootGuard &g);
// Once per boot, before the first guard_tick.
void guard_boot(RebootGuard &g);
// Periodically (every second): uptime_s of this boot. Forgets the recorded reboots after
// kGuardWindowS of run time without a stuck transfer.
void guard_tick(RebootGuard &g, uint32_t uptime_s);
// A stuck transfer is (still) there, nothing is decided: only keeps the "no stop for an hour"
// clock from running.
void guard_note_stop(RebootGuard &g);
// A stuck transfer was detected and a decision is needed. kReboot: the reboot is recorded, restart
// now. kSuppress: kGuardMaxReboots reboots already happened within the last kGuardWindowS.
GuardAction guard_on_stuck(RebootGuard &g);
// Recorded reboots that still lie within the window.
uint8_t guard_recent_reboots(const RebootGuard &g);

// One periodic check of the detector, combined with the guard. The latch (not kept across boots)
// makes a suppressed stop be reported once, not every second, and is dropped as soon as nothing is
// stuck any more, so that a later stop is judged afresh.
struct StuckLatch {
  bool suppressed{false};
};
enum class StuckStep : uint8_t {
  kIdle,              // nothing stuck
  kReboot,            // stuck: record it and restart now
  kReportSuppressed,  // stuck, but the guard forbids a restart: report it (once)
  kQuiet,             // still the suppressed stop that was already reported
};
StuckStep stuck_step(StuckLatch &latch, RebootGuard &g, bool stuck);

}  // namespace usbip
