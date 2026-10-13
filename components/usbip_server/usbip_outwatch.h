#pragma once
#include <cstddef>
#include <cstdint>

// Detection of transfers the USB stack never completes (no ESPHome/ESP-IDF dependencies). Such a
// transfer keeps its slot for good: an interrupt OUT cannot be cancelled (a periodic pipe is never
// halted, see usbip_intrq.h), a control transfer cannot be cancelled at all, and a bulk transfer
// may survive its cancel. Once all slots are taken every later transfer is refused. The server
// cannot recover the pipe, so it leaves a record and restarts.
namespace usbip {

static constexpr uint32_t kOutStuckTimeoutMs = 3000;  // default limit (YAML: out_stuck_timeout)

enum class OutStuck : uint8_t {
  kNone = 0,
  kAge = 1,     // interrupt OUT in flight longer than the limit
  kOrphan = 2,  // orphaned slot (answered with RET_UNLINK / session ended) still in flight after the limit
};

// One slot as the detector sees it. Times in ms, unsigned 32-bit; ages are taken with elapsed32
// (usbip_time.h): valid across a wrap, and a since_ms newer than "now" (the slot was reused by the
// other task after "now" was read) is an age of 0, never a huge one.
struct WatchedXfer {
  bool in_flight{false};  // submitted, completion callback not run yet (USB stack view)
  bool intr_out{false};   // interrupt OUT: completes within milliseconds on a healthy device
  bool orphan{false};     // the client was told it is over (RET_UNLINK after the wait) or is gone
  // Start of the time that is judged. Interrupt OUT: its submission, so an orphan of that kind is
  // measured by the age of the transfer. Control and bulk may legitimately be in flight for long
  // (bulk IN waits for data), so for them it is the moment the transfer was asked to end: CMD_UNLINK
  // received, or the session ended.
  uint32_t since_ms{0};
  uint8_t ep{0};          // endpoint address, 0 = control (ep0)
};

// Interrupt OUT: stuck once in flight longer than timeout_ms (kOrphan if it is an orphan by then,
// else kAge). Control and bulk: only as orphans, and only once since_ms is more than timeout_ms ago.
// Not in flight (the callback has run, the slot only waits to be released) is never stuck.
OutStuck xfer_stuck(const WatchedXfer &x, uint32_t now_ms, uint32_t timeout_ms = kOutStuckTimeoutMs);
// What the detector read about a slot. It is not an atomic snapshot (three separate atomics, two
// writer tasks), so a verdict is acted upon only if a second reading of the same slot is the same:
// still in flight, same flags, same time. A slot that completed, was reused or changed its orphan
// state in between is left for the next check.
struct WatchSample {
  bool in_flight{false};
  uint8_t flags{0};
  uint32_t since_ms{0};
};
bool watch_same(const WatchSample &first, const WatchSample &second);

// The transfer stuck for the longest time among items[0..count); kNone if there is none.
OutStuck out_stuck_scan(const WatchedXfer *items, size_t count, uint32_t now_ms, uint32_t timeout_ms, size_t *index);

// Soft stall: an interrupt OUT in flight longer than kStallSoftMs is reported and counted, but not
// acted upon. Only when it lasts out_stuck_timeout does the hard path (record, reboot) follow. With
// out_stuck_timeout above kStallSoftMs this measures how long stalls last and how many end by
// themselves; with out_stuck_timeout at or below it there is no soft stage.
static constexpr uint32_t kStallSoftMs = 3000;

enum class StallAction : uint8_t {
  kNone,
  kSoft,  // report the stall of this transfer now (once per transfer)
  kHard,  // stuck by xfer_stuck with hard_ms: the OUT_STUCK path
};
// soft_reported: kSoft was already returned for this transfer.
StallAction stall_step(const WatchedXfer &x, bool soft_reported, uint32_t now_ms, uint32_t hard_ms,
                       uint32_t soft_ms = kStallSoftMs);

struct StallStats {
  uint32_t soft{0};       // soft stalls reported
  uint32_t recovered{0};  // of those, transfers that completed afterwards
  uint32_t max_ms{0};     // longest recovered stall: submission to completion
};
// A transfer completed (done_ms: when its callback ran, since_ms: when it was submitted). Counts it
// as recovered if a soft stall had been reported for it. A flag set for a transfer that in fact
// took no longer than soft_ms (it can be a leftover of the slot's previous use) is ignored.
// Returns true if it was counted; *duration_ms then holds its length.
bool stall_on_done(StallStats &stats, bool soft_reported, uint32_t since_ms, uint32_t done_ms, uint32_t *duration_ms,
                   uint32_t soft_ms = kStallSoftMs);

// Result of the control transfer (GET_STATUS) sent to ep0 when a stuck transfer is detected.
enum class Ep0Probe : uint8_t {
  kNotSent = 0,  // no device, lock busy, allocation or submit failed
  kOk = 1,       // completed successfully within the limit
  kError = 2,    // completed with an error status within the limit
  kTimeout = 3,  // no completion within the limit
  kSkipped = 4,  // not tried: the stuck transfer is the one on ep0, or no reboot follows
};
const char *ep0_probe_str(Ep0Probe r);
// Whether to send the probe for a stuck transfer on endpoint ep (0 = control). Not if that transfer
// is the one on ep0 (the probe would only queue behind it), and not if no reboot follows: a probe
// that gets no answer cannot be cancelled and would stay on ep0, in front of the client's requests.
bool ep0_probe_wanted(uint8_t ep, bool reboot_follows);

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
