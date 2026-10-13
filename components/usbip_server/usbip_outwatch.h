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

// "OUT_STUCK ep=01 age=5012 ms reason=age ep0=timeout prefetch=2". Returns the length written
// (as snprintf, without the terminator), always terminated.
size_t out_stuck_format(const OutStuckInfo &info, char *out, size_t cap);

}  // namespace usbip
