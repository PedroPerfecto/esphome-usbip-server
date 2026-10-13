#pragma once
#include <cstdint>

// Decisions about a connection that arrives while a session is active (no ESPHome/ESP-IDF
// dependencies).
namespace usbip {

// OP_REQ_IMPORT on a side connection may take over the running session only if it would succeed:
// a device is exported and the requested busid is exactly the exported one. requested is the
// null-terminated busid from the request (decode_req_import_busid), exported the configured busid.
// A null pointer never matches.
bool takeover_allowed(bool device_ready, const char *requested, const char *exported);

// Receive timeout for one recv() of a request that must arrive completely before deadline_us
// (both times in microseconds of the same monotonic clock, e.g. esp_timer_get_time()).
// Returns false once the deadline is reached; otherwise *timeout_ms is the time left, rounded up
// to a whole millisecond and never 0: lwIP takes SO_RCVTIMEO in milliseconds and 0 means "wait
// forever". The result never exceeds the time left by a full millisecond.
bool recv_timeout_before(int64_t now_us, int64_t deadline_us, uint32_t *timeout_ms);

}  // namespace usbip
