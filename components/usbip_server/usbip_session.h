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

}  // namespace usbip
