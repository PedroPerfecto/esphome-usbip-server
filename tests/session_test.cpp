#include "doctest.h"
#include "../components/usbip_server/usbip_session.h"
#include "../components/usbip_server/usbip_device.h"
#include <cstring>

using namespace usbip;

namespace {
// busid as it arrives in OP_REQ_IMPORT: 32 B, null padded.
struct WireBusId {
  char s[kBusIdSize];
  explicit WireBusId(const char *text) {
    uint8_t raw[kBusIdSize] = {0};
    std::memcpy(raw, text, std::strlen(text));
    REQUIRE(decode_req_import_busid(raw, s));
  }
};
}  // namespace

// ---------------------------------------------------------------- takeover

TEST_CASE("takeover: import of the exported busid with a device present takes over") {
  CHECK(takeover_allowed(true, WireBusId("1-1").s, "1-1"));
  CHECK(takeover_allowed(true, WireBusId("usb-serial-0123456789abcdef0123").s, "usb-serial-0123456789abcdef0123"));  // 31 chars
}

TEST_CASE("takeover: a different busid must not interrupt the running session") {
  CHECK_FALSE(takeover_allowed(true, WireBusId("1-2").s, "1-1"));
  CHECK_FALSE(takeover_allowed(true, WireBusId("").s, "1-1"));
}

TEST_CASE("takeover: the match is exact, not a prefix in either direction") {
  CHECK_FALSE(takeover_allowed(true, WireBusId("1-1.2").s, "1-1"));
  CHECK_FALSE(takeover_allowed(true, WireBusId("1-1").s, "1-1.2"));
  CHECK_FALSE(takeover_allowed(true, WireBusId("1-").s, "1-1"));
  CHECK_FALSE(takeover_allowed(true, WireBusId("1-1 ").s, "1-1"));
}

TEST_CASE("takeover: no exported device -> refused even with the right busid") {
  CHECK_FALSE(takeover_allowed(false, WireBusId("1-1").s, "1-1"));
}

TEST_CASE("takeover: null pointers never match") {
  CHECK_FALSE(takeover_allowed(true, nullptr, "1-1"));
  CHECK_FALSE(takeover_allowed(true, WireBusId("1-1").s, nullptr));
}
