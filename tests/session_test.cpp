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

// ---------------------------------------------------------------- receive deadline

namespace {
// The receive loop of handle_side_connection_ with a client that delivers one byte each time just
// before the recv timeout runs out (the slowest client that never hits a timeout). Returns how long
// the loop is blocked in total (us) until `need` bytes arrived or the deadline closed the connection.
int64_t blocked_by_slow_client(int64_t limit_us, unsigned need, int64_t byte_delay_us, bool *complete) {
  int64_t now = 1000000;
  const int64_t start = now;
  const int64_t deadline = now + limit_us;
  unsigned have = 0;
  *complete = false;
  while (have < need) {
    uint32_t ms = 0;
    if (!recv_timeout_before(now, deadline, &ms)) return now - start;
    const int64_t timeout_us = static_cast<int64_t>(ms) * 1000;
    if (byte_delay_us >= timeout_us) return now + timeout_us - start;  // recv timed out
    now += byte_delay_us;
    ++have;
  }
  *complete = true;
  return now - start;
}
}  // namespace

TEST_CASE("recv deadline: time left is passed on, rounded up to a millisecond") {
  uint32_t ms = 0;
  CHECK(recv_timeout_before(0, 2000000, &ms));
  CHECK(ms == 2000);
  CHECK(recv_timeout_before(500000, 2000000, &ms));
  CHECK(ms == 1500);
  CHECK(recv_timeout_before(1999001, 2000000, &ms));
  CHECK(ms == 1);
}

TEST_CASE("recv deadline: less than 1 ms left is 1 ms, never 0 (0 = wait forever in lwIP)") {
  uint32_t ms = 0;
  CHECK(recv_timeout_before(1999999, 2000000, &ms));
  CHECK(ms == 1);
  CHECK(recv_timeout_before(1999500, 2000000, &ms));
  CHECK(ms == 1);
}

TEST_CASE("recv deadline: reached or passed -> no further recv") {
  uint32_t ms = 77;
  CHECK_FALSE(recv_timeout_before(2000000, 2000000, &ms));
  CHECK_FALSE(recv_timeout_before(2000001, 2000000, &ms));
  CHECK_FALSE(recv_timeout_before(INT64_MAX, 2000000, &ms));
  CHECK(ms == 77);  // untouched
}

TEST_CASE("recv deadline: a far deadline saturates instead of wrapping to a short timeout") {
  uint32_t ms = 0;
  CHECK(recv_timeout_before(0, INT64_MAX, &ms));
  CHECK(ms == UINT32_MAX);
}

TEST_CASE("recv deadline: a client trickling one byte per 999 ms is cut off after 2 s in total") {
  // OP_REQ_IMPORT is 8 + 32 = 40 B. With a fixed 1 s timeout per recv this client would hold the
  // TCP task for 40 * 0.999 s = 39.96 s and still be served.
  bool complete = true;
  const int64_t blocked = blocked_by_slow_client(2000000, 40, 999000, &complete);
  CHECK_FALSE(complete);
  CHECK(blocked <= 2000000 + 1000);  // the limit plus the round-up of the last timeout
}

TEST_CASE("recv deadline: a request that arrives in time is not affected") {
  bool complete = false;
  const int64_t blocked = blocked_by_slow_client(2000000, 40, 10000, &complete);  // 40 B in 0.4 s
  CHECK(complete);
  CHECK(blocked == 400000);
}

TEST_CASE("recv deadline: a silent client blocks for the limit, not longer") {
  bool complete = true;
  const int64_t blocked = blocked_by_slow_client(2000000, 40, INT64_MAX / 2, &complete);
  CHECK_FALSE(complete);
  CHECK(blocked == 2000000);
}
