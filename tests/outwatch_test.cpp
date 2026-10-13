#include "doctest.h"
#include "../components/usbip_server/usbip_outwatch.h"
#include <cstring>
#include <string>

using namespace usbip;

namespace {
OutWatch in_flight_since(uint32_t submit_ms, uint32_t count = 1) {
  OutWatch w;
  w.in_flight = count;
  w.have_oldest = true;
  w.oldest_submit_ms = submit_ms;
  w.oldest_ep = 0x01;
  return w;
}
}  // namespace

// ---------------------------------------------------------------- decision

TEST_CASE("out stuck: nothing in flight is never stuck") {
  OutWatch w;
  CHECK(out_stuck_check(w, 0) == OutStuck::kNone);
  CHECK(out_stuck_check(w, 1000000) == OutStuck::kNone);
}

TEST_CASE("out stuck: a transfer in flight for more than 3 s is stuck") {
  const OutWatch w = in_flight_since(10000);
  CHECK(out_stuck_check(w, 10000) == OutStuck::kNone);
  CHECK(out_stuck_check(w, 10005) == OutStuck::kNone);  // a normal completion takes milliseconds
  CHECK(out_stuck_check(w, 12999) == OutStuck::kNone);
  CHECK(out_stuck_check(w, 13000) == OutStuck::kNone);  // exactly 3 s: not yet
  CHECK(out_stuck_check(w, 13001) == OutStuck::kAge);
  CHECK(out_stuck_check(w, 10000 + 13 * 3600 * 1000) == OutStuck::kAge);  // the 12.10. case: 13 hours
}

TEST_CASE("out stuck: the age is judged by the oldest transfer, also across a 32-bit wrap") {
  const OutWatch w = in_flight_since(0xFFFFF000u, 3);  // submitted 4096 ms before the wrap
  CHECK(out_stuck_check(w, 0xFFFFF800u) == OutStuck::kNone);  // 2048 ms
  CHECK(out_stuck_check(w, 0x00000000u) == OutStuck::kAge);   // 4096 ms
  CHECK(out_stuck_check(w, 0x00001000u) == OutStuck::kAge);
}

TEST_CASE("out stuck: completed in the USB stack but not released yet is not stuck") {
  // No session: the TCP task does not drain completions, so the slot (and its submit time) stays,
  // but the callback has run (in_flight 0).
  OutWatch w = in_flight_since(1000);
  w.in_flight = 0;
  w.have_orphan = true;
  w.orphan_since_ms = 2000;
  CHECK(out_stuck_check(w, 1000 + 3600 * 1000) == OutStuck::kNone);
}

TEST_CASE("out stuck: an orphaned slot still in flight is stuck after the grace time") {
  OutWatch w = in_flight_since(50000);
  w.have_orphan = true;
  w.orphan_since_ms = 50500;  // session ended 0.5 s after the submit
  w.orphan_ep = 0x01;
  CHECK(out_stuck_check(w, 50500) == OutStuck::kNone);    // may still complete right now
  CHECK(out_stuck_check(w, 51499) == OutStuck::kNone);
  CHECK(out_stuck_check(w, 51500) == OutStuck::kOrphan);  // 1 s as an orphan, 1.5 s in flight (< 3 s)
}

TEST_CASE("out stuck: orphan after the 5 s UNLINK wait is reported by age first") {
  OutWatch w = in_flight_since(1000);
  w.have_orphan = true;
  w.orphan_since_ms = 6000;
  CHECK(out_stuck_check(w, 6000) == OutStuck::kAge);
  CHECK(out_stuck_check(w, 7500) == OutStuck::kAge);
}

TEST_CASE("out stuck: orphan without a submit time on record still counts") {
  OutWatch w;
  w.in_flight = 1;
  w.have_orphan = true;
  w.orphan_since_ms = 100;
  CHECK(out_stuck_check(w, 1099) == OutStuck::kNone);
  CHECK(out_stuck_check(w, 1100) == OutStuck::kOrphan);
}

TEST_CASE("out stuck: limits are parameters") {
  const OutWatch w = in_flight_since(0);
  CHECK(out_stuck_check(w, 501, 500, 100) == OutStuck::kAge);
  CHECK(out_stuck_check(w, 500, 500, 100) == OutStuck::kNone);
}

// ---------------------------------------------------------------- record

TEST_CASE("out stuck record: pack/unpack keeps reason, ep0 result and prefetch count") {
  for (OutStuck reason : {OutStuck::kAge, OutStuck::kOrphan}) {
    for (Ep0Probe ep0 : {Ep0Probe::kNotSent, Ep0Probe::kOk, Ep0Probe::kError, Ep0Probe::kTimeout}) {
      for (uint8_t pf : {uint8_t{0}, uint8_t{2}, uint8_t{8}}) {
        OutStuckInfo in;
        in.reason = reason;
        in.ep0 = ep0;
        in.prefetch_in_flight = pf;
        OutStuckInfo out;
        REQUIRE(out_stuck_unpack(out_stuck_pack(in), &out));
        CHECK(out.reason == reason);
        CHECK(out.ep0 == ep0);
        CHECK(out.prefetch_in_flight == pf);
      }
    }
  }
}

TEST_CASE("out stuck record: prefetch count saturates, garbage is refused") {
  OutStuckInfo in;
  in.reason = OutStuck::kAge;
  in.prefetch_in_flight = 200;
  OutStuckInfo out;
  REQUIRE(out_stuck_unpack(out_stuck_pack(in), &out));
  CHECK(out.prefetch_in_flight == 15);

  CHECK_FALSE(out_stuck_unpack(0x0000, &out));  // reason none
  CHECK_FALSE(out_stuck_unpack(0x0030, &out));  // unknown reason
  CHECK_FALSE(out_stuck_unpack(0x0410, &out));  // unknown ep0 result
  CHECK_FALSE(out_stuck_unpack(0xFFFF, &out));  // RTC memory after power loss
}

TEST_CASE("out stuck record: text for the lock_diag sensor") {
  OutStuckInfo info;
  info.reason = OutStuck::kAge;
  info.ep = 0x01;
  info.age_ms = 3712;
  info.ep0 = Ep0Probe::kTimeout;
  info.prefetch_in_flight = 2;
  char text[96];
  const size_t n = out_stuck_format(info, text, sizeof(text));
  CHECK(std::string(text) == "OUT_STUCK ep=01 age=3712 ms reason=age ep0=timeout prefetch=2");
  CHECK(n == std::strlen(text));

  info.reason = OutStuck::kOrphan;
  info.ep0 = Ep0Probe::kOk;
  info.age_ms = 4294967295u;
  out_stuck_format(info, text, sizeof(text));
  CHECK(std::string(text) == "OUT_STUCK ep=01 age=4294967295 ms reason=orphan ep0=ok prefetch=2");

  char small[12];
  const size_t m = out_stuck_format(info, small, sizeof(small));  // truncated, still terminated
  CHECK(m == 11);
  CHECK(std::string(small) == "OUT_STUCK e");
}

TEST_CASE("ep0 probe result names") {
  CHECK(std::string(ep0_probe_str(Ep0Probe::kOk)) == "ok");
  CHECK(std::string(ep0_probe_str(Ep0Probe::kError)) == "err");
  CHECK(std::string(ep0_probe_str(Ep0Probe::kTimeout)) == "timeout");
  CHECK(std::string(ep0_probe_str(Ep0Probe::kNotSent)) == "not_sent");
}
