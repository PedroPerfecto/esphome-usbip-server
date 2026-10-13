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

// ---------------------------------------------------------------- reboot loop guard

namespace {
// One boot: the device runs for run_before_stuck_s, then a stuck transfer is detected.
// Returns what the guard decides; on kReboot the next call is the next boot.
GuardAction boot_then_stuck(RebootGuard &g, uint32_t run_before_stuck_s) {
  guard_boot(g);
  for (uint32_t t = 0; t <= run_before_stuck_s; t += (run_before_stuck_s - t > 600 ? 600 : 1)) guard_tick(g, t);
  guard_tick(g, run_before_stuck_s);
  return guard_on_stuck(g);
}
// One boot without a stuck transfer (ended by an OTA, power stays on).
void boot_healthy(RebootGuard &g, uint32_t run_s) {
  guard_boot(g);
  for (uint32_t t = 0; t <= run_s; t += 60) guard_tick(g, t);
  guard_tick(g, run_s);
}
}  // namespace

TEST_CASE("guard: the first three stuck transfers reboot, the fourth within the hour does not") {
  RebootGuard g{};
  CHECK(boot_then_stuck(g, 30) == GuardAction::kReboot);
  CHECK(boot_then_stuck(g, 30) == GuardAction::kReboot);
  CHECK(boot_then_stuck(g, 30) == GuardAction::kReboot);
  CHECK(g.reboots == 3);
  CHECK(boot_then_stuck(g, 30) == GuardAction::kSuppress);
  CHECK(g.boots == 4);
  CHECK(g.run_s == 120);  // run time summed over the boots, although every uptime was only 30 s
  CHECK(guard_recent_reboots(g) == 3);
}

TEST_CASE("guard: uptime alone would never see the loop") {
  // Every boot gets stuck after 10 s of uptime: judged by uptime each one looks like the first.
  RebootGuard g{};
  int reboots = 0;
  for (int boot = 0; boot < 50; ++boot) {
    if (boot_then_stuck(g, 10) == GuardAction::kReboot) ++reboots;
  }
  CHECK(reboots == 3);
  CHECK(g.boots == 50);
}

TEST_CASE("guard: while suppressed and still stuck it stays suppressed, however long") {
  RebootGuard g{};
  for (int i = 0; i < 3; ++i) REQUIRE(boot_then_stuck(g, 30) == GuardAction::kReboot);
  guard_boot(g);
  guard_tick(g, 5);
  REQUIRE(guard_on_stuck(g) == GuardAction::kSuppress);
  for (uint32_t t = 6; t < 3 * 3600; ++t) {  // the server reports the stop every second
    guard_tick(g, t);
    guard_note_stop(g);
  }
  CHECK(g.reboots == 3);  // not forgotten: there was never an hour without a stop
}

TEST_CASE("guard: three reboots spread over more than an hour do not suppress") {
  RebootGuard g{};
  CHECK(boot_then_stuck(g, 1500) == GuardAction::kReboot);  // 25 min apart: never an hour without a stop
  CHECK(boot_then_stuck(g, 1500) == GuardAction::kReboot);
  CHECK(boot_then_stuck(g, 1500) == GuardAction::kReboot);  // at run time 4500 s
  // 4th at 6000 s: the first reboot (1500 s) is more than an hour back, only two lie in the window.
  CHECK(boot_then_stuck(g, 1500) == GuardAction::kReboot);
  CHECK(g.reboots == 3);  // the oldest was dropped
  CHECK(g.reboot_s[0] == 3000);
  CHECK(g.reboot_s[2] == 6000);
  // 5th only 100 s later (6100 s): 3000, 4500 and 6000 all lie within the hour before it.
  CHECK(boot_then_stuck(g, 100) == GuardAction::kSuppress);
}

TEST_CASE("guard: an hour of running without a stuck transfer forgets the reboots") {
  RebootGuard g{};
  for (int i = 0; i < 3; ++i) REQUIRE(boot_then_stuck(g, 30) == GuardAction::kReboot);
  guard_boot(g);
  guard_tick(g, 3599 - 0);  // 3599 s since the last stop (at run time 90)
  CHECK(g.reboots == 3);
  guard_tick(g, 3600);
  CHECK(g.reboots == 0);
  CHECK(guard_on_stuck(g) == GuardAction::kReboot);
  CHECK(g.reboots == 1);
}

TEST_CASE("guard: the healthy hour may be spread over several boots") {
  RebootGuard g{};
  for (int i = 0; i < 3; ++i) REQUIRE(boot_then_stuck(g, 30) == GuardAction::kReboot);
  boot_healthy(g, 1800);  // e.g. ended by an OTA
  CHECK(g.reboots == 3);
  CHECK(guard_recent_reboots(g) == 3);
  boot_healthy(g, 1800);
  CHECK(g.reboots == 0);
}

TEST_CASE("guard: boots that are not OUT_STUCK reboots do not count as such") {
  RebootGuard g{};
  for (int i = 0; i < 10; ++i) boot_healthy(g, 20);  // OTA, restart button, watchdog ...
  CHECK(g.boots == 10);
  CHECK(g.reboots == 0);
  CHECK(boot_then_stuck(g, 20) == GuardAction::kReboot);
}

TEST_CASE("guard: run time never goes backwards and saturates") {
  RebootGuard g{};
  guard_boot(g);
  guard_tick(g, 100);
  guard_tick(g, 40);  // a late tick with an older value
  CHECK(g.run_s == 100);
  g.run_s = UINT32_MAX - 10;
  guard_boot(g);
  guard_tick(g, 1000);
  CHECK(g.run_s == UINT32_MAX);
}

TEST_CASE("guard: validity check rejects memory that this code cannot have written") {
  RebootGuard g{};
  CHECK(guard_valid(g));
  REQUIRE(boot_then_stuck(g, 30) == GuardAction::kReboot);
  REQUIRE(boot_then_stuck(g, 30) == GuardAction::kReboot);
  CHECK(guard_valid(g));

  RebootGuard bad = g;
  bad.reboots = 4;
  CHECK_FALSE(guard_valid(bad));
  bad = g;
  bad.reboot_s[1] = g.run_s + 1;  // in the future
  CHECK_FALSE(guard_valid(bad));
  bad = g;
  bad.reboot_s[0] = bad.reboot_s[1] + 1;  // not ordered
  CHECK_FALSE(guard_valid(bad));
  bad = g;
  bad.base_s = g.run_s + 5;
  CHECK_FALSE(guard_valid(bad));
  std::memset(&bad, 0xFF, sizeof(bad));
  CHECK_FALSE(guard_valid(bad));

  guard_reset(bad);
  CHECK(guard_valid(bad));
  CHECK(bad.boots == 0);
}

TEST_CASE("out stuck record: suppressed text") {
  OutStuckInfo info;
  info.reason = OutStuck::kAge;
  info.ep = 0x01;
  info.age_ms = 3200;
  info.ep0 = Ep0Probe::kOk;
  info.prefetch_in_flight = 2;
  char text[128];
  out_stuck_format(info, text, sizeof(text), true);
  CHECK(std::string(text) == "OUT_STUCK_SUPPRESSED ep=01 age=3200 ms reason=age ep0=ok prefetch=2");
}

// ---------------------------------------------------------------- detector step (latch + guard)

namespace {
// Guard that has just seen kGuardMaxReboots reboots; the device is up for the given time.
RebootGuard guard_after_three_reboots(uint32_t uptime_s) {
  RebootGuard g{};
  for (int i = 0; i < 3; ++i) REQUIRE(boot_then_stuck(g, 30) == GuardAction::kReboot);
  guard_boot(g);
  guard_tick(g, uptime_s);
  return g;
}
}  // namespace

TEST_CASE("stuck step: nothing stuck is idle, a first stop reboots") {
  RebootGuard g{};
  StuckLatch latch;
  guard_boot(g);
  guard_tick(g, 10);
  CHECK(stuck_step(latch, g, false) == StuckStep::kIdle);
  CHECK(stuck_step(latch, g, true) == StuckStep::kReboot);
  CHECK(g.reboots == 1);
}

TEST_CASE("stuck step: a suppressed stop is reported once, then quiet while it lasts") {
  RebootGuard g = guard_after_three_reboots(5);
  StuckLatch latch;
  CHECK(stuck_step(latch, g, true) == StuckStep::kReportSuppressed);
  for (uint32_t t = 6; t < 60; ++t) {
    guard_tick(g, t);
    CHECK(stuck_step(latch, g, true) == StuckStep::kQuiet);
  }
  CHECK(g.reboots == 3);
}

TEST_CASE("stuck step: after the suppressed stop went away, a later stop is acted upon again") {
  // 12 Oct pattern after a harmless first stop: suppressed once (the transfer then completed), the
  // device runs on for two hours, then transfers get stuck for good. Without dropping the latch
  // that second stop would never be logged, recorded or answered with a reboot.
  RebootGuard g = guard_after_three_reboots(5);
  StuckLatch latch;
  REQUIRE(stuck_step(latch, g, true) == StuckStep::kReportSuppressed);
  guard_tick(g, 9);
  REQUIRE(stuck_step(latch, g, true) == StuckStep::kQuiet);
  for (uint32_t t = 10; t <= 10 + 2 * 3600; t += 10) {  // transfer completed: nothing stuck
    guard_tick(g, t);
    REQUIRE(stuck_step(latch, g, false) == StuckStep::kIdle);
  }
  CHECK(g.reboots == 0);  // an hour without a stop
  guard_tick(g, 10 + 2 * 3600 + 5);
  CHECK(stuck_step(latch, g, true) == StuckStep::kReboot);
}

TEST_CASE("stuck step: a new stop within the hour after a suppressed one is reported again, not rebooted") {
  RebootGuard g = guard_after_three_reboots(5);
  StuckLatch latch;
  REQUIRE(stuck_step(latch, g, true) == StuckStep::kReportSuppressed);
  guard_tick(g, 8);
  REQUIRE(stuck_step(latch, g, false) == StuckStep::kIdle);
  guard_tick(g, 300);
  CHECK(stuck_step(latch, g, true) == StuckStep::kReportSuppressed);  // reported again, guard still holds
}
