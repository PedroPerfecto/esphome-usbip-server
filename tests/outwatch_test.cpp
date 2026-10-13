#include "doctest.h"
#include "../components/usbip_server/usbip_outwatch.h"
#include <cstring>
#include <string>

using namespace usbip;

namespace {
WatchedXfer intr_out(uint32_t submit_ms) {
  WatchedXfer x;
  x.in_flight = true;
  x.intr_out = true;
  x.since_ms = submit_ms;
  x.ep = 0x01;
  return x;
}
WatchedXfer other(uint8_t ep, uint32_t since_ms, bool orphan) {
  WatchedXfer x;
  x.in_flight = true;
  x.orphan = orphan;
  x.since_ms = since_ms;
  x.ep = ep;
  return x;
}
}  // namespace

// ---------------------------------------------------------------- decision

TEST_CASE("out stuck: an interrupt OUT in flight for more than the limit is stuck") {
  const WatchedXfer x = intr_out(10000);
  CHECK(xfer_stuck(x, 10000) == OutStuck::kNone);
  CHECK(xfer_stuck(x, 10005) == OutStuck::kNone);  // a normal completion takes milliseconds
  CHECK(xfer_stuck(x, 13000) == OutStuck::kNone);  // exactly 3 s: not yet
  CHECK(xfer_stuck(x, 13001) == OutStuck::kAge);
  CHECK(xfer_stuck(x, 10000 + 13 * 3600 * 1000) == OutStuck::kAge);  // the 12.10. case: 13 hours
}

TEST_CASE("out stuck: across a 32-bit wrap of the clock") {
  const WatchedXfer x = intr_out(0xFFFFF000u);  // submitted 4096 ms before the wrap
  CHECK(xfer_stuck(x, 0xFFFFF800u) == OutStuck::kNone);  // 2048 ms
  CHECK(xfer_stuck(x, 0x00000000u) == OutStuck::kAge);   // 4096 ms
}

TEST_CASE("out stuck: completed in the USB stack but not released yet is not stuck") {
  // No session: the TCP task does not drain completions, the slot stays, but the callback has run.
  WatchedXfer x = intr_out(1000);
  x.orphan = true;
  x.in_flight = false;
  CHECK(xfer_stuck(x, 1000 + 3600 * 1000) == OutStuck::kNone);
}

TEST_CASE("out stuck: an orphaned interrupt OUT is judged by the age of the transfer") {
  // Session ended 2.9 s after the submit: one more check later it is over the limit, without any
  // extra time counted from the moment it became an orphan.
  WatchedXfer x = intr_out(50000);
  x.orphan = true;
  CHECK(xfer_stuck(x, 52900) == OutStuck::kNone);
  CHECK(xfer_stuck(x, 53001) == OutStuck::kOrphan);
  // And a session that ends right after a submit does not make a healthy transfer stuck.
  WatchedXfer fresh = intr_out(60000);
  fresh.orphan = true;
  CHECK(xfer_stuck(fresh, 60002) == OutStuck::kNone);
  CHECK(xfer_stuck(fresh, 61500) == OutStuck::kNone);
}

TEST_CASE("out stuck: control and bulk in flight are not stuck by age alone") {
  // Bulk IN waiting for data for an hour, a control transfer the client has not given up on.
  CHECK(xfer_stuck(other(0x82, 0, false), 3600 * 1000) == OutStuck::kNone);
  CHECK(xfer_stuck(other(0x00, 0, false), 60 * 1000) == OutStuck::kNone);
}

TEST_CASE("out stuck: control UNLINK without a completion -> orphan after the wait -> stuck") {
  // CMD_UNLINK at 20 000 (since_ms). The server waits 5 s for the completion, then answers
  // RET_UNLINK and orphans the slot: by then the limit counted from the UNLINK is long over.
  WatchedXfer ctrl = other(0x00, 20000, false);
  CHECK(xfer_stuck(ctrl, 24999) == OutStuck::kNone);  // still waiting for the completion
  ctrl.orphan = true;
  CHECK(xfer_stuck(ctrl, 25000) == OutStuck::kOrphan);
}

TEST_CASE("out stuck: bulk orphaned at the end of a session gets the limit to finish its cancel") {
  // In flight for an hour (idle bulk IN), session ends at 3 600 000: since_ms is the session end,
  // not the submit, so the cancel that is under way is not mistaken for a stuck transfer.
  WatchedXfer bulk = other(0x82, 3600000, true);
  CHECK(xfer_stuck(bulk, 3600000) == OutStuck::kNone);
  CHECK(xfer_stuck(bulk, 3600020) == OutStuck::kNone);  // cancelled completions arrive within ms
  CHECK(xfer_stuck(bulk, 3603000) == OutStuck::kNone);
  CHECK(xfer_stuck(bulk, 3603001) == OutStuck::kOrphan);
  bulk.in_flight = false;  // CANCELED callback has run
  CHECK(xfer_stuck(bulk, 3700000) == OutStuck::kNone);
}

TEST_CASE("out stuck: the limit is a parameter") {
  const WatchedXfer x = intr_out(0);
  CHECK(xfer_stuck(x, 501, 500) == OutStuck::kAge);
  CHECK(xfer_stuck(x, 500, 500) == OutStuck::kNone);
  CHECK(xfer_stuck(x, 5000, 10000) == OutStuck::kNone);
}

TEST_CASE("out stuck scan: picks the transfer stuck for the longest time") {
  WatchedXfer items[5];
  items[0] = other(0x82, 0, false);     // idle bulk IN: never
  items[1] = intr_out(9000);            // 1 s old at 10 000: not yet
  items[2] = intr_out(5000);            // 5 s
  items[3] = other(0x00, 2000, true);   // control orphan, 8 s since UNLINK
  items[4] = intr_out(1000);            // 9 s, but its callback has run
  items[4].in_flight = false;
  size_t index = 99;
  CHECK(out_stuck_scan(items, 5, 10000, 3000, &index) == OutStuck::kOrphan);
  CHECK(index == 3);
  items[3].in_flight = false;
  CHECK(out_stuck_scan(items, 5, 10000, 3000, &index) == OutStuck::kAge);
  CHECK(index == 2);
  items[2].in_flight = false;
  index = 99;
  CHECK(out_stuck_scan(items, 5, 10000, 3000, &index) == OutStuck::kNone);
  CHECK(index == 99);  // untouched
  CHECK(out_stuck_scan(items, 0, 10000, 3000, &index) == OutStuck::kNone);
}

// ---------------------------------------------------------------- record

TEST_CASE("out stuck record: pack/unpack keeps reason, ep0 result and prefetch count") {
  for (OutStuck reason : {OutStuck::kAge, OutStuck::kOrphan}) {
    for (Ep0Probe ep0 : {Ep0Probe::kNotSent, Ep0Probe::kOk, Ep0Probe::kError, Ep0Probe::kTimeout, Ep0Probe::kSkipped}) {
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
  CHECK_FALSE(out_stuck_unpack(0x0510, &out));  // unknown ep0 result
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
  CHECK(std::string(ep0_probe_str(Ep0Probe::kSkipped)) == "skipped");
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

TEST_CASE("ep0 probe: sent only for a stuck data endpoint when a reboot follows") {
  CHECK(ep0_probe_wanted(0x01, true));
  CHECK(ep0_probe_wanted(0x82, true));
  CHECK_FALSE(ep0_probe_wanted(0x00, true));   // the stuck transfer is the one on ep0
  CHECK_FALSE(ep0_probe_wanted(0x01, false));  // suppressed: an unanswered probe would stay on ep0 for good
  CHECK_FALSE(ep0_probe_wanted(0x00, false));
}

// ---------------------------------------------------------------- soft stall

TEST_CASE("stall: soft after 3 s, once, hard only at out_stuck_timeout") {
  const WatchedXfer x = intr_out(100000);
  const uint32_t hard = 60000;
  CHECK(stall_step(x, false, 100005, hard) == StallAction::kNone);
  CHECK(stall_step(x, false, 103000, hard) == StallAction::kNone);
  CHECK(stall_step(x, false, 103001, hard) == StallAction::kSoft);
  CHECK(stall_step(x, true, 104000, hard) == StallAction::kNone);   // already reported: not again
  CHECK(stall_step(x, true, 160000, hard) == StallAction::kNone);   // exactly 60 s
  CHECK(stall_step(x, true, 160001, hard) == StallAction::kHard);
  CHECK(stall_step(x, false, 160001, hard) == StallAction::kHard);  // hard wins even if soft was missed
}

TEST_CASE("stall: with out_stuck_timeout at 3 s there is no soft stage") {
  const WatchedXfer x = intr_out(0);
  CHECK(stall_step(x, false, 3000, 3000) == StallAction::kNone);
  CHECK(stall_step(x, false, 3001, 3000) == StallAction::kHard);
  CHECK(stall_step(x, false, 1500, 1000) == StallAction::kHard);  // hard limit below the soft one
}

TEST_CASE("stall: only interrupt OUT in flight is a soft stall") {
  CHECK(stall_step(other(0x82, 0, false), false, 3600000, 60000) == StallAction::kNone);  // idle bulk IN
  CHECK(stall_step(other(0x00, 0, false), false, 10000, 60000) == StallAction::kNone);    // control
  WatchedXfer done = intr_out(0);
  done.in_flight = false;  // callback has run
  CHECK(stall_step(done, false, 10000, 60000) == StallAction::kNone);
}

TEST_CASE("stall: an orphaned interrupt OUT is still soft first, hard by its age") {
  // UNLINK + 5 s wait made it an orphan at 8 s; the hard limit is 60 s from the submission.
  WatchedXfer x = intr_out(0);
  CHECK(stall_step(x, false, 3500, 60000) == StallAction::kSoft);
  x.orphan = true;
  CHECK(stall_step(x, true, 8000, 60000) == StallAction::kNone);
  CHECK(stall_step(x, true, 60001, 60000) == StallAction::kHard);
}

TEST_CASE("stall: control/bulk orphans go straight to hard (no soft stage for them)") {
  const WatchedXfer ctrl = other(0x00, 20000, true);
  CHECK(stall_step(ctrl, false, 25000, 60000) == StallAction::kNone);
  CHECK(stall_step(ctrl, false, 80001, 60000) == StallAction::kHard);
}

TEST_CASE("stall: a reported transfer that completes is counted with its length") {
  StallStats st;
  uint32_t dur = 0;
  CHECK(stall_on_done(st, true, 100000, 107250, &dur));  // completed 7.25 s after its submission
  CHECK(dur == 7250);
  CHECK(st.recovered == 1);
  CHECK(st.max_ms == 7250);
  CHECK(stall_on_done(st, true, 200000, 204100, &dur));
  CHECK(st.recovered == 2);
  CHECK(st.max_ms == 7250);  // the longest stays
  CHECK(stall_on_done(st, true, 300000, 341000, &dur));
  CHECK(st.max_ms == 41000);
  CHECK(st.recovered == 3);
}

TEST_CASE("stall: completions without a reported stall are not counted") {
  StallStats st;
  uint32_t dur = 77;
  CHECK_FALSE(stall_on_done(st, false, 1000, 1004, &dur));   // normal transfer
  CHECK_FALSE(stall_on_done(st, false, 1000, 9000, &dur));   // long, but ended between two checks: never reported
  CHECK_FALSE(stall_on_done(st, true, 5000, 5003, &dur));    // stale flag from the slot's previous use
  CHECK_FALSE(stall_on_done(st, true, 5000, 8000, &dur));    // exactly the soft limit
  CHECK(st.recovered == 0);
  CHECK(st.max_ms == 0);
  CHECK(dur == 77);
}

TEST_CASE("stall: length across a 32-bit wrap of the clock") {
  StallStats st;
  uint32_t dur = 0;
  CHECK(stall_on_done(st, true, 0xFFFFF000u, 0x00001000u, &dur));
  CHECK(dur == 0x2000);
}
