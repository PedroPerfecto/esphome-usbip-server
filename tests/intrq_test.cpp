#include "doctest.h"
#include "../components/usbip_server/usbip_intrq.h"
#include <cstring>
#include <vector>

using namespace usbip;

namespace {
// Packet of len bytes filled with value v.
std::vector<uint8_t> pkt(uint16_t len, uint8_t v) { return std::vector<uint8_t>(len, v); }
bool push(IntrRing &r, uint16_t len, uint8_t v) {
  auto p = pkt(len, v);
  return r.push(p.data(), len);
}
bool push(IntrInQueue &q, uint16_t len, uint8_t v) {
  auto p = pkt(len, v);
  return q.on_packet(p.data(), len);
}
}  // namespace

// ---------------------------------------------------------------- ring

TEST_CASE("ring: default depth, push/pop order, empty/full") {
  IntrRing r;
  REQUIRE(r.init(IntrRing::kDefaultDepth, 64));
  CHECK(r.depth() == 32);
  CHECK(r.empty());
  CHECK_FALSE(r.pop());
  for (int i = 0; i < 32; ++i) CHECK(push(r, 64, static_cast<uint8_t>(i)));
  CHECK(r.full());
  CHECK_FALSE(push(r, 64, 0xEE));  // full: dropped, ring unchanged
  CHECK(r.size() == 32);
  CHECK(r.data_at(0)[0] == 0);
  CHECK(r.pop());
  CHECK(r.data_at(0)[0] == 1);
  CHECK(push(r, 10, 0xAA));  // wraps around
  CHECK(r.len_at(31) == 10);
  CHECK(r.data_at(31)[9] == 0xAA);
}

TEST_CASE("ring: packet larger than MPS is refused, zero-length packet is stored") {
  IntrRing r;
  REQUIRE(r.init(4, 64));
  CHECK_FALSE(push(r, 65, 1));
  CHECK(r.empty());
  CHECK(r.push(nullptr, 0));
  CHECK(r.size() == 1);
  CHECK(r.len_at(0) == 0);
}

TEST_CASE("ring: depth is clamped to 4..4096, boundary depths work") {
  IntrRing r;
  REQUIRE(r.init(1, 64));
  CHECK(r.depth() == 4);
  REQUIRE(r.init(5000, 64));
  CHECK(r.depth() == 4096);

  for (uint16_t depth : {uint16_t{4}, uint16_t{256}, uint16_t{4096}}) {
    CAPTURE(depth);
    REQUIRE(r.init(depth, 64));
    CHECK(r.depth() == depth);
    for (uint16_t i = 0; i < depth; ++i) CHECK(push(r, 64, static_cast<uint8_t>(i)));
    CHECK(r.full());
    CHECK_FALSE(push(r, 64, 0));
    // drain and refill once more across the wrap
    for (uint16_t i = 0; i < depth; ++i) {
      CHECK(r.data_at(0)[0] == static_cast<uint8_t>(i));
      CHECK(r.pop());
    }
    CHECK(r.empty());
    for (uint16_t i = 0; i < depth; ++i) CHECK(push(r, 1, static_cast<uint8_t>(i + 1)));
    CHECK(r.full());
    CHECK(r.data_at(static_cast<uint16_t>(depth - 1))[0] == static_cast<uint8_t>(depth));
  }
}

TEST_CASE("ring: mps 0 is refused") {
  IntrRing r;
  CHECK_FALSE(r.init(32, 0));
  CHECK_FALSE(r.initialized());
  CHECK_FALSE(push(r, 0, 0));
}

TEST_CASE("ring: peak depth") {
  IntrRing r;
  REQUIRE(r.init(8, 64));
  push(r, 64, 1);
  push(r, 64, 2);
  push(r, 64, 3);
  r.pop();
  r.pop();
  CHECK(r.peak() == 3);
  CHECK(r.take_peak() == 3);
  CHECK(r.peak() == 1);  // restarts from the current size
  push(r, 64, 4);
  CHECK(r.take_peak() == 2);
}

// ---------------------------------------------------------------- waiting FIFO

TEST_CASE("fifo: order, remove by seqnum (UNLINK), unknown seqnum, capacity") {
  IntrWaitFifo f;
  CHECK(f.push(10, 64));
  CHECK(f.push(11, 64));
  CHECK(f.push(12, 64));
  CHECK(f.remove(11));
  CHECK_FALSE(f.contains(11));
  CHECK(f.front().seqnum == 10);
  f.pop_front();
  CHECK(f.front().seqnum == 12);
  CHECK_FALSE(f.remove(99));  // not waiting
  CHECK(f.size() == 1);
  f.clear();
  for (uint32_t i = 0; i < IntrWaitFifo::kCapacity; ++i) CHECK(f.push(i, 64));
  CHECK(f.full());
  CHECK_FALSE(f.push(1000, 64));
}

// ---------------------------------------------------------------- serving

TEST_CASE("serve: request = MPS, one full packet") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 64, 0x11);
  push(r, 64, 0x22);
  IntrServePlan p = plan_serve(r, 64);
  CHECK(p.ready);
  CHECK(p.filled);
  CHECK_FALSE(p.short_packet);
  CHECK(p.length == 64);
  CHECK(p.packets == 1);
  uint8_t out[64];
  consume(r, p, out);
  CHECK(out[0] == 0x11);
  CHECK(out[63] == 0x11);
  CHECK(r.size() == 1);  // the second packet stays for the next request
}

TEST_CASE("serve: short packet ends the request") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 64, 0x11);
  push(r, 20, 0x22);
  push(r, 64, 0x33);
  IntrServePlan p = plan_serve(r, 256);
  CHECK(p.ready);
  CHECK(p.short_packet);
  CHECK(p.length == 84);
  CHECK(p.packets == 2);
  std::vector<uint8_t> out(p.length);
  consume(r, p, out.data());
  CHECK(out[63] == 0x11);
  CHECK(out[64] == 0x22);
  CHECK(out[83] == 0x22);
  CHECK(r.size() == 1);
  CHECK(r.data_at(0)[0] == 0x33);
}

TEST_CASE("serve: zero-length packet is a short packet") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 64, 0x11);
  r.push(nullptr, 0);
  IntrServePlan p = plan_serve(r, 256);
  CHECK(p.ready);
  CHECK(p.short_packet);
  CHECK(p.length == 64);
  CHECK(p.packets == 2);

  IntrRing r2;
  REQUIRE(r2.init(32, 64));
  r2.push(nullptr, 0);
  IntrServePlan p2 = plan_serve(r2, 64);
  CHECK(p2.ready);
  CHECK(p2.short_packet);
  CHECK(p2.length == 0);
  CHECK(p2.packets == 1);
}

TEST_CASE("serve: several full packets fill the request exactly") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 64, 1);
  push(r, 64, 2);
  push(r, 64, 3);
  IntrServePlan p = plan_serve(r, 128);
  CHECK(p.ready);
  CHECK(p.filled);
  CHECK(p.length == 128);
  CHECK(p.packets == 2);
}

TEST_CASE("serve: only full packets and room left -> not ready (a real URB keeps waiting)") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 64, 1);
  IntrServePlan p = plan_serve(r, 128);
  CHECK_FALSE(p.ready);
  push(r, 5, 2);  // short packet arrives later
  p = plan_serve(r, 128);
  CHECK(p.ready);
  CHECK(p.length == 69);
}

TEST_CASE("serve: request shorter than the packet -> overflow, rest of the packet discarded") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 64, 0x11);
  push(r, 64, 0x22);
  IntrServePlan p = plan_serve(r, 8);
  CHECK(p.ready);
  CHECK(p.overflow);
  CHECK(p.length == 8);
  CHECK(p.packets == 1);
  uint8_t out[8];
  consume(r, p, out);
  CHECK(out[7] == 0x11);
  CHECK(r.size() == 1);
  CHECK(r.data_at(0)[0] == 0x22);  // next request starts with the next packet
}

TEST_CASE("serve: request shorter than a short packet -> overflow too") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 20, 0x11);
  IntrServePlan p = plan_serve(r, 10);
  CHECK(p.ready);
  CHECK(p.overflow);
  CHECK_FALSE(p.short_packet);
  CHECK(p.length == 10);
}

TEST_CASE("serve: request not a multiple of MPS, data ends exactly on it with a short packet") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  push(r, 64, 1);
  push(r, 36, 2);
  IntrServePlan p = plan_serve(r, 100);
  CHECK(p.ready);
  CHECK(p.short_packet);
  CHECK_FALSE(p.overflow);
  CHECK(p.length == 100);
}

TEST_CASE("serve: request larger than a full ring of full packets -> ring_limit") {
  IntrRing r;
  REQUIRE(r.init(4, 64));
  for (int i = 0; i < 4; ++i) push(r, 64, static_cast<uint8_t>(i));
  IntrServePlan p = plan_serve(r, 1024);
  CHECK(p.ready);
  CHECK(p.ring_limit);
  CHECK(p.length == 256);
  CHECK(p.packets == 4);
}

TEST_CASE("serve: empty ring -> not ready; zero-length request -> ready at once") {
  IntrRing r;
  REQUIRE(r.init(32, 64));
  CHECK_FALSE(plan_serve(r, 64).ready);
  IntrServePlan p = plan_serve(r, 0);
  CHECK(p.ready);
  CHECK(p.zero_request);
  CHECK(p.packets == 0);
}

// ---------------------------------------------------------------- queue

TEST_CASE("queue: empty ring with a waiting request, then a packet arrives") {
  IntrInQueue q;
  REQUIRE(q.init(32, 64));
  CHECK(q.add_request(100, 64, 0));
  uint32_t seq = 0;
  IntrServePlan p;
  CHECK_FALSE(q.front_ready(&seq, &p));
  CHECK(push(q, 64, 0x5A));
  REQUIRE(q.front_ready(&seq, &p));
  CHECK(seq == 100);
  uint8_t out[64];
  q.complete_front(p, out);
  CHECK(out[0] == 0x5A);
  CHECK(q.waiting().empty());
  CHECK(q.ring().empty());
}

TEST_CASE("queue: requests are served in order") {
  IntrInQueue q;
  REQUIRE(q.init(32, 64));
  push(q, 64, 1);
  push(q, 64, 2);
  q.add_request(7, 64, 0);
  q.add_request(8, 64, 0);
  uint32_t seq = 0;
  IntrServePlan p;
  uint8_t out[64];
  REQUIRE(q.front_ready(&seq, &p));
  CHECK(seq == 7);
  q.complete_front(p, out);
  CHECK(out[0] == 1);
  REQUIRE(q.front_ready(&seq, &p));
  CHECK(seq == 8);
  q.complete_front(p, out);
  CHECK(out[0] == 2);
  CHECK_FALSE(q.front_ready(&seq, &p));
}

TEST_CASE("queue: UNLINK of a waiting request and of an unknown seqnum") {
  IntrInQueue q;
  REQUIRE(q.init(32, 64));
  q.add_request(1, 64, 0);
  q.add_request(2, 64, 0);
  CHECK(q.unlink(1));
  CHECK_FALSE(q.unlink(1));   // already removed
  CHECK_FALSE(q.unlink(42));  // never waiting (answered or unknown) -> caller replies RET_UNLINK 0
  push(q, 64, 9);
  uint32_t seq = 0;
  IntrServePlan p;
  REQUIRE(q.front_ready(&seq, &p));
  CHECK(seq == 2);
}

TEST_CASE("queue: full ring drops new packets") {
  IntrInQueue q;
  REQUIRE(q.init(4, 64));
  for (int i = 0; i < 4; ++i) CHECK(push(q, 64, static_cast<uint8_t>(i)));
  CHECK_FALSE(push(q, 64, 0xFF));
  CHECK(q.ring().size() == 4);
}

TEST_CASE("queue: reset on a new import") {
  IntrInQueue q;
  REQUIRE(q.init(32, 64));
  push(q, 64, 1);
  push(q, 64, 2);
  q.add_request(5, 64, 1000);
  q.add_request(6, 64, 1000);
  q.reset();
  CHECK(q.ring().empty());
  CHECK(q.waiting().empty());
  CHECK(q.ring().depth() == 32);  // storage kept
  CHECK_FALSE(q.should_prefetch(true, 1001));  // last request time forgotten
  CHECK(push(q, 64, 3));
}

// ---------------------------------------------------------------- prefetch decision

TEST_CASE("prefetch_allowed: truth table") {
  // session inactive or ring full -> never
  CHECK_FALSE(prefetch_allowed(false, false, true, true, 0, 0));
  CHECK_FALSE(prefetch_allowed(true, true, true, true, 0, 0));
  // waiting request -> yes, regardless of time
  CHECK(prefetch_allowed(true, false, true, false, 0, 123456));
  // no waiting request: within the window
  CHECK(prefetch_allowed(true, false, false, true, 1000, 1000));
  CHECK(prefetch_allowed(true, false, false, true, 1000, 1049));
  CHECK_FALSE(prefetch_allowed(true, false, false, true, 1000, 1050));  // 50 ms is no longer < 50
  CHECK_FALSE(prefetch_allowed(true, false, false, true, 1000, 5000));
  // no request since the import
  CHECK_FALSE(prefetch_allowed(true, false, false, false, 0, 0));
  // custom window
  CHECK(prefetch_allowed(true, false, false, true, 1000, 1099, 100));
  CHECK_FALSE(prefetch_allowed(true, false, false, true, 1000, 1100, 100));
}

TEST_CASE("prefetch_allowed: across a 32-bit ms wrap") {
  const uint32_t last = 0xFFFFFFF0u;
  CHECK(prefetch_allowed(true, false, false, true, last, 0x00000010u));         // 32 ms later
  CHECK_FALSE(prefetch_allowed(true, false, false, true, last, 0x00000030u));   // 64 ms later
}

TEST_CASE("queue: should_prefetch follows requests, window and full ring") {
  IntrInQueue q;
  REQUIRE(q.init(4, 64));
  CHECK_FALSE(q.should_prefetch(true, 0));  // nothing requested yet
  q.add_request(1, 64, 100);
  CHECK(q.should_prefetch(true, 10000));    // waiting request: no time limit
  CHECK_FALSE(q.should_prefetch(false, 100));  // no session
  push(q, 64, 1);
  uint32_t seq;
  IntrServePlan p;
  uint8_t out[64];
  REQUIRE(q.front_ready(&seq, &p));
  q.complete_front(p, out);
  CHECK(q.should_prefetch(true, 149));       // answered, last request 49 ms ago
  CHECK_FALSE(q.should_prefetch(true, 150));  // window over
  q.add_request(2, 64, 200);
  for (int i = 0; i < 4; ++i) push(q, 1, 0);  // ring full (short packets)
  CHECK_FALSE(q.should_prefetch(true, 200));  // full ring wins over a waiting request
}

// ---------------------------------------------------------------- helpers used by the server (I2)

TEST_CASE("queue: start_frame/number_of_packets are kept with the request") {
  IntrInQueue q;
  REQUIRE(q.init(32, 64));
  CHECK(q.add_request(1, 64, 0, 7, 0xFFFFFFFFu));
  CHECK(q.waiting().front().start_frame == 7);
  CHECK(q.waiting().front().number_of_packets == 0xFFFFFFFFu);
}

TEST_CASE("queue: pop_request fails requests in order, clear_waiting keeps packets") {
  IntrInQueue q;
  REQUIRE(q.init(32, 64));
  q.add_request(1, 64, 0);
  q.add_request(2, 64, 0);
  push(q, 64, 1);
  IntrRequest r;
  CHECK(q.pop_request(&r));
  CHECK(r.seqnum == 1);
  q.clear_waiting();
  CHECK(q.waiting().empty());
  CHECK_FALSE(q.pop_request(&r));
  CHECK(q.ring().size() == 1);  // packets stay until the next import
}

TEST_CASE("queue: request_active ignores the ring, should_prefetch reserves in-flight entries") {
  IntrInQueue q;
  REQUIRE(q.init(4, 64));
  CHECK_FALSE(q.request_active(0));
  q.add_request(1, 64, 100);
  for (int i = 0; i < 4; ++i) push(q, 1, 0);
  CHECK(q.request_active(100));  // ring full does not matter here
  q.reset();
  q.add_request(1, 64, 100);
  push(q, 64, 0);
  push(q, 64, 0);
  CHECK(q.should_prefetch(true, 100, kDefaultPrefetchWindowMs, 1));        // 2 + 1 < 4
  CHECK_FALSE(q.should_prefetch(true, 100, kDefaultPrefetchWindowMs, 2));  // 2 + 2 >= 4
}

TEST_CASE("ring: custom allocator is used for both buffers and released on re-init") {
  static int allocs = 0, frees = 0;
  static size_t bytes = 0;
  allocs = frees = 0;
  bytes = 0;
  IntrAllocator a{[](size_t n) -> void * { ++allocs; bytes += n; return ::operator new(n); },
                  [](void *p) { ++frees; ::operator delete(p); }};
  {
    IntrRing r;
    REQUIRE(r.init(4096, 64, a));
    CHECK(allocs == 2);
    CHECK(bytes == 4096u * 64u + 4096u * sizeof(uint16_t));
    for (int i = 0; i < 4096; ++i) CHECK(push(r, 64, static_cast<uint8_t>(i)));
    CHECK(r.full());
    REQUIRE(r.init(32, 64, a));  // re-init frees the previous storage
    CHECK(frees == 2);
    CHECK(allocs == 4);
  }
  CHECK(frees == 4);  // destructor
}

TEST_CASE("ring: failing allocator -> init false, nothing leaks") {
  static int frees = 0;
  frees = 0;
  IntrAllocator a{[](size_t) -> void * { return nullptr; }, [](void *p) { if (p) ++frees; }};
  IntrRing r;
  CHECK_FALSE(r.init(4096, 64, a));
  CHECK_FALSE(r.initialized());
  CHECK(frees == 0);
}

// ---------------------------------------------------------------- zero-length packet

TEST_CASE("queue: a request longer than MPS is ended by a zero-length packet") {
  // Report of exactly 64 B (= MPS) read with a 128 B request: the device marks the end with a
  // zero-length packet. Without storing it the request would wait for a second data packet.
  IntrInQueue q;
  REQUIRE(q.init(8, 64));
  REQUIRE(q.add_request(7, 128, 0));
  const auto full = pkt(64, 0x5A);
  REQUIRE(q.on_completion(full.data(), 64));
  uint32_t seq = 0;
  IntrServePlan plan;
  CHECK_FALSE(q.front_ready(&seq, &plan));  // full packet, room left: keeps waiting

  CHECK(q.on_completion(full.data(), 0));  // zero-length packet (the buffer content is irrelevant)
  CHECK(q.ring().size() == 2);
  CHECK(q.ring().len_at(1) == 0);
  REQUIRE(q.front_ready(&seq, &plan));
  CHECK(seq == 7);
  CHECK(plan.short_packet);
  CHECK(plan.length == 64);
  CHECK(plan.packets == 2);  // the zero-length packet is consumed with it
  uint8_t out[128] = {0};
  q.complete_front(plan, out);
  CHECK(out[63] == 0x5A);
  CHECK(q.ring().empty());
  CHECK(q.waiting().empty());
}

TEST_CASE("queue: a zero-length packet alone completes a request with 0 bytes") {
  IntrInQueue q;
  REQUIRE(q.init(8, 64));
  REQUIRE(q.add_request(1, 64, 0));
  CHECK(q.on_completion(nullptr, 0));
  uint32_t seq = 0;
  IntrServePlan plan;
  REQUIRE(q.front_ready(&seq, &plan));
  CHECK(plan.short_packet);
  CHECK(plan.length == 0);
  CHECK(plan.packets == 1);
}

TEST_CASE("queue: on_completion cuts to MPS and reports a full ring") {
  IntrInQueue q;
  REQUIRE(q.init(4, 8));
  const auto big = pkt(16, 0x11);
  CHECK(q.on_completion(big.data(), 16));
  CHECK(q.ring().len_at(0) == 8);
  CHECK(q.on_completion(big.data(), 3));
  CHECK(q.ring().len_at(1) == 3);
  CHECK(q.on_completion(nullptr, -1));  // treated as a zero-length packet
  CHECK(q.ring().len_at(2) == 0);
  CHECK(q.on_completion(big.data(), 8));
  CHECK_FALSE(q.on_completion(nullptr, 0));  // full: also a zero-length packet needs an entry
}

// ---------------------------------------------------------------- time arithmetic

TEST_CASE("prefetch window: a last request newer than 'now' counts as just arrived") {
  // 'now' was read before the request that set last_request_ms: the window is open, not 49 days old.
  CHECK(prefetch_allowed(true, false, false, true, 1001, 1000));
  CHECK(prefetch_allowed(true, false, false, true, 1040, 1000));
  CHECK(prefetch_allowed(true, false, false, true, 0x00000000u, 0xFFFFFFFFu));  // across the wrap
  CHECK_FALSE(prefetch_allowed(true, false, false, true, 1000, 1050));           // 50 ms ago: closed
}

TEST_CASE("prefetch window: an idle queue never looks active again, however long the session lasts") {
  // Last request at t0, then nothing for weeks while the session stays open. The stored time must
  // not look recent again at 2^31 ms (24.8 days) nor when the clock comes back around (49.7 days).
  IntrInQueue q;
  REQUIRE(q.init(8, 64));
  const uint32_t t0 = 123456;
  REQUIRE(q.add_request(1, 64, t0));
  const auto p = pkt(64, 1);
  REQUIRE(q.on_completion(p.data(), 64));
  uint32_t seq = 0;
  IntrServePlan plan;
  REQUIRE(q.front_ready(&seq, &plan));
  uint8_t out[64];
  q.complete_front(plan, out);
  CHECK(q.should_prefetch(true, t0 + 10));  // within the window: the client is reading

  // The server calls expire_window on every pass of its loop (every few ms).
  for (uint32_t dt : {uint32_t{49}, uint32_t{50}, uint32_t{51}, uint32_t{1000}}) q.expire_window(t0 + dt);
  CHECK_FALSE(q.should_prefetch(true, t0 + 51));
  for (uint32_t dt : {0x7FFFFFFFu, 0x80000000u, 0x80000001u, 0xFFFF0000u, 0xFFFFFFF0u, 0xFFFFFFFFu, 0x00000000u,
                      0x00000010u}) {
    CAPTURE(dt);
    q.expire_window(t0 + dt);
    CHECK_FALSE(q.should_prefetch(true, t0 + dt));
    CHECK_FALSE(q.request_active(t0 + dt));
  }
  // A new request opens the window again.
  REQUIRE(q.add_request(2, 64, t0 + 0x00000020u));
  CHECK(q.should_prefetch(true, t0 + 0x00000020u));
}

TEST_CASE("prefetch window: expire_window keeps a running window and a just-arrived request") {
  IntrInQueue q;
  REQUIRE(q.init(8, 64));
  REQUIRE(q.add_request(1, 64, 1000));
  q.expire_window(1049);
  CHECK(q.request_active(1049));
  q.expire_window(999);  // 'now' read before the request arrived: not expired
  REQUIRE(q.unlink(1));
  CHECK(q.request_active(1049));  // nothing waiting, window still open
  q.expire_window(1050);
  CHECK_FALSE(q.request_active(1050));
  CHECK_FALSE(q.request_active(1010));  // forgotten for good, also for an older 'now'
}

TEST_CASE("prefetch window: without expiry the raw comparison holds up to a full turn of the clock") {
  // prefetch_allowed alone (no expire_window): closed at 2^31 ms, as with a plain unsigned difference.
  CHECK_FALSE(prefetch_allowed(true, false, false, true, 1000, 1000 + 0x80000000u));
  CHECK_FALSE(prefetch_allowed(true, false, false, true, 1000, 1000 + 0xF0000000u));
}
