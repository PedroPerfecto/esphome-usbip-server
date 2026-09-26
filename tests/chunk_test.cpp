#include "doctest.h"
#include "../components/usbip_server/usbip_chunk.h"

using namespace usbip;

TEST_CASE("bulk IN 118784 B (measured from Linux usb-storage) in 16 KB chunks") {
  ChunkPlan p(118784, 16384, 64, true, false);
  uint32_t chunks = 0;
  uint32_t last_req = 0;
  while (!p.finished()) {
    last_req = p.request_len();
    CHECK(p.submit_len() % 64 == 0);
    CHECK(p.offset() == chunks * 16384);
    CHECK(p.complete(p.submit_len()) == last_req);  // the device sends the whole chunk
    ++chunks;
  }
  CHECK(chunks == 8);  // 7 x 16384 + 4096
  CHECK(last_req == 4096);
  CHECK(p.done() == 118784);
}

TEST_CASE("short packet in the middle ends the transfer") {
  ChunkPlan p(65536, 16384, 64, true, false);
  CHECK(p.complete(16384) == 16384);
  CHECK_FALSE(p.finished());
  CHECK(p.complete(1000) == 1000);  // short packet
  CHECK(p.finished());
  CHECK(p.done() == 17384);
}

TEST_CASE("short packet of zero length at chunk boundary ends the transfer") {
  ChunkPlan p(65536, 16384, 64, true, false);
  p.complete(16384);
  CHECK(p.complete(0) == 0);
  CHECK(p.finished());
  CHECK(p.done() == 16384);
}

TEST_CASE("IN last chunk rounded up to MPS, extra bytes are not accepted") {
  ChunkPlan p(16384 + 13, 16384, 64, true, false);
  p.complete(16384);
  CHECK(p.request_len() == 13);
  CHECK(p.submit_len() == 64);
  CHECK(p.complete(64) == 13);  // the device sent more, we take only what was requested
  CHECK(p.finished());
  CHECK(p.done() == 16397);
}

TEST_CASE("OUT: zero packet only on the last chunk and only if requested") {
  ChunkPlan p(40960, 16384, 64, false, true);
  CHECK_FALSE(p.zero_pack());
  CHECK(p.submit_len() == 16384);
  p.complete(16384);
  CHECK_FALSE(p.zero_pack());
  p.complete(16384);
  CHECK(p.request_len() == 8192);
  CHECK(p.zero_pack());
  p.complete(8192);
  CHECK(p.finished());

  ChunkPlan q(40960, 16384, 64, false, false);
  q.complete(16384);
  q.complete(16384);
  CHECK_FALSE(q.zero_pack());
}

TEST_CASE("OUT short completion is treated as end (error)") {
  ChunkPlan p(40960, 16384, 64, false, false);
  CHECK(p.complete(1000) == 1000);
  CHECK(p.finished());
}

TEST_CASE("cap is aligned down to MPS, at least one MPS") {
  ChunkPlan a(100000, 16010, 64, true, false);
  CHECK(a.cap() == 16000);
  ChunkPlan b(100000, 10, 64, true, false);
  CHECK(b.cap() == 64);
  ChunkPlan c(100000, 16384, 512, true, false);
  CHECK(c.cap() == 16384);
}

TEST_CASE("zero-length transfer is finished immediately") {
  ChunkPlan p(0, 16384, 64, true, false);
  CHECK(p.finished());
  CHECK(p.done() == 0);
}
