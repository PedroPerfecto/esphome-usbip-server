#include "doctest.h"
#include "../components/usbip_server/usbip_time.h"

using namespace usbip;

TEST_CASE("elapsed32: ordinary differences") {
  CHECK(elapsed32(1000, 1000) == 0);
  CHECK(elapsed32(1000, 400) == 600);
  CHECK(elapsed32(3600000, 0) == 3600000);
}

TEST_CASE("elapsed32: across a wrap of the clock") {
  CHECK(elapsed32(0x00000010u, 0xFFFFFFF0u) == 0x20);
  CHECK(elapsed32(0x00001000u, 0xFFFFF000u) == 0x2000);
  CHECK(elapsed32(0x80000005u, 0x7FFFFFF0u) == 0x15);
}

TEST_CASE("elapsed32: a 'since' newer than 'now' is 0, not 49 days") {
  // The other task stored its time right after 'now' was read.
  CHECK(elapsed32(1000, 1001) == 0);
  CHECK(elapsed32(1000, 4000) == 0);
  CHECK(elapsed32(0xFFFFFFFFu, 0x00000000u) == 0);  // since is 1 newer, across the wrap
  CHECK(elapsed32(0, 1) == 0);
}

TEST_CASE("elapsed32: range is 2^31 - 1") {
  CHECK(elapsed32(0x7FFFFFFFu, 0) == 0x7FFFFFFFu);
  CHECK(elapsed32(0x80000000u, 0) == 0);  // not distinguishable from a newer 'since' any more
}
