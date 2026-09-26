#include "doctest.h"
#include "../components/usbip_server/usbip_endpoints.h"

using namespace usbip;

// JA-106K config descriptor (64 B) - same as in device_test.cpp, from offset 18.
static const uint8_t kJa106kConfig[] = {
    0x09, 0x02, 0x40, 0x00, 0x02, 0x01, 0x00, 0x40, 0x0a, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00,
    0x00, 0x04, 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x1d, 0x00, 0x07, 0x05, 0x81, 0x03, 0x40,
    0x00, 0x01, 0x07, 0x05, 0x01, 0x03, 0x40, 0x00, 0x01, 0x09, 0x04, 0x01, 0x00, 0x02, 0x08, 0x06,
    0x50, 0x05, 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, 0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00};

TEST_CASE("endpoints of real JA-106K config descriptor") {
  REQUIRE(sizeof(kJa106kConfig) == 64);
  EndpointInfo eps[kMaxEndpoints];
  size_t n = 0;
  REQUIRE(parse_endpoints(kJa106kConfig, sizeof(kJa106kConfig), eps, kMaxEndpoints, n));
  REQUIRE(n == 4);
  CHECK(eps[0].address == 0x81);
  CHECK(eps[0].type == kEpTypeInterrupt);
  CHECK(eps[0].mps == 64);
  CHECK(eps[1].address == 0x01);
  CHECK(eps[1].type == kEpTypeInterrupt);
  CHECK(eps[2].address == 0x82);
  CHECK(eps[2].type == kEpTypeBulk);
  CHECK(eps[2].mps == 64);
  CHECK(eps[3].address == 0x02);
  CHECK(eps[3].type == kEpTypeBulk);

  CHECK(find_endpoint(eps, n, 0x82) == &eps[2]);
  CHECK(find_endpoint(eps, n, 0x83) == nullptr);
}

TEST_CASE("endpoints in non-zero alternate settings are ignored") {
  // config: 1 interface, alt 0 with EP 0x81, alt 1 with EP 0x82
  const uint8_t cfg[] = {0x09, 0x02, 0x2E, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
                         0x09, 0x04, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00,
                         0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0A,
                         0x09, 0x04, 0x00, 0x01, 0x01, 0xFF, 0x00, 0x00, 0x00,
                         0x07, 0x05, 0x82, 0x01, 0x00, 0x02, 0x01,
                         0x05, 0x24, 0x00, 0x00, 0x00};  // class-specific, skip
  REQUIRE(sizeof(cfg) == 0x2E);
  EndpointInfo eps[kMaxEndpoints];
  size_t n = 0;
  REQUIRE(parse_endpoints(cfg, sizeof(cfg), eps, kMaxEndpoints, n));
  REQUIRE(n == 1);
  CHECK(eps[0].address == 0x81);
  CHECK(eps[0].mps == 8);
}

TEST_CASE("high-bandwidth bits are masked from wMaxPacketSize") {
  const uint8_t cfg[] = {0x09, 0x02, 0x19, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
                         0x09, 0x04, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00,
                         0x07, 0x05, 0x81, 0x03, 0x00, 0x14, 0x01};  // 0x1400: 2 additional transactions, 1024 B
  EndpointInfo eps[kMaxEndpoints];
  size_t n = 0;
  REQUIRE(parse_endpoints(cfg, sizeof(cfg), eps, kMaxEndpoints, n));
  REQUIRE(n == 1);
  CHECK(eps[0].mps == 0x400);
}

TEST_CASE("corrupt config descriptor is rejected") {
  EndpointInfo eps[kMaxEndpoints];
  size_t n = 0;
  CHECK_FALSE(parse_endpoints(kJa106kConfig, 40, eps, kMaxEndpoints, n));  // wTotalLength > available
  CHECK_FALSE(parse_endpoints(kJa106kConfig, sizeof(kJa106kConfig), eps, 3, n));  // more than max_count
}

TEST_CASE("endpoint_address from USB/IP header") {
  CHECK(endpoint_address(1, 1) == 0x81);
  CHECK(endpoint_address(1, 0) == 0x01);
  CHECK(endpoint_address(2, 1) == 0x82);
  CHECK(endpoint_address(0x12, 0) == 0x02);  // lower 4 bits only
}

TEST_CASE("round_up_to_mps") {
  CHECK(round_up_to_mps(29, 64) == 64);
  CHECK(round_up_to_mps(64, 64) == 64);
  CHECK(round_up_to_mps(65, 64) == 128);
  CHECK(round_up_to_mps(13, 64) == 64);   // BOT CSW 13 B
  CHECK(round_up_to_mps(4096, 64) == 4096);
  CHECK(round_up_to_mps(0, 64) == 0);
  CHECK(round_up_to_mps(29, 0) == 29);
}

TEST_CASE("URB flag values (include/uapi/linux/usbip.h)") {
  CHECK(kUrbShortNotOk == 0x0001);
  CHECK(kUrbZeroPacket == 0x0040);
  CHECK(kUrbDirIn == 0x0200);
}
