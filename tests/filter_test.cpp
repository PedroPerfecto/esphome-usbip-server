#include "doctest.h"
#include "../components/usbip_server/usbip_filter.h"
#include "../components/usbip_server/usbip_endpoints.h"

using namespace usbip;

// JA-106K config descriptor (64 B), same as in endpoints_test.cpp.
static const uint8_t kCfg[] = {
    0x09, 0x02, 0x40, 0x00, 0x02, 0x01, 0x00, 0x40, 0x0a, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00,
    0x00, 0x04, 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x1d, 0x00, 0x07, 0x05, 0x81, 0x03, 0x40,
    0x00, 0x01, 0x07, 0x05, 0x01, 0x03, 0x40, 0x00, 0x01, 0x09, 0x04, 0x01, 0x00, 0x02, 0x08, 0x06,
    0x50, 0x05, 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, 0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00};

TEST_CASE("filter: JA-106K HID only") {
  uint8_t out[128];
  size_t n = filter_config_descriptor(kCfg, sizeof(kCfg), 1u << 0, out, sizeof(out));
  // config 9 + interface 9 + HID 9 + 2x endpoint 7 = 41
  REQUIRE(n == 41);
  CHECK(out[2] == 41);
  CHECK(out[3] == 0);
  CHECK(out[4] == 1);  // bNumInterfaces
  // other header fields unchanged
  CHECK(out[0] == 0x09);
  CHECK(out[1] == 0x02);
  CHECK(out[5] == kCfg[5]);
  CHECK(out[6] == kCfg[6]);
  CHECK(out[7] == kCfg[7]);
  CHECK(out[8] == kCfg[8]);
  // body = exactly the first 32 B after the original header (interface 0 complete)
  for (size_t i = 9; i < 41; ++i) CHECK(out[i] == kCfg[i]);

  EndpointInfo eps[kMaxEndpoints];
  size_t count = 0;
  REQUIRE(parse_endpoints(out, n, eps, kMaxEndpoints, count));
  REQUIRE(count == 2);
  CHECK(eps[0].address == 0x81);
  CHECK(eps[1].address == 0x01);
}

TEST_CASE("filter: JA-106K mass storage only") {
  uint8_t out[128];
  size_t n = filter_config_descriptor(kCfg, sizeof(kCfg), 1u << 1, out, sizeof(out));
  // config 9 + interface 9 + 2x endpoint 7 = 32
  REQUIRE(n == 32);
  CHECK(out[4] == 1);
  CHECK(out[9 + 2] == 1);  // bInterfaceNumber
  for (size_t i = 9; i < 32; ++i) CHECK(out[i] == kCfg[i + 32]);
}

TEST_CASE("filter: all interfaces keeps descriptor byte for byte") {
  uint8_t out[128];
  size_t n = filter_config_descriptor(kCfg, sizeof(kCfg), kAllInterfaces, out, sizeof(out));
  REQUIRE(n == sizeof(kCfg));
  for (size_t i = 0; i < n; ++i) CHECK(out[i] == kCfg[i]);
}

TEST_CASE("filter: no interface left or bad input returns 0") {
  uint8_t out[128];
  CHECK(filter_config_descriptor(kCfg, sizeof(kCfg), 1u << 5, out, sizeof(out)) == 0);
  CHECK(filter_config_descriptor(kCfg, sizeof(kCfg), 0, out, sizeof(out)) == 0);
  CHECK(filter_config_descriptor(kCfg, 8, kAllInterfaces, out, sizeof(out)) == 0);
  CHECK(filter_config_descriptor(kCfg, sizeof(kCfg), kAllInterfaces, out, 40) == 0);  // not enough space
  uint8_t bad[sizeof(kCfg)];
  for (size_t i = 0; i < sizeof(kCfg); ++i) bad[i] = kCfg[i];
  bad[9] = 0;  // bLength 0 -> would risk an infinite loop
  CHECK(filter_config_descriptor(bad, sizeof(bad), kAllInterfaces, out, sizeof(out)) == 0);
  bad[9] = 0x09;
  bad[2] = 0x50;  // wTotalLength > buffer length
  CHECK(filter_config_descriptor(bad, sizeof(bad), kAllInterfaces, out, sizeof(out)) == 0);
}

TEST_CASE("filter: alternate settings and IAD") {
  // IAD(if 0..1) | if0 alt0 ep81 | if0 alt1 ep82 | if1 alt0 ep03
  const uint8_t cfg[] = {0x09, 0x02, 0x41, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,
                         0x08, 0x0B, 0x00, 0x02, 0x0E, 0x03, 0x00, 0x00,
                         0x09, 0x04, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00,
                         0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0A,
                         0x09, 0x04, 0x00, 0x01, 0x01, 0xFF, 0x00, 0x00, 0x00,
                         0x07, 0x05, 0x82, 0x01, 0x00, 0x02, 0x01,
                         0x09, 0x04, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00,
                         0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00};
  REQUIRE(sizeof(cfg) == 0x41);
  uint8_t out[128];
  // interface 0 only: IAD (first=0) + both alt settings, bNumInterfaces 1
  size_t n = filter_config_descriptor(cfg, sizeof(cfg), 1u << 0, out, sizeof(out));
  REQUIRE(n == 9 + 8 + 9 + 7 + 9 + 7);
  CHECK(out[4] == 1);
  // interface 1 only: the IAD is dropped (first=0)
  n = filter_config_descriptor(cfg, sizeof(cfg), 1u << 1, out, sizeof(out));
  REQUIRE(n == 9 + 9 + 7);
  CHECK(out[9 + 1] == 0x04);
  CHECK(out[4] == 1);
}

TEST_CASE("is_get_config_descriptor") {
  uint8_t idx = 0xFF;
  const uint8_t get_cfg[8] = {0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0xFF, 0x00};
  CHECK(is_get_config_descriptor(get_cfg, idx));
  CHECK(idx == 0);
  const uint8_t get_cfg1[8] = {0x80, 0x06, 0x01, 0x02, 0x00, 0x00, 0x09, 0x00};
  CHECK(is_get_config_descriptor(get_cfg1, idx));
  CHECK(idx == 1);
  const uint8_t get_dev[8] = {0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00};
  CHECK_FALSE(is_get_config_descriptor(get_dev, idx));
  const uint8_t get_report[8] = {0x81, 0x06, 0x00, 0x22, 0x00, 0x00, 0x1d, 0x00};
  CHECK_FALSE(is_get_config_descriptor(get_report, idx));
  const uint8_t set_cfg[8] = {0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK_FALSE(is_get_config_descriptor(set_cfg, idx));
}
