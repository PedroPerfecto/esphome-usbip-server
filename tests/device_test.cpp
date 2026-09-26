#include "doctest.h"
#include "../components/usbip_server/usbip_device.h"
#include <cstring>
#include <string>
#include <vector>

using namespace usbip;

// Real JA-106K (JA-100 Flexi) descriptors, captured from
// /sys/bus/usb/devices/1-1.4/descriptors on a Raspberry Pi:
// device descriptor (18 B) + config descriptor (wTotalLength 0x40 = 64 B).
static const uint8_t kJa106kDescriptors[] = {
    0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x40, 0xd6, 0x16, 0x08, 0x00, 0x06, 0x01, 0x01, 0x02,
    0x03, 0x01, 0x09, 0x02, 0x40, 0x00, 0x02, 0x01, 0x00, 0x40, 0x0a, 0x09, 0x04, 0x00, 0x00, 0x02,
    0x03, 0x00, 0x00, 0x04, 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x1d, 0x00, 0x07, 0x05, 0x81,
    0x03, 0x40, 0x00, 0x01, 0x07, 0x05, 0x01, 0x03, 0x40, 0x00, 0x01, 0x09, 0x04, 0x01, 0x00, 0x02,
    0x08, 0x06, 0x50, 0x05, 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, 0x07, 0x05, 0x02, 0x02, 0x40,
    0x00, 0x00};

static DeviceInfo ja106k() {
  DeviceInfo d;
  REQUIRE(sizeof(kJa106kDescriptors) == 82);
  REQUIRE(device_info_from_descriptors(kJa106kDescriptors, 18, kJa106kDescriptors + 18, 64, 1, 2, kSpeedFull,
                                       "1-1", "/esphome/usb/1-1", d));
  return d;
}

TEST_CASE("DeviceInfo from real JA-106K descriptors") {
  DeviceInfo d = ja106k();
  CHECK(d.idVendor == 0x16d6);
  CHECK(d.idProduct == 0x0008);
  CHECK(d.bcdDevice == 0x0106);
  CHECK(d.bDeviceClass == 0);
  CHECK(d.bDeviceSubClass == 0);
  CHECK(d.bDeviceProtocol == 0);
  CHECK(d.bNumConfigurations == 1);
  CHECK(d.bConfigurationValue == 1);
  REQUIRE(d.bNumInterfaces == 2);
  CHECK(d.interfaces[0].bInterfaceClass == 0x03);  // HID
  CHECK(d.interfaces[0].bInterfaceSubClass == 0x00);
  CHECK(d.interfaces[0].bInterfaceProtocol == 0x00);
  CHECK(d.interfaces[1].bInterfaceClass == 0x08);  // Mass Storage
  CHECK(d.interfaces[1].bInterfaceSubClass == 0x06);  // SCSI
  CHECK(d.interfaces[1].bInterfaceProtocol == 0x50);  // Bulk-Only
  CHECK(std::string(d.busid) == "1-1");
  CHECK(d.speed == kSpeedFull);
}

TEST_CASE("encode_device_info field offsets (usbip_usb_device, packed)") {
  DeviceInfo d = ja106k();
  uint8_t b[kDeviceInfoSize];
  encode_device_info(d, b);
  CHECK(std::string(reinterpret_cast<char *>(b)) == "/esphome/usb/1-1");
  CHECK(std::string(reinterpret_cast<char *>(b + 0x100)) == "1-1");
  CHECK(get_u32(b + 0x120) == 1);  // busnum
  CHECK(get_u32(b + 0x124) == 2);  // devnum
  CHECK(get_u32(b + 0x128) == 2);  // speed FULL
  CHECK(get_u16(b + 0x12C) == 0x16d6);
  CHECK(get_u16(b + 0x12E) == 0x0008);
  CHECK(get_u16(b + 0x130) == 0x0106);
  CHECK(b[0x135] == 1);  // bConfigurationValue
  CHECK(b[0x136] == 1);  // bNumConfigurations
  CHECK(b[0x137] == 2);  // bNumInterfaces
}

TEST_CASE("OP_REP_DEVLIST with JA-106K: layout per usbip_protocol.rst offsets") {
  DeviceInfo d = ja106k();
  uint8_t buf[512];
  size_t n = encode_rep_devlist(&d, buf, sizeof(buf));
  REQUIRE(n == 12 + 312 + 2 * 4);
  CHECK(get_u16(buf + 0) == kVersion);
  CHECK(get_u16(buf + 2) == kOpRepDevlist);
  CHECK(get_u32(buf + 4) == 0);
  CHECK(get_u32(buf + 8) == 1);
  // Offsets from the OP_REP_DEVLIST table (from the start of the message)
  CHECK(std::string(reinterpret_cast<char *>(buf + 0x10C)) == "1-1");
  CHECK(get_u16(buf + 0x138) == 0x16d6);
  CHECK(get_u16(buf + 0x13A) == 0x0008);
  CHECK(buf[0x143] == 2);  // bNumInterfaces
  CHECK(buf[0x144] == 0x03);
  CHECK(buf[0x145] == 0x00);
  CHECK(buf[0x146] == 0x00);
  CHECK(buf[0x147] == 0x00);  // padding
  CHECK(buf[0x148] == 0x08);
  CHECK(buf[0x149] == 0x06);
  CHECK(buf[0x14A] == 0x50);
  CHECK(buf[0x14B] == 0x00);
}

TEST_CASE("OP_REP_DEVLIST with no device is 12 bytes, count 0") {
  uint8_t buf[16];
  REQUIRE(encode_rep_devlist(nullptr, buf, sizeof(buf)) == 12);
  CHECK(get_u16(buf + 2) == kOpRepDevlist);
  CHECK(get_u32(buf + 8) == 0);
}

TEST_CASE("OP_REP_DEVLIST refuses too small buffer") {
  DeviceInfo d = ja106k();
  uint8_t buf[100];
  CHECK(encode_rep_devlist(&d, buf, sizeof(buf)) == 0);
}

TEST_CASE("OP_REP_IMPORT ok: 320 B, offsets per usbip_protocol.rst") {
  DeviceInfo d = ja106k();
  uint8_t buf[kRepImportOkSize];
  REQUIRE(encode_rep_import(&d, buf, sizeof(buf)) == 320);
  CHECK(get_u16(buf + 2) == kOpRepImport);
  CHECK(get_u32(buf + 4) == 0);
  CHECK(std::string(reinterpret_cast<char *>(buf + 0x108)) == "1-1");  // the client compares busid
  CHECK(get_u16(buf + 0x134) == 0x16d6);
  CHECK(get_u16(buf + 0x136) == 0x0008);
  CHECK(buf[0x13F] == 2);
}

TEST_CASE("OP_REP_IMPORT error: 8 B, status 1") {
  uint8_t buf[8];
  REQUIRE(encode_rep_import(nullptr, buf, sizeof(buf)) == 8);
  CHECK(get_u16(buf + 2) == kOpRepImport);
  CHECK(get_u32(buf + 4) == 1);
}

TEST_CASE("decode_req_import_busid requires NUL terminator") {
  uint8_t in[kBusIdSize] = {'1', '-', '1', 0};
  char out[kBusIdSize];
  REQUIRE(decode_req_import_busid(in, out));
  CHECK(std::string(out) == "1-1");
  uint8_t bad[kBusIdSize];
  std::memset(bad, 'x', sizeof(bad));
  CHECK_FALSE(decode_req_import_busid(bad, out));
}

TEST_CASE("invalid descriptors are rejected") {
  DeviceInfo d;
  // truncated config descriptor (wTotalLength 64 > 40 available)
  CHECK_FALSE(device_info_from_descriptors(kJa106kDescriptors, 18, kJa106kDescriptors + 18, 40, 1, 2, kSpeedFull,
                                           "1-1", "/p", d));
  // wrong device descriptor type
  uint8_t dev[18];
  std::memcpy(dev, kJa106kDescriptors, 18);
  dev[1] = 0x02;
  CHECK_FALSE(device_info_from_descriptors(dev, 18, kJa106kDescriptors + 18, 64, 1, 2, kSpeedFull, "1-1", "/p", d));
  // busid too long (32 characters with no room for the null)
  std::string longid(32, 'a');
  CHECK_FALSE(device_info_from_descriptors(kJa106kDescriptors, 18, kJa106kDescriptors + 18, 64, 1, 2, kSpeedFull,
                                           longid.c_str(), "/p", d));
}
