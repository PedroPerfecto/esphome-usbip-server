#include "doctest.h"
#include "../components/usbip_server/usbip_control.h"

using namespace usbip;

// Setup packets: bmRequestType, bRequest, wValue(LE), wIndex(LE), wLength(LE)

TEST_CASE("SET_CONFIGURATION(1) as sent by the client during attach") {
  const uint8_t s[8] = {0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  ControlAction a = classify_control(s);
  CHECK(a.kind == ControlKind::kSetConfiguration);
  CHECK(a.configuration == 1);
}

TEST_CASE("SET_INTERFACE(interface 1, alt 0)") {
  const uint8_t s[8] = {0x01, 0x0B, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  ControlAction a = classify_control(s);
  CHECK(a.kind == ControlKind::kSetInterface);
  CHECK(a.interface == 1);
  CHECK(a.alternate == 0);
}

TEST_CASE("CLEAR_FEATURE(ENDPOINT_HALT) on bulk IN 0x82") {
  const uint8_t s[8] = {0x02, 0x01, 0x00, 0x00, 0x82, 0x00, 0x00, 0x00};
  ControlAction a = classify_control(s);
  CHECK(a.kind == ControlKind::kClearHalt);
  CHECK(a.endpoint == 0x82);
}

TEST_CASE("CLEAR_FEATURE(ENDPOINT_HALT) on OUT 0x02") {
  const uint8_t s[8] = {0x02, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  ControlAction a = classify_control(s);
  CHECK(a.kind == ControlKind::kClearHalt);
  CHECK(a.endpoint == 0x02);
}

TEST_CASE("port reset SET_FEATURE(PORT_RESET) type 0x23") {
  const uint8_t s[8] = {0x23, 0x03, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00};
  CHECK(classify_control(s).kind == ControlKind::kResetDevice);
}

TEST_CASE("ordinary requests are forwarded") {
  // GET_DESCRIPTOR(device) - ordinary descriptor read
  const uint8_t get_desc[8] = {0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00};
  CHECK(classify_control(get_desc).kind == ControlKind::kForward);
  // HID GET_REPORT_DESCRIPTOR (GET_DESCRIPTOR on an interface, type 0x22)
  const uint8_t get_report[8] = {0x81, 0x06, 0x00, 0x22, 0x00, 0x00, 0x1d, 0x00};
  CHECK(classify_control(get_report).kind == ControlKind::kForward);
  // HID SET_IDLE (class, interface)
  const uint8_t set_idle[8] = {0x21, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(classify_control(set_idle).kind == ControlKind::kForward);
  // Mass storage GET_MAX_LUN (class, interface 1)
  const uint8_t max_lun[8] = {0xA1, 0xFE, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00};
  CHECK(classify_control(max_lun).kind == ControlKind::kForward);
}

TEST_CASE("near misses are forwarded (exact match like stub_rx.c)") {
  // CLEAR_FEATURE on an endpoint, but a feature other than HALT
  const uint8_t cf_other[8] = {0x02, 0x01, 0x01, 0x00, 0x81, 0x00, 0x00, 0x00};
  CHECK(classify_control(cf_other).kind == ControlKind::kForward);
  // CLEAR_FEATURE(0) on the device (DEVICE_REMOTE_WAKEUP has a different value, but recipient device)
  const uint8_t cf_dev[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(classify_control(cf_dev).kind == ControlKind::kForward);
  // SET_CONFIGURATION with recipient interface (non-standard)
  const uint8_t sc_intf[8] = {0x01, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(classify_control(sc_intf).kind == ControlKind::kForward);
  // SET_FEATURE port, but a feature other than RESET (e.g. PORT_POWER = 8)
  const uint8_t sf_power[8] = {0x23, 0x03, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00};
  CHECK(classify_control(sf_power).kind == ControlKind::kForward);
}
