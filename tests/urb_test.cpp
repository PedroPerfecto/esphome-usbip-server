#include "doctest.h"
#include "../components/usbip_server/usbip_urb.h"

using namespace usbip;

TEST_CASE("Linux errno values (asm-generic)") {
  CHECK(kLinuxENOENT == -2);
  CHECK(kLinuxEPIPE == -32);
  CHECK(kLinuxECONNRESET == -104);
  CHECK(kLinuxESHUTDOWN == -108);
}

TEST_CASE("parse_setup: GET_DESCRIPTOR(device) IN, wLength 18") {
  const uint8_t s[8] = {0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00};
  SetupInfo i = parse_setup(s);
  CHECK(i.dir_in);
  CHECK(i.wLength == 18);
}

TEST_CASE("parse_setup: SET_IDLE OUT, wLength 0; little-endian wLength") {
  const uint8_t s[8] = {0x21, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK_FALSE(parse_setup(s).dir_in);
  CHECK(parse_setup(s).wLength == 0);
  const uint8_t big[8] = {0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01};  // wLength 256
  CHECK(parse_setup(big).wLength == 256);
}

TEST_CASE("control_data_len strips setup and clamps") {
  CHECK(control_data_len(8 + 18, 18) == 18);   // whole device descriptor
  CHECK(control_data_len(8 + 9, 255) == 9);    // short reply (config header)
  CHECK(control_data_len(8 + 100, 18) == 18);  // more than requested -> clamp
  CHECK(control_data_len(8, 0) == 0);          // no data stage
  CHECK(control_data_len(5, 18) == 0);         // invalid -> 0
  CHECK(control_data_len(0, 18) == 0);
}
