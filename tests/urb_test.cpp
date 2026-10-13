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

// Control OUT: the data stage sent to the device is wLength bytes taken from the CMD_SUBMIT payload.
TEST_CASE("control_out_lengths_ok: wLength, transfer_buffer_length and payload must agree") {
  CHECK(control_out_lengths_ok(0, 0, 0));        // no data stage (SET_IDLE, SET_CONFIGURATION ...)
  CHECK(control_out_lengths_ok(7, 7, 7));        // SET_LINE_CODING
  CHECK(control_out_lengths_ok(65535, 65535, 65535));
}

TEST_CASE("control_out_lengths_ok: wLength larger than the data the client sent is refused") {
  // Without the check the device would get 64 B, of which 57 B are uninitialised buffer content.
  CHECK_FALSE(control_out_lengths_ok(64, 7, 7));
  CHECK_FALSE(control_out_lengths_ok(1, 0, 0));
}

TEST_CASE("control_out_lengths_ok: wLength smaller than transfer_buffer_length is refused") {
  // Without the check the data stage would be silently truncated to wLength.
  CHECK_FALSE(control_out_lengths_ok(7, 64, 64));
  CHECK_FALSE(control_out_lengths_ok(0, 8, 8));
}

TEST_CASE("control_out_lengths_ok: payload missing although the lengths agree is refused") {
  // Header direction IN with an OUT setup packet: the parser reads no payload (payload_len 0).
  CHECK_FALSE(control_out_lengths_ok(7, 7, 0));
  CHECK_FALSE(control_out_lengths_ok(7, 7, 6));
}

TEST_CASE("control_in_reply_len: filtered config descriptor longer than transfer_buffer_length is cut to it") {
  // GET_DESCRIPTOR(CONFIG) answered by the server: 60 B descriptor, wLength 255, client buffer 9 B.
  // A RET_SUBMIT with actual_length 60 would exceed the client's buffer.
  CHECK(control_in_reply_len(60, 255, 9) == 9);
  CHECK(control_in_reply_len(60, 255, 0) == 0);
}

TEST_CASE("control_in_reply_len: min(data, wLength, transfer_buffer_length)") {
  CHECK(control_in_reply_len(60, 255, 255) == 60);  // data is the shortest
  CHECK(control_in_reply_len(60, 9, 9) == 9);       // usual config header read
  CHECK(control_in_reply_len(60, 9, 255) == 9);     // wLength is the shortest
  CHECK(control_in_reply_len(0, 9, 9) == 0);
  CHECK(control_in_reply_len(512, 65535, 131072) == 512);
  CHECK(control_in_reply_len(70000, 65535, 131072) == 65535);
}

TEST_CASE("control_in_reply_len as the limit for a forwarded IN reply") {
  // Slot limit = min(wLength, transfer_buffer_length); the device data is clamped to it.
  const uint32_t limit = control_in_reply_len(9, 18, 9);  // wLength 18, client buffer 9
  CHECK(limit == 9);
  CHECK(control_data_len(8 + 18, limit) == 9);
}
