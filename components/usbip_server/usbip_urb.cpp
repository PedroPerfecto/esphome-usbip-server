#include "usbip_urb.h"

namespace usbip {

SetupInfo parse_setup(const uint8_t setup[kSetupPacketSize]) {
  SetupInfo s;
  s.dir_in = (setup[0] & 0x80) != 0;
  s.wLength = static_cast<uint16_t>(setup[6] | (setup[7] << 8));
  return s;
}

uint32_t control_data_len(int actual_num_bytes_with_setup, uint32_t requested) {
  if (actual_num_bytes_with_setup <= static_cast<int>(kSetupPacketSize)) return 0;
  uint32_t data = static_cast<uint32_t>(actual_num_bytes_with_setup) - kSetupPacketSize;
  return data > requested ? requested : data;
}

uint32_t control_in_reply_len(uint32_t data_len, uint16_t wLength, uint32_t transfer_buffer_length) {
  uint32_t len = data_len;
  if (len > wLength) len = wLength;
  if (len > transfer_buffer_length) len = transfer_buffer_length;
  return len;
}

bool in_overflow(int actual_num_bytes, uint32_t requested) {
  return actual_num_bytes > 0 && static_cast<uint32_t>(actual_num_bytes) > requested;
}

bool control_out_lengths_ok(uint16_t wLength, uint32_t transfer_buffer_length, uint32_t payload_len) {
  return wLength == transfer_buffer_length && wLength == payload_len;
}

}  // namespace usbip
