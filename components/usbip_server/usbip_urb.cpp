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

}  // namespace usbip
