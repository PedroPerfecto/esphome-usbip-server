#include "usbip_endpoints.h"

namespace usbip {

static uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

bool parse_endpoints(const uint8_t *cfg, size_t cfg_len, EndpointInfo *out, size_t max_count, size_t &count) {
  count = 0;
  if (cfg == nullptr || cfg_len < 9 || cfg[0] != 9 || cfg[1] != 0x02) return false;
  size_t total = le16(cfg + 2);
  if (total < 9 || total > cfg_len) return false;

  bool in_alt0 = false;
  size_t pos = 0;
  while (pos + 2 <= total) {
    uint8_t len = cfg[pos];
    uint8_t type = cfg[pos + 1];
    if (len < 2 || pos + len > total) return false;
    if (type == 0x04 && len >= 9) {
      in_alt0 = cfg[pos + 3] == 0;  // bAlternateSetting
    } else if (type == 0x05 && len >= 7 && in_alt0) {
      if (count >= max_count) return false;
      out[count].address = cfg[pos + 2];
      out[count].type = cfg[pos + 3] & 0x03;
      out[count].mps = le16(cfg + pos + 4) & 0x07FF;
      ++count;
    }
    pos += len;
  }
  return true;
}

const EndpointInfo *find_endpoint(const EndpointInfo *eps, size_t count, uint8_t address) {
  for (size_t i = 0; i < count; ++i) {
    if (eps[i].address == address) return &eps[i];
  }
  return nullptr;
}

uint8_t endpoint_address(uint32_t ep, uint32_t direction) {
  return static_cast<uint8_t>((ep & 0x0F) | (direction != 0 ? 0x80 : 0x00));
}

uint32_t round_up_to_mps(uint32_t len, uint16_t mps) {
  if (mps == 0) return len;
  return ((len + mps - 1) / mps) * mps;
}

}  // namespace usbip
