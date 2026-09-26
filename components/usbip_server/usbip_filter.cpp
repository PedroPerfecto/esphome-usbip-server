#include "usbip_filter.h"
#include <cstring>

namespace usbip {

namespace {
constexpr uint8_t kDtConfig = 0x02;
constexpr uint8_t kDtInterface = 0x04;
constexpr uint8_t kDtIad = 0x0B;
constexpr size_t kConfigHeaderSize = 9;
constexpr size_t kInterfaceDescSize = 9;
constexpr size_t kIadSize = 8;
}  // namespace

size_t filter_config_descriptor(const uint8_t *cfg, size_t cfg_len, uint32_t mask, uint8_t *out, size_t out_cap) {
  if (cfg == nullptr || out == nullptr || cfg_len < kConfigHeaderSize || cfg[0] < kConfigHeaderSize ||
      cfg[1] != kDtConfig || out_cap < kConfigHeaderSize)
    return 0;
  const size_t total = static_cast<size_t>(cfg[2]) | (static_cast<size_t>(cfg[3]) << 8);
  if (total > cfg_len || total < cfg[0]) return 0;

  memcpy(out, cfg, kConfigHeaderSize);
  size_t out_len = kConfigHeaderSize;
  bool keep = true;  // descriptors before the first interface
  uint32_t seen = 0;  // kept bInterfaceNumber values (for bNumInterfaces)
  uint8_t num_interfaces = 0;

  size_t pos = cfg[0];
  while (pos < total) {
    const uint8_t len = cfg[pos];
    if (len < 2 || pos + len > total) return 0;
    const uint8_t type = cfg[pos + 1];
    if (type == kDtInterface) {
      if (len < kInterfaceDescSize) return 0;
      const uint8_t inum = cfg[pos + 2];
      keep = interface_exported(mask, inum);
      if (keep && (seen & (1u << inum)) == 0) {
        seen |= 1u << inum;
        ++num_interfaces;
      }
    } else if (type == kDtIad) {
      if (len < kIadSize) return 0;
      // An IAD does not start an interface by itself; it is judged by bFirstInterface
      // and does not affect whether the following descriptors are kept.
      if (!interface_exported(mask, cfg[pos + 2])) {
        pos += len;
        continue;
      }
      if (out_len + len > out_cap) return 0;
      memcpy(out + out_len, cfg + pos, len);
      out_len += len;
      pos += len;
      continue;
    }
    if (keep) {
      if (out_len + len > out_cap) return 0;
      memcpy(out + out_len, cfg + pos, len);
      out_len += len;
    }
    pos += len;
  }
  if (num_interfaces == 0 || out_len > 0xFFFF) return 0;

  out[2] = static_cast<uint8_t>(out_len & 0xFF);
  out[3] = static_cast<uint8_t>(out_len >> 8);
  out[4] = num_interfaces;
  return out_len;
}

bool is_get_config_descriptor(const uint8_t setup[8], uint8_t &index_out) {
  if (setup[0] != 0x80 || setup[1] != 0x06 || setup[3] != kDtConfig) return false;
  index_out = setup[2];
  return true;
}

}  // namespace usbip
