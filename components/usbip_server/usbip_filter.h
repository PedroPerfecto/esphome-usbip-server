#pragma once
#include <cstdint>
#include <cstddef>

// Filtering interfaces of a composite device (no ESPHome dependencies).
// The client (Linux) enumerates the device from the config descriptor we send it;
// an interface missing from it is invisible to the client kernel and gets no driver bound.
namespace usbip {

// Bit mask of exported interfaces: bit N = bInterfaceNumber N.
// All ones = no filtering.
constexpr uint32_t kAllInterfaces = 0xFFFFFFFFu;

inline bool interface_exported(uint32_t mask, uint8_t inum) { return inum < 32 && (mask & (1u << inum)) != 0; }

// From the (whole, wTotalLength B) config descriptor creates a copy with only the interfaces
// in the mask: their interface, class-specific and endpoint descriptors (all
// alternate settings). An IAD is kept if its bFirstInterface is in the mask.
// Descriptors before the first interface are kept. Fixes wTotalLength
// and bNumInterfaces (number of distinct kept bInterfaceNumber values).
// Returns the result length; 0 on corrupted input, insufficient space
// or if no interface would remain.
size_t filter_config_descriptor(const uint8_t *cfg, size_t cfg_len, uint32_t mask, uint8_t *out, size_t out_cap);

// Standard GET_DESCRIPTOR(CONFIGURATION) to the device (USB 2.0 9.4.3):
// bmRequestType 0x80, bRequest 6, wValue = (2 << 8) | index. Returns the index in index_out.
bool is_get_config_descriptor(const uint8_t setup[8], uint8_t &index_out);

}  // namespace usbip
