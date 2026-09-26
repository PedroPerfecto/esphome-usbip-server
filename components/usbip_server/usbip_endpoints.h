#pragma once
#include <cstdint>
#include <cstddef>

// Device endpoints and helper calculations for non-control transfers
// (no ESPHome dependencies).
namespace usbip {

// bmAttributes & 0x03 (USB 2.0 table 9-13)
constexpr uint8_t kEpTypeControl = 0;
constexpr uint8_t kEpTypeIso = 1;
constexpr uint8_t kEpTypeBulk = 2;
constexpr uint8_t kEpTypeInterrupt = 3;

// transfer_flags from CMD_SUBMIT (include/uapi/linux/usbip.h)
constexpr uint32_t kUrbShortNotOk = 0x0001;
constexpr uint32_t kUrbZeroPacket = 0x0040;
constexpr uint32_t kUrbDirIn = 0x0200;

constexpr size_t kMaxEndpoints = 16;

struct EndpointInfo {
  uint8_t address = 0;  // including direction (0x80 = IN)
  uint8_t type = 0;     // kEpType*
  uint16_t mps = 0;     // wMaxPacketSize & 0x7FF
};

// Selects endpoints of interfaces with bAlternateSetting == 0 from the config descriptor.
// Returns false on a corrupted descriptor or more than max_count endpoints.
bool parse_endpoints(const uint8_t *cfg, size_t cfg_len, EndpointInfo *out, size_t max_count, size_t &count);

// Finds an endpoint by address; nullptr if it does not exist.
const EndpointInfo *find_endpoint(const EndpointInfo *eps, size_t count, uint8_t address);

// Endpoint address from the USB/IP header: ep (number) + direction (1 = IN).
uint8_t endpoint_address(uint32_t ep, uint32_t direction);

// For non-control IN transfers ESP-IDF requires num_bytes to be a multiple of MPS
// (usb_types_stack.h, ESP-IDF 5.5). Rounds up; mps == 0 -> unchanged.
uint32_t round_up_to_mps(uint32_t len, uint16_t mps);

}  // namespace usbip
