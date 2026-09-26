#pragma once
#include <cstdint>

// Helper calculations for URBs (no ESPHome dependencies).
namespace usbip {

// The URB status in RET_SUBMIT/RET_UNLINK is a negative Linux errno. The values are
// from Linux (include/uapi/asm-generic/errno-base.h and errno.h), NOT from newlib
// on the ESP32, where they may differ.
constexpr int32_t kLinuxENOENT = -2;
constexpr int32_t kLinuxEPIPE = -32;
constexpr int32_t kLinuxECONNRESET = -104;
constexpr int32_t kLinuxESHUTDOWN = -108;

constexpr uint32_t kSetupPacketSize = 8;

struct SetupInfo {
  bool dir_in = false;   // bmRequestType bit 7
  uint16_t wLength = 0;  // little-endian in the setup packet
};

SetupInfo parse_setup(const uint8_t setup[kSetupPacketSize]);

// A control transfer in ESP-IDF has the buffer [8 B setup][data] and actual_num_bytes
// includes the setup too. Returns the number of data bytes for RET_SUBMIT, clamped
// to the range 0..requested.
uint32_t control_data_len(int actual_num_bytes_with_setup, uint32_t requested);

}  // namespace usbip
