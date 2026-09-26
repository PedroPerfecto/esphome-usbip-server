#pragma once
#include <cstdint>
#include <cstddef>
#include "usbip_wire.h"

// Description of the exported device for OP_REP_DEVLIST and OP_REP_IMPORT.
// Format: struct usbip_usb_device and usbip_usb_interface
// (tools/usb/usbip/libsrc/usbip_common.h, Documentation/usb/usbip_protocol.rst).
namespace usbip {

constexpr size_t kPathSize = 256;         // SYSFS_PATH_MAX
constexpr size_t kDeviceInfoSize = 312;   // 0x138: usbip_usb_device (packed)
constexpr size_t kInterfaceInfoSize = 4;  // class, subclass, protocol, padding
constexpr size_t kMaxInterfaces = 8;

// enum usb_device_speed (include/uapi/linux/usb/ch9.h)
constexpr uint32_t kSpeedLow = 1;
constexpr uint32_t kSpeedFull = 2;
constexpr uint32_t kSpeedHigh = 3;

struct InterfaceInfo {
  uint8_t bInterfaceClass = 0;
  uint8_t bInterfaceSubClass = 0;
  uint8_t bInterfaceProtocol = 0;
};

struct DeviceInfo {
  char path[kPathSize] = {0};
  char busid[kBusIdSize] = {0};
  uint32_t busnum = 0;
  uint32_t devnum = 0;
  uint32_t speed = 0;
  uint16_t idVendor = 0;
  uint16_t idProduct = 0;
  uint16_t bcdDevice = 0;
  uint8_t bDeviceClass = 0;
  uint8_t bDeviceSubClass = 0;
  uint8_t bDeviceProtocol = 0;
  uint8_t bConfigurationValue = 0;
  uint8_t bNumConfigurations = 0;
  uint8_t bNumInterfaces = 0;
  InterfaceInfo interfaces[kMaxInterfaces];
};

// Fills DeviceInfo from the device descriptor (18 B) and the whole config descriptor
// (wTotalLength B), as returned by the USB device (little-endian fields).
// Takes only interfaces with bAlternateSetting == 0, in order of appearance.
// path and busid must be strings shorter than kPathSize / kBusIdSize.
// Returns false on invalid/truncated descriptors or overly long strings.
bool device_info_from_descriptors(const uint8_t *dev_desc, size_t dev_len, const uint8_t *cfg_desc,
                                  size_t cfg_len, uint32_t busnum, uint32_t devnum, uint32_t speed,
                                  const char *busid, const char *path, DeviceInfo &out);

// Writes usbip_usb_device (312 B, big-endian) to out.
void encode_device_info(const DeviceInfo &d, uint8_t out[kDeviceInfoSize]);

// Size of OP_REP_DEVLIST: 12 B + (312 B + 4 B * interfaces) if there is a device.
size_t rep_devlist_size(const DeviceInfo *dev);
// OP_REP_DEVLIST with 0 or 1 devices (dev == nullptr -> 0 devices).
// Returns the written length, or 0 if cap is insufficient.
size_t encode_rep_devlist(const DeviceInfo *dev, uint8_t *out, size_t cap);

constexpr size_t kRepImportOkSize = kOpHeaderSize + kDeviceInfoSize;  // 320
constexpr size_t kRepImportErrorSize = kOpHeaderSize;                 // 8
// OP_REP_IMPORT: dev != nullptr -> status 0 + device description (320 B),
// dev == nullptr -> status 1, header only (8 B). Returns the length or 0 if cap is insufficient.
size_t encode_rep_import(const DeviceInfo *dev, uint8_t *out, size_t cap);

// Extracts busid as a string from the OP_REQ_IMPORT body (32 B after the header).
// Returns false if the 32 B contain no terminating null.
bool decode_req_import_busid(const uint8_t in[kBusIdSize], char busid_out[kBusIdSize]);

}  // namespace usbip
