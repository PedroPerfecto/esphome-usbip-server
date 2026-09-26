#include "usbip_device.h"
#include <cstring>

namespace usbip {

static uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

static bool copy_str(char *dst, size_t cap, const char *src) {
  if (src == nullptr) return false;
  size_t n = std::strlen(src);
  if (n >= cap) return false;
  std::memset(dst, 0, cap);
  std::memcpy(dst, src, n);
  return true;
}

bool device_info_from_descriptors(const uint8_t *dev_desc, size_t dev_len, const uint8_t *cfg_desc,
                                  size_t cfg_len, uint32_t busnum, uint32_t devnum, uint32_t speed,
                                  const char *busid, const char *path, DeviceInfo &out) {
  DeviceInfo d;
  // Device descriptor: bLength=18, bDescriptorType=1
  if (dev_desc == nullptr || dev_len < 18 || dev_desc[0] != 18 || dev_desc[1] != 0x01) return false;
  // Config descriptor: bLength=9, bDescriptorType=2, wTotalLength <= cfg_len
  if (cfg_desc == nullptr || cfg_len < 9 || cfg_desc[0] != 9 || cfg_desc[1] != 0x02) return false;
  size_t total = le16(cfg_desc + 2);
  if (total < 9 || total > cfg_len) return false;

  if (!copy_str(d.path, kPathSize, path)) return false;
  if (!copy_str(d.busid, kBusIdSize, busid)) return false;

  d.busnum = busnum;
  d.devnum = devnum;
  d.speed = speed;
  d.bDeviceClass = dev_desc[4];
  d.bDeviceSubClass = dev_desc[5];
  d.bDeviceProtocol = dev_desc[6];
  d.idVendor = le16(dev_desc + 8);
  d.idProduct = le16(dev_desc + 10);
  d.bcdDevice = le16(dev_desc + 12);
  d.bNumConfigurations = dev_desc[17];
  d.bConfigurationValue = cfg_desc[5];

  // Walk the descriptors within the config descriptor, interfaces with alt 0.
  uint8_t count = 0;
  size_t pos = 0;
  while (pos + 2 <= total) {
    uint8_t len = cfg_desc[pos];
    uint8_t type = cfg_desc[pos + 1];
    if (len < 2 || pos + len > total) return false;  // corrupted descriptor
    if (type == 0x04 && len >= 9 && cfg_desc[pos + 3] == 0) {
      if (count >= kMaxInterfaces) return false;
      d.interfaces[count].bInterfaceClass = cfg_desc[pos + 5];
      d.interfaces[count].bInterfaceSubClass = cfg_desc[pos + 6];
      d.interfaces[count].bInterfaceProtocol = cfg_desc[pos + 7];
      ++count;
    }
    pos += len;
  }
  d.bNumInterfaces = count;
  out = d;
  return true;
}

void encode_device_info(const DeviceInfo &d, uint8_t out[kDeviceInfoSize]) {
  std::memset(out, 0, kDeviceInfoSize);
  std::memcpy(out + 0x000, d.path, kPathSize);
  std::memcpy(out + 0x100, d.busid, kBusIdSize);
  put_u32(out + 0x120, d.busnum);
  put_u32(out + 0x124, d.devnum);
  put_u32(out + 0x128, d.speed);
  put_u16(out + 0x12C, d.idVendor);
  put_u16(out + 0x12E, d.idProduct);
  put_u16(out + 0x130, d.bcdDevice);
  out[0x132] = d.bDeviceClass;
  out[0x133] = d.bDeviceSubClass;
  out[0x134] = d.bDeviceProtocol;
  out[0x135] = d.bConfigurationValue;
  out[0x136] = d.bNumConfigurations;
  out[0x137] = d.bNumInterfaces;
}

size_t rep_devlist_size(const DeviceInfo *dev) {
  size_t n = kOpHeaderSize + 4;
  if (dev != nullptr) n += kDeviceInfoSize + kInterfaceInfoSize * dev->bNumInterfaces;
  return n;
}

size_t encode_rep_devlist(const DeviceInfo *dev, uint8_t *out, size_t cap) {
  size_t need = rep_devlist_size(dev);
  if (out == nullptr || cap < need) return 0;
  std::memset(out, 0, need);
  encode_op_header({kVersion, kOpRepDevlist, 0}, out);
  put_u32(out + kOpHeaderSize, dev != nullptr ? 1 : 0);
  if (dev != nullptr) {
    uint8_t *p = out + kOpHeaderSize + 4;
    encode_device_info(*dev, p);
    p += kDeviceInfoSize;
    for (uint8_t i = 0; i < dev->bNumInterfaces; ++i) {
      p[0] = dev->interfaces[i].bInterfaceClass;
      p[1] = dev->interfaces[i].bInterfaceSubClass;
      p[2] = dev->interfaces[i].bInterfaceProtocol;
      p[3] = 0;  // padding
      p += kInterfaceInfoSize;
    }
  }
  return need;
}

size_t encode_rep_import(const DeviceInfo *dev, uint8_t *out, size_t cap) {
  if (out == nullptr) return 0;
  if (dev == nullptr) {
    if (cap < kRepImportErrorSize) return 0;
    encode_op_header({kVersion, kOpRepImport, 1}, out);
    return kRepImportErrorSize;
  }
  if (cap < kRepImportOkSize) return 0;
  encode_op_header({kVersion, kOpRepImport, 0}, out);
  encode_device_info(*dev, out + kOpHeaderSize);
  return kRepImportOkSize;
}

bool decode_req_import_busid(const uint8_t in[kBusIdSize], char busid_out[kBusIdSize]) {
  if (std::memchr(in, 0, kBusIdSize) == nullptr) return false;
  std::memcpy(busid_out, in, kBusIdSize);
  return true;
}

}  // namespace usbip
