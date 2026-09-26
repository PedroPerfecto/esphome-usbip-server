#pragma once
#include <cstdint>

// Recognizes control requests that the server must not forward to the device
// blindly (no ESPHome dependencies). Conditions match is_*_cmd() 1:1 in
// Linux drivers/usb/usbip/stub_rx.c, constants from include/uapi/linux/usb/ch9.h
// and ch11.h.
namespace usbip {

// Standard requests (ch9.h)
constexpr uint8_t kReqClearFeature = 0x01;
constexpr uint8_t kReqSetFeature = 0x03;
constexpr uint8_t kReqSetConfiguration = 0x09;
constexpr uint8_t kReqSetInterface = 0x0B;

// bmRequestType (ch9.h, ch11.h)
constexpr uint8_t kRecipDevice = 0x00;
constexpr uint8_t kRecipInterface = 0x01;
constexpr uint8_t kRecipEndpoint = 0x02;
constexpr uint8_t kRtPort = 0x23;  // USB_TYPE_CLASS | USB_RECIP_OTHER

constexpr uint16_t kFeatEndpointHalt = 0;  // USB_ENDPOINT_HALT
constexpr uint16_t kFeatPortReset = 4;     // USB_PORT_FEAT_RESET

enum class ControlKind : uint8_t {
  kForward,           // ordinary request, send to the device
  kClearHalt,         // CLEAR_FEATURE(ENDPOINT_HALT) on an endpoint
  kSetInterface,      // SET_INTERFACE(interface, alternate)
  kSetConfiguration,  // SET_CONFIGURATION(value)
  kResetDevice,       // SET_FEATURE(PORT_RESET) on a port
};

struct ControlAction {
  ControlKind kind = ControlKind::kForward;
  uint8_t endpoint = 0;       // kClearHalt: endpoint address including direction (wIndex & 0x8F)
  uint8_t interface = 0;      // kSetInterface
  uint8_t alternate = 0;      // kSetInterface
  uint8_t configuration = 0;  // kSetConfiguration
};

// setup: 8 B setup packet (bmRequestType, bRequest, wValue LE, wIndex LE, wLength LE)
ControlAction classify_control(const uint8_t setup[8]);

}  // namespace usbip
