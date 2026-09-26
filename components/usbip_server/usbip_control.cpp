#include "usbip_control.h"

namespace usbip {

static uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

ControlAction classify_control(const uint8_t setup[8]) {
  ControlAction a;
  const uint8_t type = setup[0];
  const uint8_t req = setup[1];
  const uint16_t value = le16(setup + 2);
  const uint16_t index = le16(setup + 4);

  // is_clear_halt_cmd
  if (req == kReqClearFeature && type == kRecipEndpoint && value == kFeatEndpointHalt) {
    a.kind = ControlKind::kClearHalt;
    a.endpoint = static_cast<uint8_t>(index & 0x008F);  // number (0x0F) + direction (0x80)
    return a;
  }
  // is_set_interface_cmd
  if (req == kReqSetInterface && type == kRecipInterface) {
    a.kind = ControlKind::kSetInterface;
    a.interface = static_cast<uint8_t>(index);
    a.alternate = static_cast<uint8_t>(value);
    return a;
  }
  // is_set_configuration_cmd
  if (req == kReqSetConfiguration && type == kRecipDevice) {
    a.kind = ControlKind::kSetConfiguration;
    a.configuration = static_cast<uint8_t>(value);
    return a;
  }
  // is_reset_device_cmd
  if (req == kReqSetFeature && type == kRtPort && value == kFeatPortReset) {
    a.kind = ControlKind::kResetDevice;
    return a;
  }
  return a;  // kForward
}

}  // namespace usbip
