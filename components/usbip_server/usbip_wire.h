#pragma once
#include <cstdint>
#include <cstddef>

// Encoding/decoding of USB/IP messages (no ESPHome dependencies).
// Format source: Linux Documentation/usb/usbip_protocol.rst (version 1.1.1).
// All fields on the wire are big-endian.
namespace usbip {

constexpr uint16_t kVersion = 0x0111;

constexpr uint16_t kOpReqDevlist = 0x8005;
constexpr uint16_t kOpRepDevlist = 0x0005;
constexpr uint16_t kOpReqImport = 0x8003;
constexpr uint16_t kOpRepImport = 0x0003;

constexpr uint32_t kCmdSubmit = 0x00000001;
constexpr uint32_t kCmdUnlink = 0x00000002;
constexpr uint32_t kRetSubmit = 0x00000003;
constexpr uint32_t kRetUnlink = 0x00000004;

constexpr uint32_t kDirOut = 0;
constexpr uint32_t kDirIn = 1;

constexpr size_t kOpHeaderSize = 8;
constexpr size_t kBusIdSize = 32;
constexpr size_t kUrbHeaderSize = 48;

// --- big-endian helpers ---
void put_u16(uint8_t *p, uint16_t v);
void put_u32(uint8_t *p, uint32_t v);
uint16_t get_u16(const uint8_t *p);
uint32_t get_u32(const uint8_t *p);

// --- OP_* header (8 B): version, code, status ---
struct OpHeader {
  uint16_t version = 0;
  uint16_t code = 0;
  uint32_t status = 0;
};
void encode_op_header(const OpHeader &h, uint8_t out[kOpHeaderSize]);
OpHeader decode_op_header(const uint8_t in[kOpHeaderSize]);

// --- usbip_header_basic (first 20 B of the 48 B URB header) ---
struct UrbHeaderBasic {
  uint32_t command = 0;
  uint32_t seqnum = 0;
  uint32_t devid = 0;
  uint32_t direction = 0;
  uint32_t ep = 0;
};

// USBIP_CMD_SUBMIT (48 B header, followed by OUT data)
struct CmdSubmit {
  UrbHeaderBasic base;
  uint32_t transfer_flags = 0;
  uint32_t transfer_buffer_length = 0;
  uint32_t start_frame = 0;
  uint32_t number_of_packets = 0;
  uint32_t interval = 0;
  uint8_t setup[8] = {0};
};

// USBIP_CMD_UNLINK (48 B)
struct CmdUnlink {
  UrbHeaderBasic base;
  uint32_t unlink_seqnum = 0;
};

// Reads only the command from the URB header (first 4 B) so the caller knows
// which decoder to use.
uint32_t peek_urb_command(const uint8_t in[kUrbHeaderSize]);
CmdSubmit decode_cmd_submit(const uint8_t in[kUrbHeaderSize]);
CmdUnlink decode_cmd_unlink(const uint8_t in[kUrbHeaderSize]);

// USBIP_RET_SUBMIT (48 B header, followed by IN data).
// devid/direction/ep = 0 (server). start_frame and number_of_packets are
// returned as they arrived in CMD_SUBMIT - the Linux stub does the same
// (usbip_pack_cmd_submit stores them in the URB, usbip_pack_ret_submit returns them)
// and it matches the RetIntrIn/RetIntrOut example in usbip_protocol.rst.
struct RetSubmit {
  uint32_t seqnum = 0;
  int32_t status = 0;
  uint32_t actual_length = 0;
  uint32_t start_frame = 0;
  uint32_t number_of_packets = 0;
  uint32_t error_count = 0;
};
void encode_ret_submit(const RetSubmit &r, uint8_t out[kUrbHeaderSize]);

// USBIP_RET_UNLINK (48 B)
void encode_ret_unlink(uint32_t seqnum, int32_t status, uint8_t out[kUrbHeaderSize]);

}  // namespace usbip
