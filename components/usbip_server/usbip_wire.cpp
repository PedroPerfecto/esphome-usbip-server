#include "usbip_wire.h"
#include <cstring>

namespace usbip {

void put_u16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v);
}

void put_u32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v >> 24);
  p[1] = static_cast<uint8_t>(v >> 16);
  p[2] = static_cast<uint8_t>(v >> 8);
  p[3] = static_cast<uint8_t>(v);
}

uint16_t get_u16(const uint8_t *p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }

uint32_t get_u32(const uint8_t *p) {
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

void encode_op_header(const OpHeader &h, uint8_t out[kOpHeaderSize]) {
  put_u16(out + 0, h.version);
  put_u16(out + 2, h.code);
  put_u32(out + 4, h.status);
}

OpHeader decode_op_header(const uint8_t in[kOpHeaderSize]) {
  OpHeader h;
  h.version = get_u16(in + 0);
  h.code = get_u16(in + 2);
  h.status = get_u32(in + 4);
  return h;
}

static UrbHeaderBasic decode_basic(const uint8_t *in) {
  UrbHeaderBasic b;
  b.command = get_u32(in + 0x00);
  b.seqnum = get_u32(in + 0x04);
  b.devid = get_u32(in + 0x08);
  b.direction = get_u32(in + 0x0C);
  b.ep = get_u32(in + 0x10);
  return b;
}

static void encode_basic_server(uint8_t *out, uint32_t command, uint32_t seqnum) {
  put_u32(out + 0x00, command);
  put_u32(out + 0x04, seqnum);
  put_u32(out + 0x08, 0);  // devid: server = 0
  put_u32(out + 0x0C, 0);  // direction: server = 0
  put_u32(out + 0x10, 0);  // ep: server = 0
}

uint32_t peek_urb_command(const uint8_t in[kUrbHeaderSize]) { return get_u32(in); }

CmdSubmit decode_cmd_submit(const uint8_t in[kUrbHeaderSize]) {
  CmdSubmit c;
  c.base = decode_basic(in);
  c.transfer_flags = get_u32(in + 0x14);
  c.transfer_buffer_length = get_u32(in + 0x18);
  c.start_frame = get_u32(in + 0x1C);
  c.number_of_packets = get_u32(in + 0x20);
  c.interval = get_u32(in + 0x24);
  std::memcpy(c.setup, in + 0x28, 8);
  return c;
}

CmdUnlink decode_cmd_unlink(const uint8_t in[kUrbHeaderSize]) {
  CmdUnlink c;
  c.base = decode_basic(in);
  c.unlink_seqnum = get_u32(in + 0x14);
  return c;
}

void encode_ret_submit(const RetSubmit &r, uint8_t out[kUrbHeaderSize]) {
  std::memset(out, 0, kUrbHeaderSize);
  encode_basic_server(out, kRetSubmit, r.seqnum);
  put_u32(out + 0x14, static_cast<uint32_t>(r.status));
  put_u32(out + 0x18, r.actual_length);
  put_u32(out + 0x1C, r.start_frame);
  put_u32(out + 0x20, r.number_of_packets);
  put_u32(out + 0x24, r.error_count);
  // 0x28..0x2F padding = 0
}

void encode_ret_unlink(uint32_t seqnum, int32_t status, uint8_t out[kUrbHeaderSize]) {
  std::memset(out, 0, kUrbHeaderSize);
  encode_basic_server(out, kRetUnlink, seqnum);
  put_u32(out + 0x14, static_cast<uint32_t>(status));
}

}  // namespace usbip
