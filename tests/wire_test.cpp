#include "doctest.h"
#include "../components/usbip_server/usbip_wire.h"
#include <cstring>
#include <string>
#include <vector>

using namespace usbip;

namespace {
// Hex string (spaces ignored) -> bytes.
std::vector<uint8_t> hex(const std::string &s) {
  std::string clean;
  for (char c : s)
    if (c != ' ') clean += c;
  std::vector<uint8_t> out(clean.size() / 2);
  for (size_t i = 0; i < out.size(); ++i) out[i] = static_cast<uint8_t>(std::stoul(clean.substr(i * 2, 2), nullptr, 16));
  return out;
}
}  // namespace

// Vectors from the EXAMPLE section of Linux Documentation/usb/usbip_protocol.rst
static const char *kCmdIntrIn =
    "00000001 00000d05 0001000f 00000001 00000001 00000200 00000040 ffffffff 00000000 00000004 00000000 00000000";
static const char *kCmdIntrOut =
    "00000001 00000d06 0001000f 00000000 00000001 00000000 00000040 ffffffff 00000000 00000004 00000000 00000000";
static const char *kRetIntrOut =
    "00000003 00000d06 00000000 00000000 00000000 00000000 00000040 ffffffff 00000000 00000000 00000000 00000000";
static const char *kRetIntrIn =
    "00000003 00000d05 00000000 00000000 00000000 00000000 00000040 ffffffff 00000000 00000000 00000000 00000000";

TEST_CASE("big-endian helpers roundtrip") {
  uint8_t b[4];
  put_u32(b, 0x01020304);
  CHECK(b[0] == 0x01);
  CHECK(b[3] == 0x04);
  CHECK(get_u32(b) == 0x01020304);
  put_u16(b, 0x8005);
  CHECK(b[0] == 0x80);
  CHECK(b[1] == 0x05);
  CHECK(get_u16(b) == 0x8005);
}

TEST_CASE("OP header: OP_REQ_DEVLIST decode") {
  auto b = hex("0111 8005 00000000");
  OpHeader h = decode_op_header(b.data());
  CHECK(h.version == kVersion);
  CHECK(h.code == kOpReqDevlist);
  CHECK(h.status == 0);
}

TEST_CASE("OP header: encode OP_REP_IMPORT error") {
  uint8_t out[kOpHeaderSize];
  encode_op_header({kVersion, kOpRepImport, 1}, out);
  auto expected = hex("0111 0003 00000001");
  CHECK(std::memcmp(out, expected.data(), kOpHeaderSize) == 0);
}

TEST_CASE("CMD_SUBMIT interrupt IN from kernel example") {
  auto b = hex(kCmdIntrIn);
  REQUIRE(b.size() == kUrbHeaderSize);
  CHECK(peek_urb_command(b.data()) == kCmdSubmit);
  CmdSubmit c = decode_cmd_submit(b.data());
  CHECK(c.base.seqnum == 0x0d05);
  CHECK(c.base.devid == 0x0001000f);  // busnum 1, devnum 15
  CHECK(c.base.direction == kDirIn);
  CHECK(c.base.ep == 1);
  CHECK(c.transfer_flags == 0x200);
  CHECK(c.transfer_buffer_length == 64);
  CHECK(c.start_frame == 0xffffffff);
  CHECK(c.number_of_packets == 0);
  CHECK(c.interval == 4);
  for (uint8_t s : c.setup) CHECK(s == 0);
}

TEST_CASE("CMD_SUBMIT interrupt OUT from kernel example") {
  auto b = hex(kCmdIntrOut);
  CmdSubmit c = decode_cmd_submit(b.data());
  CHECK(c.base.seqnum == 0x0d06);
  CHECK(c.base.direction == kDirOut);
  CHECK(c.base.ep == 1);
  CHECK(c.transfer_buffer_length == 64);
}

TEST_CASE("RET_SUBMIT for interrupt IN is byte-identical to kernel example") {
  auto cmd = decode_cmd_submit(hex(kCmdIntrIn).data());
  RetSubmit r;
  r.seqnum = cmd.base.seqnum;
  r.status = 0;
  r.actual_length = 64;
  r.start_frame = cmd.start_frame;
  r.number_of_packets = cmd.number_of_packets;
  uint8_t out[kUrbHeaderSize];
  encode_ret_submit(r, out);
  auto expected = hex(kRetIntrIn);
  CHECK(std::memcmp(out, expected.data(), kUrbHeaderSize) == 0);
}

TEST_CASE("RET_SUBMIT for interrupt OUT is byte-identical to kernel example") {
  auto cmd = decode_cmd_submit(hex(kCmdIntrOut).data());
  RetSubmit r;
  r.seqnum = cmd.base.seqnum;
  r.actual_length = 64;
  r.start_frame = cmd.start_frame;
  r.number_of_packets = cmd.number_of_packets;
  uint8_t out[kUrbHeaderSize];
  encode_ret_submit(r, out);
  auto expected = hex(kRetIntrOut);
  CHECK(std::memcmp(out, expected.data(), kUrbHeaderSize) == 0);
}

TEST_CASE("CMD_UNLINK decode and RET_UNLINK encode with -ECONNRESET") {
  auto b = hex("00000002 00000010 0001000f 00000000 00000000 0000000c"
               "00000000 00000000 00000000 00000000 00000000 00000000");
  REQUIRE(b.size() == kUrbHeaderSize);
  CHECK(peek_urb_command(b.data()) == kCmdUnlink);
  CmdUnlink u = decode_cmd_unlink(b.data());
  CHECK(u.base.seqnum == 0x10);
  CHECK(u.unlink_seqnum == 0x0c);

  uint8_t out[kUrbHeaderSize];
  encode_ret_unlink(u.base.seqnum, -104, out);  // -ECONNRESET (Linux: ECONNRESET = 104)
  CHECK(get_u32(out + 0x00) == kRetUnlink);
  CHECK(get_u32(out + 0x04) == 0x10);
  CHECK(get_u32(out + 0x08) == 0);
  CHECK(static_cast<int32_t>(get_u32(out + 0x14)) == -104);
  for (size_t i = 0x18; i < kUrbHeaderSize; ++i) CHECK(out[i] == 0);
}
