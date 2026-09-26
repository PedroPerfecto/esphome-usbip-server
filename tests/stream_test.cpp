#include "doctest.h"
#include "../components/usbip_server/usbip_stream.h"
#include <cstring>
#include <string>
#include <vector>

using namespace usbip;

namespace {

std::vector<uint8_t> hex(const std::string &s) {
  std::string clean;
  for (char c : s)
    if (c != ' ') clean += c;
  std::vector<uint8_t> out(clean.size() / 2);
  for (size_t i = 0; i < out.size(); ++i) out[i] = static_cast<uint8_t>(std::stoul(clean.substr(i * 2, 2), nullptr, 16));
  return out;
}

std::vector<uint8_t> req_import(const char *busid) {
  std::vector<uint8_t> v = hex("0111 8003 00000000");
  v.resize(8 + kBusIdSize, 0);
  std::memcpy(v.data() + 8, busid, std::strlen(busid));
  return v;
}

// From the vectors in Documentation/usb/usbip_protocol.rst (EXAMPLE)
const char *kCmdIntrIn =
    "00000001 00000d05 0001000f 00000001 00000001 00000200 00000040 ffffffff 00000000 00000004 00000000 00000000";
const char *kCmdIntrOutHdr =
    "00000001 00000d06 0001000f 00000000 00000001 00000000 00000040 ffffffff 00000000 00000004 00000000 00000000";
const char *kCmdIntrOutData =
    "ffffffff860008a784ce5ae2123763000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000";

// Sends the whole input in pieces of size `chunk` and collects the finished messages.
// After a successful REQ_IMPORT switches the parser to post-import mode (like the server).
std::vector<Message> run(StreamParser &p, const std::vector<uint8_t> &in, size_t chunk,
                         std::vector<std::vector<uint8_t>> *payloads = nullptr) {
  std::vector<Message> msgs;
  size_t pos = 0;
  while (pos < in.size()) {
    size_t n = std::min(chunk, in.size() - pos);
    size_t off = 0;
    while (off < n) {
      Message m;
      size_t used = p.feed(in.data() + pos + off, n - off, m);
      if (m.type != MsgType::kNone) {
        msgs.push_back(m);
        if (payloads) payloads->push_back(std::vector<uint8_t>(m.payload, m.payload + m.payload_len));
        if (m.type == MsgType::kReqImport) p.set_imported();
        if (m.type == MsgType::kError) return msgs;
      }
      if (used == 0) return msgs;
      off += used;
    }
    pos += n;
  }
  return msgs;
}

}  // namespace

TEST_CASE("OP_REQ_DEVLIST in one piece and byte by byte") {
  uint8_t buf[64];
  for (size_t chunk : {size_t(8), size_t(1), size_t(3)}) {
    StreamParser p(buf, sizeof(buf));
    auto msgs = run(p, hex("0111 8005 00000000"), chunk);
    REQUIRE(msgs.size() == 1);
    CHECK(msgs[0].type == MsgType::kReqDevlist);
    CHECK_FALSE(p.imported());
  }
}

TEST_CASE("OP_REQ_IMPORT byte by byte gives busid") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  auto msgs = run(p, req_import("1-1"), 1);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].type == MsgType::kReqImport);
  CHECK(std::string(msgs[0].busid) == "1-1");
  CHECK(p.imported());
}

TEST_CASE("wrong version is an error and parser stays failed") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  auto msgs = run(p, hex("0106 8005 00000000"), 8);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].type == MsgType::kError);
  CHECK(msgs[0].error == ParseError::kBadVersion);
  Message m;
  CHECK(p.feed(hex("0111 8005 00000000").data(), 8, m) == 0);
  CHECK(m.type == MsgType::kError);
}

TEST_CASE("unknown OP code is an error") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  auto msgs = run(p, hex("0111 1234 00000000"), 8);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].error == ParseError::kUnknownOp);
}

TEST_CASE("busid without NUL terminator is an error") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  std::vector<uint8_t> v = hex("0111 8003 00000000");
  v.resize(8 + kBusIdSize, 'x');
  auto msgs = run(p, v, 40);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].error == ParseError::kBadBusId);
}

TEST_CASE("import then kernel example IN + OUT in every chunk size") {
  auto in_hdr = hex(kCmdIntrIn);
  auto out_hdr = hex(kCmdIntrOutHdr);
  auto out_data = hex(kCmdIntrOutData);
  REQUIRE(out_data.size() == 64);

  std::vector<uint8_t> stream = req_import("1-1");
  stream.insert(stream.end(), in_hdr.begin(), in_hdr.end());
  stream.insert(stream.end(), out_hdr.begin(), out_hdr.end());
  stream.insert(stream.end(), out_data.begin(), out_data.end());

  for (size_t chunk = 1; chunk <= stream.size(); ++chunk) {
    CAPTURE(chunk);
    uint8_t buf[64];
    StreamParser p(buf, sizeof(buf));
    std::vector<std::vector<uint8_t>> payloads;
    auto msgs = run(p, stream, chunk, &payloads);
    REQUIRE(msgs.size() == 3);
    CHECK(msgs[0].type == MsgType::kReqImport);
    CHECK(msgs[1].type == MsgType::kCmdSubmit);
    CHECK(msgs[1].submit.base.seqnum == 0x0d05);
    CHECK(msgs[1].submit.base.direction == kDirIn);
    CHECK(msgs[1].payload_len == 0);
    CHECK(msgs[2].type == MsgType::kCmdSubmit);
    CHECK(msgs[2].submit.base.seqnum == 0x0d06);
    CHECK(msgs[2].payload_len == 64);
    CHECK(payloads[2] == out_data);
  }
}

TEST_CASE("CMD_UNLINK after import") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  std::vector<uint8_t> stream = req_import("1-1");
  auto u = hex("00000002 00000010 0001000f 00000000 00000000 0000000c"
               "00000000 00000000 00000000 00000000 00000000 00000000");
  stream.insert(stream.end(), u.begin(), u.end());
  auto msgs = run(p, stream, 7);
  REQUIRE(msgs.size() == 2);
  CHECK(msgs[1].type == MsgType::kCmdUnlink);
  CHECK(msgs[1].unlink.base.seqnum == 0x10);
  CHECK(msgs[1].unlink.unlink_seqnum == 0x0c);
}

TEST_CASE("OUT payload larger than buffer is an error") {
  uint8_t buf[32];  // smaller than the 64 B from the example
  StreamParser p(buf, sizeof(buf));
  std::vector<uint8_t> stream = req_import("1-1");
  auto out_hdr = hex(kCmdIntrOutHdr);
  stream.insert(stream.end(), out_hdr.begin(), out_hdr.end());
  auto msgs = run(p, stream, 1000);
  REQUIRE(msgs.size() == 2);
  CHECK(msgs[1].type == MsgType::kError);
  CHECK(msgs[1].error == ParseError::kPayloadTooLarge);
}

TEST_CASE("ISO submit is rejected") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  std::vector<uint8_t> stream = req_import("1-1");
  auto iso = hex("00000001 00000001 0001000f 00000001 00000003 00000000 00000040 00000000 00000004 00000001 00000000 00000000");
  stream.insert(stream.end(), iso.begin(), iso.end());
  auto msgs = run(p, stream, 1000);
  REQUIRE(msgs.size() == 2);
  CHECK(msgs[1].error == ParseError::kIsoUnsupported);
}

TEST_CASE("unknown URB command is an error") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  std::vector<uint8_t> stream = req_import("1-1");
  std::vector<uint8_t> bad(kUrbHeaderSize, 0);
  bad[3] = 0x07;
  stream.insert(stream.end(), bad.begin(), bad.end());
  auto msgs = run(p, stream, 1000);
  REQUIRE(msgs.size() == 2);
  CHECK(msgs[1].error == ParseError::kUnknownCommand);
}

TEST_CASE("reset() allows a new connection after error") {
  uint8_t buf[64];
  StreamParser p(buf, sizeof(buf));
  run(p, hex("0106 8005 00000000"), 8);
  REQUIRE(p.failed());
  p.reset();
  CHECK_FALSE(p.failed());
  auto msgs = run(p, hex("0111 8005 00000000"), 8);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].type == MsgType::kReqDevlist);
}
