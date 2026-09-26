#include "usbip_stream.h"
#include <algorithm>
#include <cstring>

namespace usbip {

StreamParser::StreamParser(uint8_t *payload_buf, size_t payload_cap)
    : payload_buf_(payload_buf), payload_cap_(payload_cap) {}

void StreamParser::reset() {
  hdr_have_ = 0;
  busid_have_ = 0;
  payload_have_ = 0;
  stage_ = Stage::kHeader;
  imported_ = false;
  error_ = ParseError::kNone;
}

void StreamParser::set_imported() {
  imported_ = true;
  hdr_have_ = 0;
  stage_ = Stage::kHeader;
}

void StreamParser::fail_(ParseError e, Message &out) {
  error_ = e;
  out.type = MsgType::kError;
  out.error = e;
}

// Header is complete. Returns true if the message is already complete now.
bool StreamParser::on_header_complete_(Message &out) {
  if (!imported_) {
    OpHeader h = decode_op_header(hdr_);
    if (h.version != kVersion) {
      fail_(ParseError::kBadVersion, out);
      return true;
    }
    if (h.code == kOpReqDevlist) {
      out.type = MsgType::kReqDevlist;
      hdr_have_ = 0;
      return true;
    }
    if (h.code == kOpReqImport) {
      stage_ = Stage::kImportBody;
      busid_have_ = 0;
      return false;
    }
    fail_(ParseError::kUnknownOp, out);
    return true;
  }

  uint32_t cmd = peek_urb_command(hdr_);
  if (cmd == kCmdUnlink) {
    out.type = MsgType::kCmdUnlink;
    out.unlink = decode_cmd_unlink(hdr_);
    hdr_have_ = 0;
    return true;
  }
  if (cmd == kCmdSubmit) {
    CmdSubmit s = decode_cmd_submit(hdr_);
    if (s.number_of_packets != 0 && s.number_of_packets != 0xffffffff) {
      fail_(ParseError::kIsoUnsupported, out);
      return true;
    }
    hdr_have_ = 0;
    if (s.base.direction == kDirOut && s.transfer_buffer_length > 0) {
      if (s.transfer_buffer_length > payload_cap_) {
        fail_(ParseError::kPayloadTooLarge, out);
        return true;
      }
      pending_submit_ = s;
      payload_have_ = 0;
      stage_ = Stage::kPayload;
      return false;
    }
    out.type = MsgType::kCmdSubmit;
    out.submit = s;
    return true;
  }
  fail_(ParseError::kUnknownCommand, out);
  return true;
}

size_t StreamParser::feed(const uint8_t *data, size_t len, Message &out) {
  out = Message();
  if (error_ != ParseError::kNone) {
    out.type = MsgType::kError;
    out.error = error_;
    return 0;
  }

  size_t used = 0;
  while (used < len) {
    const uint8_t *p = data + used;
    size_t avail = len - used;

    if (stage_ == Stage::kHeader) {
      size_t hdr_size = imported_ ? kUrbHeaderSize : kOpHeaderSize;
      size_t take = std::min(avail, hdr_size - hdr_have_);
      std::memcpy(hdr_ + hdr_have_, p, take);
      hdr_have_ += take;
      used += take;
      if (hdr_have_ == hdr_size) {
        if (on_header_complete_(out)) return used;
      }
    } else if (stage_ == Stage::kImportBody) {
      size_t take = std::min(avail, kBusIdSize - busid_have_);
      std::memcpy(busid_ + busid_have_, p, take);
      busid_have_ += take;
      used += take;
      if (busid_have_ == kBusIdSize) {
        stage_ = Stage::kHeader;
        hdr_have_ = 0;
        if (std::memchr(busid_, 0, kBusIdSize) == nullptr) {
          fail_(ParseError::kBadBusId, out);
          return used;
        }
        std::memcpy(out.busid, busid_, kBusIdSize);
        out.type = MsgType::kReqImport;
        return used;
      }
    } else {  // kPayload
      size_t need = pending_submit_.transfer_buffer_length - payload_have_;
      size_t take = std::min(avail, need);
      std::memcpy(payload_buf_ + payload_have_, p, take);
      payload_have_ += take;
      used += take;
      if (payload_have_ == pending_submit_.transfer_buffer_length) {
        stage_ = Stage::kHeader;
        out.type = MsgType::kCmdSubmit;
        out.submit = pending_submit_;
        out.payload = payload_buf_;
        out.payload_len = pending_submit_.transfer_buffer_length;
        return used;
      }
    }
  }
  return used;
}

}  // namespace usbip
