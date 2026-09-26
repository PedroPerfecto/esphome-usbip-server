#pragma once
#include <cstdint>
#include <cstddef>
#include "usbip_wire.h"

// Assembling USB/IP messages from the TCP stream (no ESPHome dependencies).
// TCP delivers data in arbitrary pieces; the parser keeps the partial
// message and returns it only once it is complete.
//
// Modes:
//  - before import: OP_REQ_DEVLIST (8 B) or OP_REQ_IMPORT (8 + 32 B)
//  - after import (set_imported()): 48 B URB header, for CMD_SUBMIT OUT
//    followed by transfer_buffer_length bytes of data
// After an error the parser stays in the error state; the connection must be closed.
namespace usbip {

enum class MsgType : uint8_t {
  kNone,        // message not complete yet
  kReqDevlist,
  kReqImport,
  kCmdSubmit,
  kCmdUnlink,
  kError,
};

enum class ParseError : uint8_t {
  kNone,
  kBadVersion,       // OP header with a version other than 0x0111
  kUnknownOp,        // unknown OP code before import
  kUnknownCommand,   // unknown URB command after import
  kPayloadTooLarge,  // OUT data larger than the buffer
  kIsoUnsupported,   // isochronous transfers are not supported
  kBadBusId,         // busid in OP_REQ_IMPORT without a terminating null
};

struct Message {
  MsgType type = MsgType::kNone;
  ParseError error = ParseError::kNone;
  char busid[kBusIdSize] = {0};  // kReqImport
  CmdSubmit submit;              // kCmdSubmit
  CmdUnlink unlink;              // kCmdUnlink
  const uint8_t *payload = nullptr;  // kCmdSubmit OUT: points into the parser buffer,
  uint32_t payload_len = 0;          // valid until the next feed() call
};

class StreamParser {
 public:
  // payload_buf: buffer for CMD_SUBMIT OUT data, owned by the caller (no allocation).
  StreamParser(uint8_t *payload_buf, size_t payload_cap);

  // Processes data. Consumes at most one complete message and returns the number
  // of bytes consumed. If a message was completed, out.type != kNone.
  // If unconsumed bytes remain (next message), call again with the rest.
  size_t feed(const uint8_t *data, size_t len, Message &out);

  void set_imported();  // after sending a successful OP_REP_IMPORT
  bool imported() const { return imported_; }
  bool failed() const { return error_ != ParseError::kNone; }
  void reset();         // new connection

 private:
  enum class Stage : uint8_t { kHeader, kImportBody, kPayload };

  void fail_(ParseError e, Message &out);
  bool on_header_complete_(Message &out);

  uint8_t *payload_buf_;
  size_t payload_cap_;

  uint8_t hdr_[kUrbHeaderSize] = {0};
  size_t hdr_have_ = 0;
  uint8_t busid_[kBusIdSize] = {0};
  size_t busid_have_ = 0;
  size_t payload_have_ = 0;
  CmdSubmit pending_submit_;

  Stage stage_ = Stage::kHeader;
  bool imported_ = false;
  ParseError error_ = ParseError::kNone;
};

}  // namespace usbip
