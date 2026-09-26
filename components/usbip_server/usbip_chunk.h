#pragma once
#include <cstdint>

// Splitting a large bulk transfer into several consecutive USB transfers (no ESPHome
// dependencies). A bulk transfer is just a sequence of packets; splitting it into chunks
// that are multiples of MPS is equivalent to a single transfer, provided IN ends on a short
// packet (USB 2.0 5.8.3).
namespace usbip {

class ChunkPlan {
 public:
  // total: transfer_buffer_length from CMD_SUBMIT
  // cap:   largest single USB transfer (rounded down to a multiple of MPS, min. MPS)
  // zero_packet: client requests a zero-length packet (URB_ZERO_PACKET) - OUT only, last chunk only
  ChunkPlan(uint32_t total, uint32_t cap, uint16_t mps, bool dir_in, bool zero_packet);
  ChunkPlan() = default;

  uint32_t request_len() const;  // bytes of the current chunk (<= cap)
  uint32_t submit_len() const;   // num_bytes for ESP-IDF (IN rounded up to MPS)
  bool zero_pack() const;        // OUT: zero-length packet at the end of this chunk
  uint32_t offset() const { return done_; }  // where the current chunk starts in the overall buffer
  uint32_t done() const { return done_; }
  bool finished() const { return finished_; }
  uint32_t cap() const { return cap_; }

  // Chunk completed successfully with actual bytes transferred. Returns how many bytes
  // of the chunk belong to the result (<= request_len). After a short packet (IN) or
  // reaching total, sets finished().
  uint32_t complete(uint32_t actual);

 private:
  uint32_t total_{0};
  uint32_t cap_{0};
  uint32_t done_{0};
  uint16_t mps_{0};
  bool dir_in_{false};
  bool zero_packet_{false};
  bool finished_{false};
};

}  // namespace usbip
