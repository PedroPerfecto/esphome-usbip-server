#include "usbip_chunk.h"
#include "usbip_endpoints.h"

namespace usbip {

ChunkPlan::ChunkPlan(uint32_t total, uint32_t cap, uint16_t mps, bool dir_in, bool zero_packet)
    : total_(total), mps_(mps), dir_in_(dir_in), zero_packet_(zero_packet) {
  if (mps_ > 0) {
    cap = cap - cap % mps_;
    if (cap < mps_) cap = mps_;
  }
  cap_ = cap;
  finished_ = total_ == 0;
}

uint32_t ChunkPlan::request_len() const {
  uint32_t left = total_ - done_;
  return left < cap_ ? left : cap_;
}

uint32_t ChunkPlan::submit_len() const {
  uint32_t r = request_len();
  return dir_in_ ? round_up_to_mps(r, mps_) : r;
}

bool ChunkPlan::zero_pack() const { return !dir_in_ && zero_packet_ && done_ + request_len() == total_; }

uint32_t ChunkPlan::complete(uint32_t actual) {
  const uint32_t req = request_len();
  const uint32_t accepted = actual < req ? actual : req;
  done_ += accepted;
  // IN: a short packet (less than requested) ends the transfer.
  // OUT: less than requested means an error - also the end.
  if (actual < req || done_ >= total_) finished_ = true;
  return accepted;
}

}  // namespace usbip
