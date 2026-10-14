#include "usbip_intrq.h"
#include "usbip_time.h"
#include <algorithm>
#include <cstring>
#include <new>

namespace usbip {

uint8_t clamp_prefetch_transfers(uint8_t transfers) {
  return std::min<uint8_t>(std::max<uint8_t>(transfers, 1), kMaxPrefetchTransfers);
}

bool prefetch_ring_full(uint16_t size, uint16_t depth, uint16_t reserved) {
  return static_cast<uint32_t>(size) + reserved >= depth;
}

// ---------------------------------------------------------------- IntrRing

void *IntrRing::alloc_(size_t size) {
  if (this->allocator_.alloc != nullptr) return this->allocator_.alloc(size);
  return new (std::nothrow) uint8_t[size];
}

void IntrRing::free_(void *ptr) {
  if (ptr == nullptr) return;
  if (this->allocator_.free != nullptr) {
    this->allocator_.free(ptr);
  } else {
    delete[] static_cast<uint8_t *>(ptr);
  }
}

void IntrRing::release_() {
  this->free_(this->data_);
  this->free_(this->lens_);
  this->data_ = nullptr;
  this->lens_ = nullptr;
  this->depth_ = 0;
  this->mps_ = 0;
  this->clear();
}

bool IntrRing::init(uint16_t depth, uint16_t mps, IntrAllocator allocator) {
  this->release_();
  this->allocator_ = (allocator.alloc != nullptr && allocator.free != nullptr) ? allocator : IntrAllocator{};
  if (mps == 0) return false;
  depth = std::min<uint16_t>(std::max<uint16_t>(depth, kMinDepth), kMaxDepth);
  this->data_ = static_cast<uint8_t *>(this->alloc_(static_cast<size_t>(depth) * mps));
  this->lens_ = static_cast<uint16_t *>(this->alloc_(static_cast<size_t>(depth) * sizeof(uint16_t)));
  if (this->data_ == nullptr || this->lens_ == nullptr) {
    this->release_();
    return false;
  }
  this->depth_ = depth;
  this->mps_ = mps;
  return true;
}

bool IntrRing::push(const uint8_t *data, uint16_t len) {
  if (this->data_ == nullptr || this->full() || len > this->mps_) return false;
  const uint16_t s = this->slot_(this->count_);
  if (len > 0) memcpy(this->data_ + static_cast<size_t>(s) * this->mps_, data, len);
  this->lens_[s] = len;
  ++this->count_;
  if (this->count_ > this->peak_) this->peak_ = this->count_;
  return true;
}

bool IntrRing::pop() {
  if (this->count_ == 0) return false;
  this->head_ = this->slot_(1);
  --this->count_;
  return true;
}

uint16_t IntrRing::len_at(uint16_t i) const { return this->lens_[this->slot_(i)]; }

const uint8_t *IntrRing::data_at(uint16_t i) const {
  return this->data_ + static_cast<size_t>(this->slot_(i)) * this->mps_;
}

void IntrRing::clear() {
  this->head_ = 0;
  this->count_ = 0;
  this->peak_ = 0;
}

uint16_t IntrRing::take_peak() {
  const uint16_t p = this->peak_;
  this->peak_ = this->count_;
  return p;
}

// ---------------------------------------------------------------- IntrWaitFifo

bool IntrWaitFifo::push(uint32_t seqnum, uint32_t length, uint32_t start_frame, uint32_t number_of_packets) {
  if (this->full()) return false;
  this->items_[this->count_].seqnum = seqnum;
  this->items_[this->count_].length = length;
  this->items_[this->count_].start_frame = start_frame;
  this->items_[this->count_].number_of_packets = number_of_packets;
  ++this->count_;
  return true;
}

void IntrWaitFifo::pop_front() {
  if (this->count_ == 0) return;
  std::copy(this->items_ + 1, this->items_ + this->count_, this->items_);
  --this->count_;
}

bool IntrWaitFifo::remove(uint32_t seqnum) {
  for (uint8_t i = 0; i < this->count_; ++i) {
    if (this->items_[i].seqnum != seqnum) continue;
    std::copy(this->items_ + i + 1, this->items_ + this->count_, this->items_ + i);
    --this->count_;
    return true;
  }
  return false;
}

bool IntrWaitFifo::contains(uint32_t seqnum) const {
  for (uint8_t i = 0; i < this->count_; ++i) {
    if (this->items_[i].seqnum == seqnum) return true;
  }
  return false;
}

// ---------------------------------------------------------------- serving

IntrServePlan plan_serve(const IntrRing &ring, uint32_t requested) {
  IntrServePlan p;
  if (requested == 0) {
    // Not established what a real host controller does with a zero-length interrupt IN URB
    // (Linux usbhid never submits one); completed at once without touching the ring.
    p.ready = true;
    p.zero_request = true;
    return p;
  }
  const uint16_t mps = ring.mps();
  uint32_t got = 0;
  for (uint16_t i = 0; i < ring.size(); ++i) {
    const uint16_t len = ring.len_at(i);
    const uint32_t remaining = requested - got;
    if (len > remaining) {
      p.ready = true;
      p.overflow = true;
      p.length = requested;
      p.packets = static_cast<uint16_t>(i + 1);
      return p;
    }
    got += len;
    if (len < mps) {
      p.ready = true;
      p.short_packet = true;
      p.length = got;
      p.packets = static_cast<uint16_t>(i + 1);
      return p;
    }
    if (got == requested) {
      p.ready = true;
      p.filled = true;
      p.length = got;
      p.packets = static_cast<uint16_t>(i + 1);
      return p;
    }
  }
  // Only full MPS packets so far and still room in the request: a real URB keeps waiting.
  if (ring.full() && ring.size() > 0) {
    p.ready = true;
    p.ring_limit = true;
    p.length = got;
    p.packets = ring.size();
  }
  return p;
}

void consume(IntrRing &ring, const IntrServePlan &plan, uint8_t *out) {
  uint32_t off = 0;
  for (uint16_t i = 0; i < plan.packets && !ring.empty(); ++i) {
    const uint32_t take = std::min<uint32_t>(ring.len_at(0), plan.length - off);
    if (take > 0 && out != nullptr) memcpy(out + off, ring.data_at(0), take);
    off += take;
    ring.pop();
  }
}

bool prefetch_allowed(bool session_active, bool ring_full, bool request_waiting, bool have_last_request,
                      uint32_t last_request_ms, uint32_t now_ms, uint32_t window_ms) {
  if (!session_active || ring_full) return false;
  if (request_waiting) return true;
  return have_last_request && elapsed32(now_ms, last_request_ms) < window_ms;
}

// ---------------------------------------------------------------- IntrInQueue

bool IntrInQueue::add_request(uint32_t seqnum, uint32_t length, uint32_t now_ms, uint32_t start_frame,
                              uint32_t number_of_packets) {
  if (!this->waiting_.push(seqnum, length, start_frame, number_of_packets)) return false;
  this->last_request_ms_ = now_ms;
  this->have_last_request_ = true;
  return true;
}

bool IntrInQueue::pop_request(IntrRequest *out) {
  if (this->waiting_.empty()) return false;
  if (out != nullptr) *out = this->waiting_.front();
  this->waiting_.pop_front();
  return true;
}

bool IntrInQueue::front_ready(uint32_t *seqnum, IntrServePlan *plan) const {
  if (this->waiting_.empty()) return false;
  const IntrRequest &r = this->waiting_.front();
  const IntrServePlan p = plan_serve(this->ring_, r.length);
  if (!p.ready) return false;
  if (seqnum != nullptr) *seqnum = r.seqnum;
  if (plan != nullptr) *plan = p;
  return true;
}

void IntrInQueue::complete_front(const IntrServePlan &plan, uint8_t *out) {
  consume(this->ring_, plan, out);
  this->waiting_.pop_front();
}

bool IntrInQueue::on_completion(const uint8_t *data, int actual_num_bytes) {
  if (actual_num_bytes <= 0) return this->ring_.push(nullptr, 0);
  const uint16_t mps = this->ring_.mps();
  return this->ring_.push(data, actual_num_bytes > mps ? mps : static_cast<uint16_t>(actual_num_bytes));
}

void IntrInQueue::expire_window(uint32_t now_ms, uint32_t window_ms) {
  if (this->have_last_request_ && elapsed32(now_ms, this->last_request_ms_) >= window_ms) {
    this->have_last_request_ = false;
  }
}

bool IntrInQueue::should_prefetch(bool session_active, uint32_t now_ms, uint32_t window_ms,
                                  uint16_t reserved) const {
  const bool full = prefetch_ring_full(this->ring_.size(), this->ring_.depth(), reserved);
  return prefetch_allowed(session_active, full, !this->waiting_.empty(), this->have_last_request_,
                          this->last_request_ms_, now_ms, window_ms);
}

bool IntrInQueue::request_active(uint32_t now_ms, uint32_t window_ms) const {
  return prefetch_allowed(true, false, !this->waiting_.empty(), this->have_last_request_, this->last_request_ms_,
                          now_ms, window_ms);
}

void IntrInQueue::reset() {
  this->ring_.clear();
  this->waiting_.clear();
  this->last_request_ms_ = 0;
  this->have_last_request_ = false;
}

}  // namespace usbip
