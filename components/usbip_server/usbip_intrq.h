#pragma once
#include <cstddef>
#include <cstdint>

// Interrupt IN endpoints without HALT (no ESPHome/ESP-IDF dependencies).
//
// Halting a periodic (interrupt) pipe in ESP-IDF can wait forever for a halt interrupt that the
// DWC controller never raises (_pipe_cmd_halt -> _internal_pipe_event_wait). Instead of cancelling
// USB transfers on CMD_UNLINK, the server keeps its own prefetch transfers on the endpoint, stores
// received packets in IntrRing and answers the client's CMD_SUBMIT requests from it. A CMD_UNLINK
// then only removes a request from IntrWaitFifo and never touches the USB pipe.
namespace usbip {

// Prefetch storage is always sized for the maximum; only the configured slots are used.
static constexpr uint8_t kMaxPrefetchTransfers = 2;
uint8_t clamp_prefetch_transfers(uint8_t transfers);
// Each transfer whose completion has not been drained reserves one ring entry.
bool prefetch_ring_full(uint16_t size, uint16_t depth, uint16_t reserved);

// Storage allocator for IntrRing (the server passes one that uses PSRAM; the default uses new/delete).
struct IntrAllocator {
  void *(*alloc)(size_t size){nullptr};
  void (*free)(void *ptr){nullptr};
};

// Ring of received packets of one interrupt IN endpoint. Each entry is one USB packet (0..MPS
// bytes; a zero-length packet is stored as an entry of length 0). Storage is allocated once in
// init(): depth * MPS bytes of data plus one length per entry.
class IntrRing {
 public:
  static constexpr uint16_t kMinDepth = 4;
  static constexpr uint16_t kMaxDepth = 4096;
  static constexpr uint16_t kDefaultDepth = 32;

  IntrRing() = default;
  ~IntrRing() { this->release_(); }
  IntrRing(const IntrRing &) = delete;
  IntrRing &operator=(const IntrRing &) = delete;

  // depth is clamped to [kMinDepth, kMaxDepth]. Returns false on allocation failure or mps == 0.
  // allocator: where the storage (depth * MPS data bytes + depth lengths) goes; both functions
  // null = new/delete. Kept until the next init() or destruction.
  bool init(uint16_t depth, uint16_t mps, IntrAllocator allocator = {});
  bool initialized() const { return data_ != nullptr; }

  // Appends a copy of one packet. Fails (returns false, ring unchanged) if full or len > MPS.
  bool push(const uint8_t *data, uint16_t len);
  // Removes the oldest packet. Returns false if empty.
  bool pop();
  // i-th packet from the oldest (0 = oldest); i < size().
  uint16_t len_at(uint16_t i) const;
  const uint8_t *data_at(uint16_t i) const;

  void clear();
  uint16_t size() const { return count_; }
  uint16_t depth() const { return depth_; }
  uint16_t mps() const { return mps_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ == depth_; }

  // Highest size() since the last take_peak() (or init/clear). take_peak() returns it and
  // restarts the measurement from the current size.
  uint16_t peak() const { return peak_; }
  uint16_t take_peak();

 private:
  uint16_t slot_(uint16_t i) const { return static_cast<uint16_t>((head_ + i) % depth_); }

  void release_();
  void *alloc_(size_t size);
  void free_(void *ptr);

  IntrAllocator allocator_{};
  uint8_t *data_{nullptr};
  uint16_t *lens_{nullptr};
  uint16_t depth_{0};
  uint16_t mps_{0};
  uint16_t head_{0};
  uint16_t count_{0};
  uint16_t peak_{0};
};

// The client's IN requests (CMD_SUBMIT) waiting for data, oldest first.
struct IntrRequest {
  uint32_t seqnum{0};
  uint32_t length{0};  // transfer_buffer_length from CMD_SUBMIT
  // Echoed back in RET_SUBMIT, as for every other transfer.
  uint32_t start_frame{0};
  uint32_t number_of_packets{0};
};

class IntrWaitFifo {
 public:
  static constexpr uint8_t kCapacity = 32;  // = MAX_PENDING_LIMIT of the server

  bool push(uint32_t seqnum, uint32_t length, uint32_t start_frame = 0,
            uint32_t number_of_packets = 0);  // false if full
  const IntrRequest &front() const { return items_[0]; }  // size() > 0
  void pop_front();
  // CMD_UNLINK: removes the request with this seqnum, keeps the order of the others.
  // Returns false if no such request is waiting.
  bool remove(uint32_t seqnum);
  bool contains(uint32_t seqnum) const;
  void clear() { count_ = 0; }
  uint8_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ == kCapacity; }

 private:
  IntrRequest items_[kCapacity];
  uint8_t count_{0};
};

// How a request is completed from the ring (computed by plan_serve, applied by consume).
//
// Rules, as a host controller completes an interrupt IN URB (USB 2.0 5.7.3 / 8.4.4 short packets):
// packets are appended in order until
//   - a short packet (len < MPS, including a zero-length packet) arrives      -> short_packet
//   - the requested length is reached exactly with full MPS packets           -> filled
//   - a packet larger than the remaining space arrives                         -> overflow
// If the ring runs out before one of these happens, the request is not ready (it keeps waiting,
// as a real URB would), unless the ring is full: then nothing more can arrive and what is in the
// ring is delivered (ring_limit). That last case has no real-USB equivalent.
struct IntrServePlan {
  bool ready{false};
  uint32_t length{0};    // bytes delivered to the client
  uint16_t packets{0};   // packets consumed from the ring
  bool short_packet{false};
  bool filled{false};
  bool overflow{false};  // the last packet did not fit; only its first bytes are delivered,
                         // the rest of it is discarded. The status to report (-EOVERFLOW as a
                         // Linux HCD would, or 0 with truncated data as the server did so far)
                         // is the caller's decision.
  bool ring_limit{false};
  bool zero_request{false};  // requested length 0: completed at once, no packet consumed
};

// Pure: does not change the ring.
IntrServePlan plan_serve(const IntrRing &ring, uint32_t requested);
// Copies plan.length bytes into out (must hold plan.length bytes) and pops plan.packets packets.
void consume(IntrRing &ring, const IntrServePlan &plan, uint8_t *out);

// Whether to (re)submit a prefetch transfer: session active AND ring not full AND
// (a request is waiting OR the last request came less than window_ms ago). Times in ms,
// unsigned 32-bit (differences valid across a wrap). have_last_request == false: no request since
// the import (the window does not apply).
static constexpr uint32_t kDefaultPrefetchWindowMs = 50;
bool prefetch_allowed(bool session_active, bool ring_full, bool request_waiting, bool have_last_request,
                      uint32_t last_request_ms, uint32_t now_ms, uint32_t window_ms = kDefaultPrefetchWindowMs);

// One interrupt IN endpoint: ring + waiting requests + time of the last request.
class IntrInQueue {
 public:
  bool init(uint16_t depth, uint16_t mps, IntrAllocator allocator = {}) {
    return this->ring_.init(depth, mps, allocator);
  }

  // CMD_SUBMIT IN. Returns false if the FIFO is full (request not stored).
  bool add_request(uint32_t seqnum, uint32_t length, uint32_t now_ms, uint32_t start_frame = 0,
                   uint32_t number_of_packets = 0);
  // Removes the oldest waiting request (e.g. to fail it); false if none.
  bool pop_request(IntrRequest *out);
  // Session ended: drop the waiting requests, keep the stored packets.
  void clear_waiting() { this->waiting_.clear(); }
  // CMD_UNLINK of a waiting request; false if not waiting (already answered or unknown).
  bool unlink(uint32_t seqnum) { return this->waiting_.remove(seqnum); }
  // Received packet from a prefetch transfer; false if the ring is full (packet dropped).
  bool on_packet(const uint8_t *data, uint16_t len) { return this->ring_.push(data, len); }
  // Successful completion of a prefetch transfer with actual_num_bytes as the USB stack reports it.
  // More than MPS is cut to MPS; 0 (or less) is a zero-length packet and is stored as an entry of
  // length 0: for the request being assembled it is a short packet, i.e. its end. false if the ring
  // is full.
  bool on_completion(const uint8_t *data, int actual_num_bytes);

  // The oldest waiting request, if it can be completed now. On true, *seqnum and *plan describe it;
  // call complete_front() with a buffer of plan->length bytes to apply it.
  bool front_ready(uint32_t *seqnum, IntrServePlan *plan) const;
  void complete_front(const IntrServePlan &plan, uint8_t *out);

  // Forgets the time of the last request once its window has passed (call it periodically, before
  // should_prefetch). Without it the stored time would look recent again whenever the 32-bit clock
  // comes back around to it (after 49.7 days without a request), and the prefetch would start
  // taking packets nobody asked for.
  void expire_window(uint32_t now_ms, uint32_t window_ms = kDefaultPrefetchWindowMs);
  // A request is waiting or the last one came less than window_ms ago (the client is reading).
  bool request_active(uint32_t now_ms, uint32_t window_ms = kDefaultPrefetchWindowMs) const;
  // reserved: ring entries already promised to transfers in flight; the ring counts as full when
  // size() + reserved >= depth(), so every completion of those transfers still has room.
  bool should_prefetch(bool session_active, uint32_t now_ms, uint32_t window_ms = kDefaultPrefetchWindowMs,
                       uint16_t reserved = 0) const;

  // New import: drop stored packets and waiting requests, forget the last request time.
  void reset();

  IntrRing &ring() { return this->ring_; }
  const IntrRing &ring() const { return this->ring_; }
  const IntrWaitFifo &waiting() const { return this->waiting_; }

 private:
  IntrRing ring_;
  IntrWaitFifo waiting_;
  uint32_t last_request_ms_{0};
  bool have_last_request_{false};
};

}  // namespace usbip
