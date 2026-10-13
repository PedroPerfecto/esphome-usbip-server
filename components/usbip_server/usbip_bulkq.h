#pragma once
#include <cstdint>

// Order of bulk transfers on one endpoint (no ESPHome/ESP-IDF dependencies).
//
// A large bulk transfer is sent to the device as several consecutive USB transfers (usbip_chunk.h),
// each submitted only after the previous one completed. A transfer submitted to the same endpoint
// in between would be served by the device in the middle of the large one. So while a chunked
// transfer is in progress on an endpoint, later transfers for that endpoint wait here, in arrival
// order, and are handed out only after it finished completely. Transfers that fit one USB transfer
// do not block the ones behind them: the USB host stack keeps their order by itself.
//
// Transfers are identified by an id chosen by the caller (the server uses its slot index).
namespace usbip {

enum class BulkAdd : uint8_t {
  kSubmitNow,  // nothing in front of it has to finish first: submit to the device now
  kQueued,     // stored; next_ready() hands it out later
  kFull,       // no room (transfer not stored)
};

class BulkOrder {
 public:
  static constexpr uint8_t kCapacity = 32;  // = MAX_PENDING_LIMIT of the server

  // New transfer for endpoint ep. chunked: it needs more than one USB transfer.
  BulkAdd add(uint8_t id, uint8_t ep, bool chunked);
  // The oldest waiting transfer that may be submitted now (its endpoint has no chunked transfer in
  // progress and nothing older waiting). Removes it from the FIFO; if it is chunked, its endpoint is
  // blocked from now on. Returns false if there is none. Call repeatedly until it returns false.
  bool next_ready(uint8_t *id);
  // The transfer is gone for whatever reason: completed (all chunks), failed, cancelled, or dropped
  // while waiting. Unblocks its endpoint if it was the chunked transfer in progress. Unknown ids
  // are ignored.
  void forget(uint8_t id);
  // CMD_UNLINK: removes the transfer if it is still waiting (never submitted) and keeps the order
  // of the others. Returns false if it is not waiting (submitted already or unknown).
  bool remove_waiting(uint8_t id);

  bool is_waiting(uint8_t id) const;
  bool blocked(uint8_t ep) const;  // a chunked transfer is in progress on ep
  uint8_t waiting() const { return count_; }
  uint8_t waiting_on(uint8_t ep) const;
  void clear();

 private:
  struct Item {
    uint8_t id{0};
    uint8_t ep{0};
    bool chunked{false};
  };
  void erase_(uint8_t i);

  Item waiting_[kCapacity];   // all endpoints, oldest first
  Item blockers_[kCapacity];  // chunked transfers in progress, at most one per endpoint
  uint8_t count_{0};
  uint8_t blocker_count_{0};
};

}  // namespace usbip
