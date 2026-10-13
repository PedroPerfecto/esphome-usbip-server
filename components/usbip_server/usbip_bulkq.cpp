#include "usbip_bulkq.h"

namespace usbip {

bool BulkOrder::blocked(uint8_t ep) const {
  for (uint8_t i = 0; i < blocker_count_; ++i) {
    if (blockers_[i].ep == ep) return true;
  }
  return false;
}

uint8_t BulkOrder::waiting_on(uint8_t ep) const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < count_; ++i) {
    if (waiting_[i].ep == ep) ++n;
  }
  return n;
}

bool BulkOrder::is_waiting(uint8_t id) const {
  for (uint8_t i = 0; i < count_; ++i) {
    if (waiting_[i].id == id) return true;
  }
  return false;
}

BulkAdd BulkOrder::add(uint8_t id, uint8_t ep, bool chunked) {
  // Also behind transfers that are only waiting for next_ready(): never overtake them.
  if (this->blocked(ep) || this->waiting_on(ep) != 0) {
    if (count_ >= kCapacity) return BulkAdd::kFull;
    waiting_[count_].id = id;
    waiting_[count_].ep = ep;
    waiting_[count_].chunked = chunked;
    ++count_;
    return BulkAdd::kQueued;
  }
  if (chunked) {
    if (blocker_count_ >= kCapacity) return BulkAdd::kFull;
    blockers_[blocker_count_].id = id;
    blockers_[blocker_count_].ep = ep;
    blockers_[blocker_count_].chunked = true;
    ++blocker_count_;
  }
  return BulkAdd::kSubmitNow;
}

void BulkOrder::erase_(uint8_t i) {
  for (uint8_t k = i; k + 1 < count_; ++k) waiting_[k] = waiting_[k + 1];
  --count_;
}

bool BulkOrder::next_ready(uint8_t *id) {
  // The first waiting transfer of an endpoint that is not blocked. Older transfers of the same
  // endpoint cannot have been skipped: they would see the same (unblocked) endpoint.
  for (uint8_t i = 0; i < count_; ++i) {
    const Item it = waiting_[i];
    if (this->blocked(it.ep)) continue;
    this->erase_(i);
    if (it.chunked) blockers_[blocker_count_++] = it;  // room: one blocker per endpoint, see add()
    *id = it.id;
    return true;
  }
  return false;
}

bool BulkOrder::remove_waiting(uint8_t id) {
  for (uint8_t i = 0; i < count_; ++i) {
    if (waiting_[i].id == id) {
      this->erase_(i);
      return true;
    }
  }
  return false;
}

void BulkOrder::forget(uint8_t id) {
  this->remove_waiting(id);
  for (uint8_t i = 0; i < blocker_count_; ++i) {
    if (blockers_[i].id != id) continue;
    blockers_[i] = blockers_[blocker_count_ - 1];
    --blocker_count_;
    return;
  }
}

void BulkOrder::clear() {
  count_ = 0;
  blocker_count_ = 0;
}

}  // namespace usbip
