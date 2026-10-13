#include "doctest.h"
#include "../components/usbip_server/usbip_bulkq.h"
#include <vector>

using namespace usbip;

namespace {
constexpr uint8_t kOut = 0x02;
constexpr uint8_t kIn = 0x81;

// Everything next_ready() hands out now, in order.
std::vector<int> ready(BulkOrder &q) {
  std::vector<int> ids;
  uint8_t id = 0;
  while (q.next_ready(&id)) ids.push_back(id);
  return ids;
}
}  // namespace

TEST_CASE("bulk order: transfers that fit one USB transfer are all submitted at once") {
  BulkOrder q;
  CHECK(q.add(0, kIn, false) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kIn, false) == BulkAdd::kSubmitNow);
  CHECK(q.add(2, kIn, false) == BulkAdd::kSubmitNow);
  CHECK(q.waiting() == 0);
  CHECK_FALSE(q.blocked(kIn));
  CHECK(ready(q).empty());
}

TEST_CASE("bulk order: nothing is submitted while a chunked transfer is in progress") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.blocked(kOut));
  CHECK(q.add(1, kOut, false) == BulkAdd::kQueued);
  CHECK(q.add(2, kOut, false) == BulkAdd::kQueued);
  CHECK(q.waiting_on(kOut) == 2);
  CHECK(ready(q).empty());  // chunks of 0 still to come: 1 and 2 must not get in between

  q.forget(0);  // last chunk done
  CHECK_FALSE(q.blocked(kOut));
  CHECK(ready(q) == std::vector<int>{1, 2});  // in arrival order
  CHECK(q.waiting() == 0);
}

TEST_CASE("bulk order: a waiting chunked transfer blocks the ones behind it once it starts") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kOut, false) == BulkAdd::kQueued);
  CHECK(q.add(2, kOut, true) == BulkAdd::kQueued);
  CHECK(q.add(3, kOut, false) == BulkAdd::kQueued);
  CHECK(q.add(4, kOut, true) == BulkAdd::kQueued);

  q.forget(0);
  CHECK(ready(q) == std::vector<int>{1, 2});  // 2 is chunked: 3 waits for all of it
  CHECK(q.blocked(kOut));
  q.forget(1);
  CHECK(ready(q).empty());  // 1 was not the blocker
  q.forget(2);
  CHECK(ready(q) == std::vector<int>{3, 4});
  CHECK(q.blocked(kOut));
  q.forget(3);
  q.forget(4);
  CHECK_FALSE(q.blocked(kOut));
  CHECK(q.waiting() == 0);
}

TEST_CASE("bulk order: a chunked transfer behind single transfers starts at once and blocks later ones") {
  BulkOrder q;
  CHECK(q.add(0, kIn, false) == BulkAdd::kSubmitNow);
  // The host stack serves 0 first; the first chunk of 1 queues behind it.
  CHECK(q.add(1, kIn, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(2, kIn, false) == BulkAdd::kQueued);
  q.forget(0);
  CHECK(ready(q).empty());
  q.forget(1);
  CHECK(ready(q) == std::vector<int>{2});
}

TEST_CASE("bulk order: a new transfer never overtakes waiting ones, even after the endpoint got free") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kOut, false) == BulkAdd::kQueued);
  q.forget(0);  // endpoint free, but 1 has not been handed out yet
  CHECK(q.add(2, kOut, false) == BulkAdd::kQueued);
  CHECK(ready(q) == std::vector<int>{1, 2});
}

TEST_CASE("bulk order: endpoints are independent") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kIn, false) == BulkAdd::kSubmitNow);  // other endpoint: not blocked
  CHECK(q.add(2, kIn, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(3, kOut, false) == BulkAdd::kQueued);
  CHECK(q.add(4, kIn, false) == BulkAdd::kQueued);
  CHECK(q.add(5, kOut, false) == BulkAdd::kQueued);

  q.forget(2);  // IN done: only the IN transfer is released, OUT keeps waiting
  CHECK(ready(q) == std::vector<int>{4});
  CHECK(q.waiting_on(kOut) == 2);
  q.forget(0);
  CHECK(ready(q) == std::vector<int>{3, 5});
}

TEST_CASE("bulk order: UNLINK of a waiting transfer removes it and keeps the order of the others") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kOut, false) == BulkAdd::kQueued);
  CHECK(q.add(2, kOut, false) == BulkAdd::kQueued);
  CHECK(q.add(3, kOut, false) == BulkAdd::kQueued);
  CHECK(q.is_waiting(2));

  CHECK(q.remove_waiting(2));
  CHECK_FALSE(q.is_waiting(2));
  CHECK_FALSE(q.remove_waiting(2));  // only once
  CHECK(q.blocked(kOut));            // the transfer in progress is untouched
  CHECK(ready(q).empty());           // and nothing is released by the removal

  q.forget(0);
  CHECK(ready(q) == std::vector<int>{1, 3});  // 2 is never handed out
}

TEST_CASE("bulk order: UNLINK of a waiting chunked transfer does not leave the endpoint blocked") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kOut, true) == BulkAdd::kQueued);
  CHECK(q.add(2, kOut, false) == BulkAdd::kQueued);
  CHECK(q.remove_waiting(1));
  q.forget(0);
  CHECK(ready(q) == std::vector<int>{2});
  CHECK_FALSE(q.blocked(kOut));
}

TEST_CASE("bulk order: a submitted transfer is not 'waiting' for UNLINK") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kOut, false) == BulkAdd::kQueued);
  CHECK_FALSE(q.remove_waiting(0));  // in progress: needs the USB cancel path
  CHECK(q.blocked(kOut));
  q.forget(0);
  CHECK(ready(q) == std::vector<int>{1});
  CHECK_FALSE(q.remove_waiting(1));  // handed out = submitted
  CHECK_FALSE(q.remove_waiting(99));
}

TEST_CASE("bulk order: forget() of a waiting transfer drops it; unknown ids are ignored") {
  BulkOrder q;
  CHECK(q.add(0, kOut, true) == BulkAdd::kSubmitNow);
  CHECK(q.add(1, kOut, false) == BulkAdd::kQueued);
  q.forget(77);
  CHECK(q.blocked(kOut));
  CHECK(q.waiting() == 1);
  q.forget(1);
  CHECK(q.waiting() == 0);
  q.forget(0);
  CHECK(ready(q).empty());
}

TEST_CASE("bulk order: an id can be reused after forget()") {
  BulkOrder q;
  CHECK(q.add(5, kOut, true) == BulkAdd::kSubmitNow);
  q.forget(5);
  CHECK(q.add(5, kIn, false) == BulkAdd::kSubmitNow);
  q.forget(5);  // was not a blocker this time
  CHECK_FALSE(q.blocked(kOut));
  CHECK_FALSE(q.blocked(kIn));
}

TEST_CASE("bulk order: full FIFO refuses, clear() empties everything") {
  BulkOrder q;
  CHECK(q.add(200, kOut, true) == BulkAdd::kSubmitNow);
  for (uint8_t i = 0; i < BulkOrder::kCapacity; ++i) CHECK(q.add(i, kOut, false) == BulkAdd::kQueued);
  CHECK(q.add(100, kOut, false) == BulkAdd::kFull);
  CHECK(q.waiting() == BulkOrder::kCapacity);
  CHECK_FALSE(q.is_waiting(100));

  q.clear();
  CHECK(q.waiting() == 0);
  CHECK_FALSE(q.blocked(kOut));
  CHECK(q.add(0, kOut, false) == BulkAdd::kSubmitNow);
}
