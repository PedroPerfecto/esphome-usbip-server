#pragma once
#include "esphome/core/defines.h"
#include "esphome/components/usb_host/usb_host.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "usbip_chunk.h"
#include "usbip_device.h"
#include "usbip_endpoints.h"
#include "usbip_filter.h"
#include "usbip_intrq.h"
#include "usbip_stream.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <atomic>
#include <cstdint>
#include <string>

namespace esphome::usbip_server {

static constexpr uint8_t MAX_PENDING_LIMIT = 32;
// Interrupt IN endpoints served from server-owned prefetch transfers (no HALT, see usbip_intrq.h).
static constexpr uint8_t MAX_INTR_IN_EPS = 4;
static constexpr uint8_t PREFETCH_PER_EP = 2;
static constexpr uint8_t PREFETCH_COUNT = MAX_INTR_IN_EPS * PREFETCH_PER_EP;

// USB/IP server on top of ESPHome usb_host.
// Import, control transfers on ep0, interrupt and bulk transfers (large bulk
// transfers split into chunks with data in PSRAM), UNLINK with transfer cancellation.
class USBIPServer : public usb_host::USBClient {
 public:
  using usb_host::USBClient::USBClient;
  void setup() override;
  void loop() override;
  void dump_config() override;
  // Before a planned reboot (OTA, restart from HA) cleanly closes the USB/IP
  // connection so the client gets a FIN and releases its virtual port.
  bool teardown() override;

#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(usb_connected)
  SUB_BINARY_SENSOR(client_connected)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(client_address)
  // Record of a long dev_lock_ wait / watchdog reboot in the previous boot (RTC_NOINIT), "none" if absent.
  SUB_TEXT_SENSOR(lock_diag)
#endif
#ifdef USE_SENSOR
  SUB_SENSOR(import_count)
  // Diagnostic counters (data endpoints only, not ep0). See usbip_server.cpp for exact count points.
  SUB_SENSOR(in_submitted)
  SUB_SENSOR(in_done_data)
  SUB_SENSOR(in_done_empty)
  SUB_SENSOR(in_done_err)
  SUB_SENSOR(in_ret_sent)
  SUB_SENSOR(out_submitted)
  SUB_SENSOR(out_done_ok)
  SUB_SENSOR(unlink_rx)
  SUB_SENSOR(dropped_silent)
  SUB_SENSOR(stale_skip)
  SUB_SENSOR(send_errors)
  SUB_SENSOR(slot_exhausted)
  SUB_SENSOR(unlink_discarded)
  SUB_SENSOR(in_resubmitted)
  SUB_SENSOR(in_send_failed)
  SUB_SENSOR(cb_queue_full)
  // Peak dev_lock_ hold time (us) around the usb_host_* calls since the last diag publish.
  SUB_SENSOR(dev_lock_hold_max)
  // Interrupt endpoints without HALT (see usbip_server.cpp for exact count points).
  SUB_SENSOR(intr_q_depth_max)
  SUB_SENSOR(intr_q_full)
  SUB_SENSOR(intr_dropped_nosession)
  SUB_SENSOR(intr_flushed_import)
  SUB_SENSOR(intr_unlink_waiting)
  SUB_SENSOR(out_unlink_wait_timeout)
  SUB_SENSOR(intr_overflow)
  SUB_SENSOR(intr_zlp_dropped)
#endif

  void set_port(uint16_t port) { this->port_ = port; }
  void set_busid(const std::string &busid) { this->busid_ = busid; }
  void set_max_transfer_size(uint32_t size) { this->max_transfer_size_ = size; }
  void set_max_pending(uint8_t count) { this->max_pending_ = count; }
  void set_dma_chunk_size(uint32_t size) { this->dma_chunk_size_ = size; }
  void set_interface_mask(uint32_t mask) { this->interface_mask_ = mask; }
  void set_interrupt_queue_depth(uint16_t depth) { this->interrupt_queue_depth_ = depth; }

 protected:
  // Pending URB. Only the TCP task modifies it; the USB task only posts the index to the queue.
  struct PendingSlot {
    USBIPServer *owner{nullptr};
    usb_transfer_t *xfer{nullptr};
    uint32_t seqnum{0};
    uint32_t start_frame{0};
    uint32_t number_of_packets{0};
    uint32_t requested{0};  // transfer_buffer_length from CMD_SUBMIT
    uint32_t unlink_seqnum{0};
    uint8_t index{0};
    uint8_t ep_addr{0};        // non-control: endpoint address including direction
    uint8_t clear_halt_ep{0};  // != 0: on success reset the pipe state on the host
    bool used{false};
    bool dir_in{false};
    bool is_control{false};
    bool is_intr{false};         // interrupt endpoint (only OUT uses slots, IN is served by IntrInEp)
    uint32_t unlink_deadline_ms{0};  // interrupt OUT with unlink_pending: RET_UNLINK at the latest then
    bool orphan{false};          // connection ended, just release
    bool unlink_pending{false};  // client sent CMD_UNLINK, reply with RET_UNLINK
    bool chunked{false};         // large bulk transfer split into chunks
    uint8_t *big_buf{nullptr};   // chunked: all data in PSRAM (IN result / OUT source)
    usbip::ChunkPlan plan;       // chunked: split state
  };

  void on_connected() override;
  void on_disconnected() override;

  // TCP task (owns the socket, the slots and submission)
  static void tcp_task_fn_(void *arg);
  void tcp_task_loop_();
  // Serves a session; returns the socket of the next client that took it over
  // (new OP_REQ_IMPORT), or -1. takeover_busid: import already read.
  int serve_client_(int sock, int listen_sock, const char *takeover_busid);
  // New connection during a session: OP_REQ_DEVLIST is served and closed (-1),
  // OP_REQ_IMPORT returns the socket and busid (takeover), otherwise closes (-1).
  int handle_side_connection_(int listen_sock, char *busid_out);
  static void configure_session_socket_(int sock);
  bool handle_submit_(int sock, const usbip::Message &m);
  bool handle_data_submit_(int sock, const usbip::Message &m);
  esp_err_t submit_locked_(usb_transfer_t *x, bool is_control);  // submits under dev_lock_
  void cancel_endpoint_(uint8_t ep_addr);                        // halt + flush + clear
  void note_request_size_(uint32_t size);
  static uint8_t *alloc_big_(size_t size);  // PSRAM if available
  bool handle_unlink_(int sock, const usbip::Message &m);
  bool drain_completions_(int sock);  // sock < 0: just release, send nothing
  void end_session_();
  PendingSlot *alloc_slot_();
  void free_slot_(PendingSlot &slot);
  static bool send_all_(int sock, const uint8_t *data, size_t len);
  static bool send_ret_submit_(int sock, uint32_t seqnum, uint32_t start_frame, uint32_t number_of_packets,
                               int32_t status, uint32_t actual_length, const uint8_t *in_data);
  static bool send_ret_unlink_(int sock, uint32_t seqnum, int32_t status);
  static int32_t map_status_(usb_transfer_status_t status);

  // USB task
  static void transfer_cb_(usb_transfer_t *xfer);
  static void prefetch_cb_(usb_transfer_t *xfer);

  // Interrupt IN without HALT. Everything below runs in the TCP task only, except
  // prefetch_in_flight_ (also decremented by prefetch_cb_ in the USB task and read by on_disconnected).
  struct IntrInEp;
  struct PrefetchXfer {
    USBIPServer *owner{nullptr};
    IntrInEp *ep{nullptr};
    usb_transfer_t *xfer{nullptr};
    uint8_t index{0};        // done_queue_ value: MAX_PENDING_LIMIT + position in prefetch_
    bool in_flight{false};   // submitted, completion not drained yet (TCP task view)
  };
  struct IntrInEp {
    bool active{false};
    uint8_t addr{0};
    uint16_t mps{0};
    usbip::IntrInQueue q;
    PrefetchXfer *pf[PREFETCH_PER_EP]{};
    bool stalled{false};     // a prefetch transfer ended with STALL; only a forwarded CLEAR_FEATURE(HALT) resumes
    bool need_clear{false};  // pipe halted by a transfer error (not STALL): clear on the host before resubmitting
    bool blocked_full{false};  // prefetch wanted but held back by a full ring (for intr_q_full transitions)
  };
  static uint32_t now_ms_() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
  IntrInEp *find_intr_in_(uint8_t addr);
  bool setup_intr_eps_();             // (re)configures intr_in_ for the current device generation
  void intr_on_import_();             // new import: reset the queues, start the session state
  bool handle_intr_in_submit_(int sock, const usbip::Message &m, const usbip::EndpointInfo &e);
  bool serve_intr_ep_(int sock, IntrInEp &ep);  // answers waiting requests from the ring
  bool fail_waiting_(int sock, IntrInEp &ep, int32_t status);
  void pump_prefetch_(IntrInEp &ep, uint32_t now);
  bool handle_prefetch_done_(int sock, uint8_t pf_index);
  bool intr_service_(int sock);       // serve_client_ loop: prefetch window, OUT unlink timeouts, peak depth
  uint16_t pf_in_flight_(const IntrInEp &ep) const;

  // timeout_ticks != portMAX_DELAY: gives up if dev_lock_ is busy (*busy = true, returns false).
  bool get_device_info_(usbip::DeviceInfo &out, TickType_t timeout_ticks = portMAX_DELAY, bool *busy = nullptr);
  bool get_endpoint_(uint8_t address, usbip::EndpointInfo &out);
  bool device_ready_();
  void note_import_(int sock);   // TCP task: successful import (client + counter)
  void publish_entities_();      // main loop: state to entities, only on change
  void publish_diag_();          // main loop: diag counters to log/entities, only on change
  void check_lock_watchdog_();   // main loop: reboot if dev_lock_ stays held in a usb_host_* section
  void check_intr_setup_stuck_();  // main loop: reboot if the interrupt setup stays postponed
  // Main loop only. Store a record of a long dev_lock_ wait in RTC_NOINIT memory (survives the
  // reboot that follows), see usbip_server.cpp.
  void record_lock_event_(uint8_t kind);
  // Main loop only: dev_lock_ Take that leaves a record after LOOP_LOCK_WARN_MS, then waits as before.
  void take_dev_lock_from_loop_(uint8_t kind);
  void load_lock_diag_();  // setup(): report and clear the record left by the previous boot
  void release_interfaces_();

  uint16_t port_{3240};
  std::string busid_{"1-1"};
  uint32_t max_transfer_size_{131072};
  uint8_t max_pending_{16};
  uint32_t dma_chunk_size_{16384};
  uint32_t interface_mask_{usbip::kAllInterfaces};
  uint16_t interrupt_queue_depth_{usbip::IntrRing::kDefaultDepth};

  IntrInEp intr_in_[MAX_INTR_IN_EPS];
  PrefetchXfer prefetch_[PREFETCH_COUNT];
  uint32_t intr_generation_{0};      // device generation intr_in_ was set up for (TCP task)
  bool intr_configured_{false};      // intr_in_ matches intr_generation_ (TCP task)
  bool intr_session_{false};         // an import is active (TCP task)
  uint8_t *intr_out_buf_{nullptr};   // RET_SUBMIT data assembled from the ring (dma_chunk_size_ B)
  std::atomic<uint32_t> prefetch_in_flight_{0};
  std::atomic<uint32_t> intr_out_in_flight_{0};  // interrupt OUT slots submitted, callback not run yet
  // setup_intr_eps_ postponed since (now_ms_, 0 = not postponed) and with how many transfers in flight;
  // written by the TCP task, read by check_intr_setup_stuck_ (main loop).
  std::atomic<uint32_t> intr_setup_postponed_ms_{0};
  std::atomic<uint32_t> intr_setup_postponed_count_{0};
  std::atomic<uint32_t> dev_generation_{0};  // ++ in on_connected (under dev_lock_)

  SemaphoreHandle_t dev_lock_{nullptr};
  usbip::DeviceInfo dev_info_{};  // guarded by dev_lock_
  bool dev_ready_{false};         // guarded by dev_lock_
  // Mirror of dev_ready_ for the main loop (publish_entities_), so that task never waits for
  // dev_lock_ in its 1 s path. Written only under dev_lock_ (on_connected/on_disconnected).
  std::atomic<bool> dev_ready_pub_{false};
  usbip::EndpointInfo eps_[usbip::kMaxEndpoints];  // guarded by dev_lock_
  size_t ep_count_{0};                              // guarded by dev_lock_
  uint32_t claimed_mask_{0};                        // claimed interfaces (bit = number)
  // Filtered config descriptor that the server sends to the client itself;
  // length 0 = no filter, GET_DESCRIPTOR(CONFIG) is forwarded to the device.
  static constexpr size_t MAX_CONFIG_DESC = 512;
  uint8_t cfg_filtered_[MAX_CONFIG_DESC];  // guarded by dev_lock_
  uint16_t cfg_filtered_len_{0};           // guarded by dev_lock_

  // dev_lock_ hold tracking around the three sections that call usb_host_* (submit, cancel,
  // clear). lock_since_us_ != 0 while such a section holds the lock; read by the main loop
  // watchdog. 32-bit microsecond timestamps (wrap after ~71 min, differences stay valid for
  // holds far shorter than that) keep the atomics at the native word size of the 32-bit core.
  enum class LockSite : uint8_t { NONE = 0, SUBMIT = 1, CANCEL = 2, CLEAR = 3 };
  static const char *lock_site_str_(uint8_t site);
  static uint32_t lock_clock_us_() {
    const uint32_t t = static_cast<uint32_t>(esp_timer_get_time());
    return t != 0 ? t : 1;  // 0 is reserved for "not held"
  }
  std::atomic<uint32_t> lock_since_us_{0};
  std::atomic<uint8_t> lock_site_{0};
  std::atomic<uint32_t> dev_lock_hold_max_us_{0};  // written by the TCP task, read+reset by publish_diag_
  uint32_t lock_watchdog_reboots_{0};              // main loop only; only meaningful in the log line

  // RAII: takes dev_lock_ and records since when/where it is held (see above); the destructor
  // clears the record, folds the hold time into dev_lock_hold_max_us_ and releases the lock.
  class DevLockHold {
   public:
    DevLockHold(USBIPServer *s, LockSite site) : s_(s) {
      xSemaphoreTake(s_->dev_lock_, portMAX_DELAY);
      t0_ = lock_clock_us_();
      s_->lock_site_.store(static_cast<uint8_t>(site), std::memory_order_relaxed);
      s_->lock_since_us_.store(t0_, std::memory_order_release);  // after lock_site_: the reader sees both
    }
    ~DevLockHold() {
      const uint32_t held = lock_clock_us_() - t0_;
      s_->lock_since_us_.store(0, std::memory_order_relaxed);
      uint32_t cur = s_->dev_lock_hold_max_us_.load(std::memory_order_relaxed);
      while (held > cur && !s_->dev_lock_hold_max_us_.compare_exchange_weak(cur, held, std::memory_order_relaxed)) {
      }
      xSemaphoreGive(s_->dev_lock_);
    }
    DevLockHold(const DevLockHold &) = delete;
    DevLockHold &operator=(const DevLockHold &) = delete;

   private:
    USBIPServer *s_;
    uint32_t t0_{0};
  };

  PendingSlot slots_[MAX_PENDING_LIMIT];
  QueueHandle_t done_queue_{nullptr};
  uint8_t *rx_payload_{nullptr};
  uint32_t max_seen_request_{0};
  TaskHandle_t tcp_task_{nullptr};
  bool tcp_started_{false};
  std::atomic<bool> tcp_listening_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> session_active_{false};
  std::atomic<uint32_t> session_closed_tick_{0};
  // Entity state: written by the TCP task, read by the main loop (publish_entities_).
  std::atomic<uint32_t> client_addr_{0};   // s_addr of the client with an import, 0 = none
  std::atomic<uint32_t> import_count_{0};  // successful imports since boot

  // Diagnostic counters, data endpoints only (not ep0/control). Written from the TCP task
  // (and, for in_done_*/stale_skip, conceptually "observed by" the TCP task draining the USB
  // task's completion queue); read from the main loop by publish_diag_(). fetch_add with
  // memory_order_relaxed: only the counts matter, not ordering between counters.
  std::atomic<uint32_t> in_submitted_{0};    // handle_data_submit_: IN submit_locked_ succeeded
  std::atomic<uint32_t> in_done_data_{0};    // drain_completions_: IN, status COMPLETED, actual_num_bytes > 0
  std::atomic<uint32_t> in_done_empty_{0};   // drain_completions_: IN, status COMPLETED, actual_num_bytes == 0
  std::atomic<uint32_t> in_done_err_{0};     // drain_completions_: IN, status != COMPLETED (incl. CANCELED)
  std::atomic<uint32_t> in_ret_sent_{0};     // drain_completions_: IN COMPLETED with data, RET_SUBMIT sent ok
  std::atomic<uint32_t> out_submitted_{0};   // handle_data_submit_: OUT submit_locked_ succeeded
  std::atomic<uint32_t> out_done_ok_{0};     // drain_completions_: OUT, status COMPLETED
  std::atomic<uint32_t> unlink_rx_{0};       // handle_unlink_: CMD_UNLINK received
  std::atomic<uint32_t> dropped_silent_{0};  // drain_completions_: IN COMPLETED data not sent (ok==false/orphan/sock<0)
  std::atomic<uint32_t> stale_skip_{0};      // drain_completions_: stale queue entry (!s.used || x == nullptr)
  std::atomic<uint32_t> send_errors_{0};     // drain_completions_: send_ret_submit_/send_all_ failed (data ep)
  std::atomic<uint32_t> slot_exhausted_{0};  // handle_data_submit_: alloc_slot_() == nullptr
  std::atomic<uint32_t> unlink_discarded_{0};  // drain_completions_: IN COMPLETED with data, but only RET_UNLINK sent
  std::atomic<uint32_t> in_resubmitted_{0};    // drain_completions_: IN collateral cancel, resubmit succeeded
  std::atomic<uint32_t> in_send_failed_{0};    // drain_completions_: RET_SUBMIT send failed for IN COMPLETED data
  std::atomic<uint32_t> cb_queue_full_{0};     // transfer_cb_/prefetch_cb_ (USB task): xQueueSend to done_queue_ failed
  // Interrupt endpoints without HALT:
  std::atomic<uint32_t> intr_q_depth_max_{0};        // peak ring size over all IN eps, read+reset by publish_diag_
  std::atomic<uint32_t> intr_q_full_{0};             // prefetch held back by a full ring (transitions) or packet dropped
  std::atomic<uint32_t> intr_dropped_nosession_{0};  // prefetch packet received without an active session
  std::atomic<uint32_t> intr_flushed_import_{0};     // packets left in the ring and dropped at a new import
  std::atomic<uint32_t> intr_unlink_waiting_{0};     // CMD_UNLINK of a waiting IN request (no USB operation)
  std::atomic<uint32_t> out_unlink_wait_timeout_{0};  // interrupt OUT did not finish 5 s after CMD_UNLINK
  std::atomic<uint32_t> intr_overflow_{0};           // IN request shorter than the packet -> -EOVERFLOW
  std::atomic<uint32_t> intr_zlp_dropped_{0};        // prefetch completed with 0 B, not stored

  // Last published values (main loop only).
  int8_t pub_usb_{-1};
  int8_t pub_client_{-1};
  uint32_t pub_addr_{0xFFFFFFFFu};
  int64_t pub_imports_{-1};
  // Last published diag values (main loop only), same order as in publish_diag_(): the 16
  // counters, dev_lock_hold_max_us, then the 8 interrupt values. setup() fills it with
  // UINT32_MAX = "never published", so the first publish_diag_ emits every value, zeros included.
  static constexpr size_t DIAG_COUNT = 25;
  uint32_t diag_pub_[DIAG_COUNT]{};
  char takeover_busid_[usbip::kBusIdSize]{};  // busid from the taken-over import (TCP task only)
};

}  // namespace esphome::usbip_server
