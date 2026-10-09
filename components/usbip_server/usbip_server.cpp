#include "usbip_server.h"
#include "usbip_control.h"
#include "usbip_urb.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <lwip/sockets.h>

namespace esphome::usbip_server {

static const char *const TAG = "usbip_server";
static constexpr uint32_t TCP_TASK_STACK_SIZE = 6144;
static constexpr UBaseType_t TCP_TASK_PRIORITY = 4;  // below the USB task (5), above the main loop
static constexpr uint32_t SELECT_TIMEOUT_US = 2000;
static constexpr uint32_t SESSION_DRAIN_TIMEOUT_MS = 2000;
static constexpr uint32_t DUMP_CONFIG_LOCK_TIMEOUT_MS = 100;
// on_connected/on_disconnected (main loop) leave a record if dev_lock_ is not free after this long.
static constexpr uint32_t LOOP_LOCK_WARN_MS = 1000;
// A usb_host_* section (submit/cancel/clear) holding dev_lock_ this long is considered hung.
static constexpr uint32_t LOCK_WATCHDOG_US = 10u * 1000u * 1000u;
// Interrupt OUT after CMD_UNLINK: RET_UNLINK at the latest after this long, then the slot is an orphan.
static constexpr uint32_t OUT_UNLINK_WAIT_MS = 5000;
// on_disconnected (main loop): bounded wait for the prefetch transfers to complete (NO_DEVICE) before
// releasing the interfaces; far below the lock watchdog (10 s) and the task watchdog (5 s).
static constexpr uint32_t DISCONNECT_PREFETCH_WAIT_MS = 500;
static constexpr uint32_t DISCONNECT_PREFETCH_POLL_MS = 10;

static const char *parse_error_str(usbip::ParseError e) {
  switch (e) {
    case usbip::ParseError::kBadVersion:
      return "bad version";
    case usbip::ParseError::kUnknownOp:
      return "unknown OP code";
    case usbip::ParseError::kUnknownCommand:
      return "unknown URB command";
    case usbip::ParseError::kPayloadTooLarge:
      return "OUT payload larger than max_transfer_size";
    case usbip::ParseError::kIsoUnsupported:
      return "ISO transfers not supported";
    case usbip::ParseError::kBadBusId:
      return "bad busid";
    default:
      return "none";
  }
}

// ---------------------------------------------------------------- lock diag record

// Survives software resets and panic reboots (RTC slow memory, not cleared by IDF startup), but not
// power loss: after a power cycle the content is garbage, hence the magic + range checks. It is a
// different section/memory than the ESPHome crash handler's .noinit data (esp32/crash_handler.cpp).
// Only the main loop writes it; setup() of the next boot reads and clears it.
namespace {
constexpr uint32_t LOCK_DIAG_MAGIC = 0x4C4B4447;  // "LKDG"
enum LockDiagKind : uint8_t {
  KIND_NONE = 0,
  KIND_WATCHDOG_REBOOT = 1,
  KIND_CONNECT_WAIT = 2,
  KIND_DISCONNECT_WAIT = 3,
};
struct LockDiagRecord {
  uint32_t magic;
  uint8_t kind;
  uint8_t site;  // USBIPServer::LockSite, 0 = no tracked section holding the lock
  uint16_t reserved;
  uint32_t held_ms;
  uint32_t uptime_ms;
};
static_assert(sizeof(LockDiagRecord) == 16, "unexpected LockDiagRecord layout");
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
RTC_NOINIT_ATTR LockDiagRecord s_lock_diag;

// Fields first, the magic last (with a fence), so a reset in the middle leaves no half-written
// record that looks valid.
void store_lock_diag(uint8_t kind, uint8_t site, uint32_t held_ms) {
  s_lock_diag.magic = 0;
  s_lock_diag.kind = kind;
  s_lock_diag.site = site;
  s_lock_diag.reserved = 0;
  s_lock_diag.held_ms = held_ms;
  s_lock_diag.uptime_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  std::atomic_thread_fence(std::memory_order_release);
  s_lock_diag.magic = LOCK_DIAG_MAGIC;
}

const char *lock_diag_kind_str(uint8_t kind) {
  switch (kind) {
    case KIND_WATCHDOG_REBOOT:
      return "WATCHDOG_REBOOT";
    case KIND_CONNECT_WAIT:
      return "CONNECT_WAIT";
    case KIND_DISCONNECT_WAIT:
      return "DISCONNECT_WAIT";
    default:
      return "unknown";
  }
}
}  // namespace

// ---------------------------------------------------------------- ESPHome

void USBIPServer::setup() {
  this->load_lock_diag_();
  if (this->max_pending_ > MAX_PENDING_LIMIT) this->max_pending_ = MAX_PENDING_LIMIT;
  this->interrupt_queue_depth_ = std::min<uint16_t>(
      std::max<uint16_t>(this->interrupt_queue_depth_, usbip::IntrRing::kMinDepth), usbip::IntrRing::kMaxDepth);
  this->dev_lock_ = xSemaphoreCreateMutex();
  // Room for every slot and every prefetch transfer: each posts at most one entry per submission.
  this->done_queue_ = xQueueCreate(this->max_pending_ + PREFETCH_COUNT, sizeof(uint8_t));
  this->rx_payload_ = alloc_big_(this->max_transfer_size_);
  this->intr_out_buf_ = alloc_big_(this->dma_chunk_size_);
  if (this->dev_lock_ == nullptr || this->done_queue_ == nullptr || this->rx_payload_ == nullptr ||
      this->intr_out_buf_ == nullptr) {
    ESP_LOGE(TAG, "Allocation failed");
    this->mark_failed();
    return;
  }
  for (uint8_t i = 0; i < MAX_PENDING_LIMIT; ++i) {
    this->slots_[i].owner = this;
    this->slots_[i].index = i;
  }
  for (uint8_t i = 0; i < PREFETCH_COUNT; ++i) {
    this->prefetch_[i].owner = this;
    this->prefetch_[i].index = static_cast<uint8_t>(MAX_PENDING_LIMIT + i);
  }
  std::fill(std::begin(this->diag_pub_), std::end(this->diag_pub_), UINT32_MAX);
  usb_host::USBClient::setup();
  // Entities may only be changed from the main loop; loop() may be disabled
  // (disable_loop), the scheduler runs regardless.
  this->set_interval("entities", 1000, [this]() { this->publish_entities_(); });
  this->set_interval("diag", 10000, [this]() { this->publish_diag_(); });
}

void USBIPServer::load_lock_diag_() {
  char text[96] = "žiadny";
  const LockDiagRecord rec = s_lock_diag;
  if (rec.magic == LOCK_DIAG_MAGIC && rec.kind >= KIND_WATCHDOG_REBOOT && rec.kind <= KIND_DISCONNECT_WAIT &&
      rec.site <= static_cast<uint8_t>(LockSite::CLEAR)) {
    snprintf(text, sizeof(text), "%s site=%s held=%u ms uptime=%u ms", lock_diag_kind_str(rec.kind),
             lock_site_str_(rec.site), (unsigned) rec.held_ms, (unsigned) rec.uptime_ms);
    ESP_LOGE(TAG, "Previous boot left a dev_lock_ record: %s", text);
  }
  s_lock_diag.magic = 0;
#ifdef USE_TEXT_SENSOR
  if (this->lock_diag_text_sensor_ != nullptr) this->lock_diag_text_sensor_->publish_state(text);
#endif
}

void USBIPServer::record_lock_event_(uint8_t kind) {
  const uint32_t since = this->lock_since_us_.load(std::memory_order_acquire);
  uint8_t site = 0;
  uint32_t held_ms = 0;
  if (since != 0) {  // 0 = the lock is held by a section that is not tracked (or a stale site)
    site = this->lock_site_.load(std::memory_order_relaxed);
    held_ms = (lock_clock_us_() - since) / 1000;
  }
  store_lock_diag(kind, site, held_ms);
  ESP_LOGE(TAG, "dev_lock_ busy: %s site=%s held=%u ms", lock_diag_kind_str(kind), lock_site_str_(site),
           (unsigned) held_ms);
}

void USBIPServer::take_dev_lock_from_loop_(uint8_t kind) {
  if (xSemaphoreTake(this->dev_lock_, pdMS_TO_TICKS(LOOP_LOCK_WARN_MS)) == pdTRUE) return;
  this->record_lock_event_(kind);
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
}

const char *USBIPServer::lock_site_str_(uint8_t site) {
  switch (static_cast<LockSite>(site)) {
    case LockSite::NONE:
      return "none";
    case LockSite::SUBMIT:
      return "submit_locked_";
    case LockSite::CANCEL:
      return "cancel_endpoint_";
    case LockSite::CLEAR:
      return "drain_completions_ clear";
    default:
      return "unknown";
  }
}

// Main loop, every 1 s. The USB/IP TCP task holds dev_lock_ around usb_host_* calls that can wait
// without a timeout inside ESP-IDF (e.g. usb_host_endpoint_halt). If that never returns, the TCP
// task is wedged for good, so restart instead of waiting for the task watchdog to hit the main loop.
void USBIPServer::check_lock_watchdog_() {
  const uint32_t since = this->lock_since_us_.load(std::memory_order_acquire);
  if (since == 0) return;
  const uint32_t held_us = lock_clock_us_() - since;  // modulo 2^32: correct across a 32-bit wrap
  if (held_us <= LOCK_WATCHDOG_US) return;
  // Re-check: the section may have finished (and a new one started) between the two reads.
  if (this->lock_since_us_.load(std::memory_order_relaxed) != since) return;
  const uint8_t site = this->lock_site_.load(std::memory_order_relaxed);
  ++this->lock_watchdog_reboots_;
  ESP_LOGE(TAG, "dev_lock_ held %u ms in %s (watchdog reboot #%u), rebooting", (unsigned) (held_us / 1000),
           lock_site_str_(site), (unsigned) this->lock_watchdog_reboots_);
  // Written last: safe_reboot() is the point of no return, the record is what the next boot reports.
  store_lock_diag(KIND_WATCHDOG_REBOOT, site, held_us / 1000);
  App.safe_reboot();
}

void USBIPServer::publish_entities_() {
  this->check_lock_watchdog_();
  // dev_ready_pub_ instead of device_ready_(): this path must never wait for dev_lock_.
  const bool usb = this->dev_ready_pub_.load(std::memory_order_relaxed);
  const uint32_t addr = this->client_addr_.load();
  const uint32_t imports = this->import_count_.load();
#ifdef USE_BINARY_SENSOR
  if (this->usb_connected_binary_sensor_ != nullptr && this->pub_usb_ != (usb ? 1 : 0))
    this->usb_connected_binary_sensor_->publish_state(usb);
  if (this->client_connected_binary_sensor_ != nullptr && this->pub_client_ != (addr != 0 ? 1 : 0))
    this->client_connected_binary_sensor_->publish_state(addr != 0);
#endif
#ifdef USE_TEXT_SENSOR
  if (this->client_address_text_sensor_ != nullptr && this->pub_addr_ != addr) {
    char ip[16] = "";
    if (addr != 0) {
      struct in_addr a {};
      a.s_addr = addr;
      inet_ntoa_r(a, ip, sizeof(ip));
    }
    this->client_address_text_sensor_->publish_state(ip);
  }
#endif
#ifdef USE_SENSOR
  if (this->import_count_sensor_ != nullptr && this->pub_imports_ != static_cast<int64_t>(imports))
    this->import_count_sensor_->publish_state(static_cast<float>(imports));
#endif
  this->pub_usb_ = usb ? 1 : 0;
  this->pub_client_ = addr != 0 ? 1 : 0;
  this->pub_addr_ = addr;
  this->pub_imports_ = imports;
}

void USBIPServer::publish_diag_() {
  const uint32_t v[DIAG_COUNT] = {
      this->in_submitted_.load(std::memory_order_relaxed),
      this->in_done_data_.load(std::memory_order_relaxed),
      this->in_done_empty_.load(std::memory_order_relaxed),
      this->in_done_err_.load(std::memory_order_relaxed),
      this->in_ret_sent_.load(std::memory_order_relaxed),
      this->out_submitted_.load(std::memory_order_relaxed),
      this->out_done_ok_.load(std::memory_order_relaxed),
      this->unlink_rx_.load(std::memory_order_relaxed),
      this->dropped_silent_.load(std::memory_order_relaxed),
      this->stale_skip_.load(std::memory_order_relaxed),
      this->send_errors_.load(std::memory_order_relaxed),
      this->slot_exhausted_.load(std::memory_order_relaxed),
      this->unlink_discarded_.load(std::memory_order_relaxed),
      this->in_resubmitted_.load(std::memory_order_relaxed),
      this->in_send_failed_.load(std::memory_order_relaxed),
      this->cb_queue_full_.load(std::memory_order_relaxed),
      this->dev_lock_hold_max_us_.exchange(0, std::memory_order_relaxed),  // peak since the last publish
      this->intr_q_depth_max_.exchange(0, std::memory_order_relaxed),     // peak since the last publish
      this->intr_q_full_.load(std::memory_order_relaxed),
      this->intr_dropped_nosession_.load(std::memory_order_relaxed),
      this->intr_flushed_import_.load(std::memory_order_relaxed),
      this->intr_unlink_waiting_.load(std::memory_order_relaxed),
      this->out_unlink_wait_timeout_.load(std::memory_order_relaxed),
      this->intr_overflow_.load(std::memory_order_relaxed),
      this->intr_zlp_dropped_.load(std::memory_order_relaxed),
  };
  static const char *const kNames[DIAG_COUNT] = {
      "in_submitted", "in_done_data", "in_done_empty", "in_done_err",  "in_ret_sent",    "out_submitted",
      "out_done_ok",  "unlink_rx",    "dropped_silent", "stale_skip", "send_errors",    "slot_exhausted",
      "unlink_discarded", "in_resubmitted", "in_send_failed", "cb_queue_full",
      "dev_lock_hold_max_us",
      "intr_q_depth_max", "intr_q_full", "intr_dropped_nosession", "intr_flushed_import", "intr_unlink_waiting",
      "out_unlink_wait_timeout", "intr_overflow", "intr_zlp_dropped",
  };
  bool any_changed = false;
  for (size_t i = 0; i < DIAG_COUNT; ++i) {
    if (v[i] != this->diag_pub_[i]) any_changed = true;
  }
  if (!any_changed) return;

  char line[720];
  int off = 0;
  for (size_t i = 0; i < DIAG_COUNT && off < static_cast<int>(sizeof(line)); ++i) {
    off += snprintf(line + off, sizeof(line) - off, "%s%s=%u", i == 0 ? "" : " ", kNames[i], (unsigned) v[i]);
  }
  ESP_LOGI(TAG, "diag: %s", line);

#ifdef USE_SENSOR
  sensor::Sensor *const sensors[DIAG_COUNT] = {
      this->in_submitted_sensor_, this->in_done_data_sensor_,  this->in_done_empty_sensor_,
      this->in_done_err_sensor_,  this->in_ret_sent_sensor_,   this->out_submitted_sensor_,
      this->out_done_ok_sensor_,  this->unlink_rx_sensor_,     this->dropped_silent_sensor_,
      this->stale_skip_sensor_,   this->send_errors_sensor_,   this->slot_exhausted_sensor_,
      this->unlink_discarded_sensor_, this->in_resubmitted_sensor_, this->in_send_failed_sensor_,
      this->cb_queue_full_sensor_,     this->dev_lock_hold_max_sensor_,
      this->intr_q_depth_max_sensor_,  this->intr_q_full_sensor_,  this->intr_dropped_nosession_sensor_,
      this->intr_flushed_import_sensor_, this->intr_unlink_waiting_sensor_, this->out_unlink_wait_timeout_sensor_,
      this->intr_overflow_sensor_,     this->intr_zlp_dropped_sensor_,
  };
  for (size_t i = 0; i < DIAG_COUNT; ++i) {
    if (sensors[i] != nullptr && v[i] != this->diag_pub_[i]) sensors[i]->publish_state(static_cast<float>(v[i]));
  }
#endif

  for (size_t i = 0; i < DIAG_COUNT; ++i) this->diag_pub_[i] = v[i];
}

void USBIPServer::note_import_(int sock) {
  struct sockaddr_in peer {};
  socklen_t len = sizeof(peer);
  uint32_t addr = 0;
  if (getpeername(sock, reinterpret_cast<struct sockaddr *>(&peer), &len) == 0) addr = peer.sin_addr.s_addr;
  this->client_addr_ = addr != 0 ? addr : 1;  // 1 = connected, address unknown
  ++this->import_count_;
}

void USBIPServer::loop() {
  bool work = this->process_usb_events_();
  // Start the TCP task only once the network is up (setup runs before WiFi initialization).
  if (!this->tcp_started_) {
    work = true;
    if (network::is_connected()) {
      xTaskCreate(tcp_task_fn_, "usbip_tcp", TCP_TASK_STACK_SIZE, this, TCP_TASK_PRIORITY, &this->tcp_task_);
      if (this->tcp_task_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create TCP task");
        this->mark_failed();
        return;
      }
      this->tcp_started_ = true;
    }
  }
  if (!work) {
    this->disable_loop();
  }
}

void USBIPServer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "USB/IP server:\n"
                "  Port: %u\n"
                "  Bus ID: %s\n"
                "  Max transfer size: %u\n"
                "  DMA chunk size: %u\n"
                "  Max pending: %u\n"
                "  Interrupt queue depth: %u",
                this->port_, this->busid_.c_str(), (unsigned) this->max_transfer_size_,
                (unsigned) this->dma_chunk_size_, this->max_pending_, this->interrupt_queue_depth_);
  ESP_LOGCONFIG(TAG, "  TCP listening: %s", this->tcp_listening_.load() ? "YES" : "NO");
  usbip::DeviceInfo d;
  bool busy = false;
  if (this->dev_lock_ != nullptr &&
      this->get_device_info_(d, pdMS_TO_TICKS(DUMP_CONFIG_LOCK_TIMEOUT_MS), &busy)) {
    ESP_LOGCONFIG(TAG, "  Exported device: %04X:%04X (%u interfaces)", d.idVendor, d.idProduct, d.bNumInterfaces);
  } else if (busy) {
    ESP_LOGCONFIG(TAG, "  Exported device: unknown (busy)");
  } else {
    ESP_LOGCONFIG(TAG, "  Exported device: none");
  }
  usb_host::USBClient::dump_config();
}

void USBIPServer::on_connected() {
  const usb_device_desc_t *dev_desc = nullptr;
  const usb_config_desc_t *cfg_desc = nullptr;
  usb_device_info_t info;
  if (usb_host_get_device_descriptor(this->device_handle_, &dev_desc) != ESP_OK ||
      usb_host_get_active_config_descriptor(this->device_handle_, &cfg_desc) != ESP_OK ||
      usb_host_device_info(this->device_handle_, &info) != ESP_OK) {
    ESP_LOGE(TAG, "Cannot read device descriptors");
    return;
  }

  uint32_t speed;
  switch (info.speed) {
    case USB_SPEED_LOW:
      speed = usbip::kSpeedLow;
      break;
    case USB_SPEED_FULL:
      speed = usbip::kSpeedFull;
      break;
    default:
      speed = usbip::kSpeedHigh;
      break;
  }

  char path[usbip::kPathSize];
  snprintf(path, sizeof(path), "/esphome/usb/%s", this->busid_.c_str());

  // Config descriptor the client will see: complete, or with the selected interfaces only.
  const uint8_t *cfg = reinterpret_cast<const uint8_t *>(cfg_desc);
  size_t cfg_len = cfg_desc->wTotalLength;
  static uint8_t filtered[MAX_CONFIG_DESC];  // on_connected only (USB task)
  size_t filtered_len = 0;
  if (this->interface_mask_ != usbip::kAllInterfaces) {
    filtered_len = usbip::filter_config_descriptor(cfg, cfg_len, this->interface_mask_, filtered, sizeof(filtered));
    if (filtered_len == 0) {
      ESP_LOGE(TAG, "Interface filter left no valid interface (or descriptor > %u B), device not exported",
               (unsigned) MAX_CONFIG_DESC);
      return;
    }
    cfg = filtered;
    cfg_len = filtered_len;
  }

  usbip::DeviceInfo d;
  if (!usbip::device_info_from_descriptors(reinterpret_cast<const uint8_t *>(dev_desc), sizeof(usb_device_desc_t),
                                           cfg, cfg_len, 1, info.dev_addr, speed, this->busid_.c_str(), path, d)) {
    ESP_LOGE(TAG, "Invalid device descriptors, device not exported");
    return;
  }

  usbip::EndpointInfo eps[usbip::kMaxEndpoints];
  size_t ep_count = 0;
  if (!usbip::parse_endpoints(cfg, cfg_len, eps, usbip::kMaxEndpoints, ep_count)) {
    ESP_LOGE(TAG, "Cannot parse endpoints, device not exported");
    return;
  }

  // Claim the exported interfaces (alt 0) by their numbers from the original descriptor.
  this->claimed_mask_ = 0;
  for (uint8_t i = 0; i < cfg_desc->bNumInterfaces && i < 32; ++i) {
    if (!usbip::interface_exported(this->interface_mask_, i)) continue;
    esp_err_t err = usb_host_interface_claim(this->handle_, this->device_handle_, i, 0);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Interface %u claim failed: %s, device not exported", i, esp_err_to_name(err));
      this->release_interfaces_();
      return;
    }
    this->claimed_mask_ |= 1u << i;
  }
  if (filtered_len > 0)
    ESP_LOGI(TAG, "Interface filter: exporting %u of %u interfaces", d.bNumInterfaces, cfg_desc->bNumInterfaces);

  this->take_dev_lock_from_loop_(KIND_CONNECT_WAIT);
  if (filtered_len > 0) memcpy(this->cfg_filtered_, filtered, filtered_len);
  this->cfg_filtered_len_ = static_cast<uint16_t>(filtered_len);
  this->dev_info_ = d;
  for (size_t i = 0; i < ep_count; ++i) this->eps_[i] = eps[i];
  this->ep_count_ = ep_count;
  this->dev_ready_ = true;
  this->dev_ready_pub_.store(true, std::memory_order_relaxed);
  this->dev_generation_.fetch_add(1, std::memory_order_relaxed);  // the TCP task sets up intr_in_ again
  xSemaphoreGive(this->dev_lock_);

  ESP_LOGI(TAG, "Exporting %04X:%04X as busid %s (%u interfaces claimed, %u endpoints, speed %u)", d.idVendor,
           d.idProduct, d.busid, d.bNumInterfaces, (unsigned) ep_count, (unsigned) d.speed);
}

bool USBIPServer::teardown() {
  this->shutdown_requested_ = true;
  if (this->session_active_) return false;  // TCP task is still closing the session
  // The FIN is queued in lwIP; wait briefly for the tcpip thread to send it
  // (ESPHome gives teardown() at most 1 s in total, TEARDOWN_TIMEOUT_REBOOT_MS).
  return xTaskGetTickCount() - this->session_closed_tick_ >= pdMS_TO_TICKS(100);
}

void USBIPServer::release_interfaces_() {
  for (uint8_t i = 0; i < 32; ++i) {
    if ((this->claimed_mask_ & (1u << i)) == 0) continue;
    esp_err_t err = usb_host_interface_release(this->handle_, this->device_handle_, i);
    if (err != ESP_OK) ESP_LOGW(TAG, "Interface %u release failed: %s", i, esp_err_to_name(err));
  }
  this->claimed_mask_ = 0;
}

void USBIPServer::on_disconnected() {
  // Stop new transfers first (the TCP task submits under the same lock).
  this->take_dev_lock_from_loop_(KIND_DISCONNECT_WAIT);
  this->dev_ready_ = false;
  this->dev_ready_pub_.store(false, std::memory_order_relaxed);
  xSemaphoreGive(this->dev_lock_);
  // usb_host_interface_release refuses an interface with transfers in flight (ESP_ERR_INVALID_STATE).
  // The stack flushes the prefetch transfers of a gone device with NO_DEVICE (no halt wait, the port
  // is invalid) and prefetch_cb_ runs in the USB task, so a short wait here is enough. No new prefetch
  // can start: submit_locked_ checks dev_ready_ under dev_lock_.
  for (uint32_t waited = 0; this->prefetch_in_flight_.load(std::memory_order_acquire) != 0 &&
                            waited < DISCONNECT_PREFETCH_WAIT_MS;
       waited += DISCONNECT_PREFETCH_POLL_MS) {
    vTaskDelay(pdMS_TO_TICKS(DISCONNECT_PREFETCH_POLL_MS));
  }
  const uint32_t left = this->prefetch_in_flight_.load(std::memory_order_acquire);
  if (left != 0) {
    ESP_LOGW(TAG, "%u prefetch transfer(s) still in flight after %u ms, releasing interfaces anyway", (unsigned) left,
             (unsigned) DISCONNECT_PREFETCH_WAIT_MS);
  }
  this->release_interfaces_();
  ESP_LOGW(TAG, "USB device gone, nothing to export");
  usb_host::USBClient::on_disconnected();
}

bool USBIPServer::get_device_info_(usbip::DeviceInfo &out, TickType_t timeout_ticks, bool *busy) {
  if (xSemaphoreTake(this->dev_lock_, timeout_ticks) != pdTRUE) {
    if (busy != nullptr) *busy = true;
    return false;
  }
  bool ready = this->dev_ready_;
  if (ready) out = this->dev_info_;
  xSemaphoreGive(this->dev_lock_);
  return ready;
}

bool USBIPServer::get_endpoint_(uint8_t address, usbip::EndpointInfo &out) {
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
  const usbip::EndpointInfo *e =
      this->dev_ready_ ? usbip::find_endpoint(this->eps_, this->ep_count_, address) : nullptr;
  if (e != nullptr) out = *e;
  xSemaphoreGive(this->dev_lock_);
  return e != nullptr;
}

bool USBIPServer::device_ready_() {
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
  bool ready = this->dev_ready_;
  xSemaphoreGive(this->dev_lock_);
  return ready;
}

// ---------------------------------------------------------------- USB task

void USBIPServer::transfer_cb_(usb_transfer_t *xfer) {
  auto *slot = static_cast<PendingSlot *>(xfer->context);
  // The queue has room for every slot. Runs in usb_task (a FreeRTOS task, not an ISR).
  if (xQueueSend(slot->owner->done_queue_, &slot->index, 0) != pdTRUE) {
    slot->owner->cb_queue_full_.fetch_add(1, std::memory_order_relaxed);
  }
}

// USB task. Same as transfer_cb_, for the prefetch transfers of interrupt IN endpoints.
void USBIPServer::prefetch_cb_(usb_transfer_t *xfer) {
  auto *pf = static_cast<PrefetchXfer *>(xfer->context);
  USBIPServer *s = pf->owner;
  if (xQueueSend(s->done_queue_, &pf->index, 0) != pdTRUE) {
    s->cb_queue_full_.fetch_add(1, std::memory_order_relaxed);
  }
  s->prefetch_in_flight_.fetch_sub(1, std::memory_order_release);
}

// ---------------------------------------------------------------- TCP task

void USBIPServer::tcp_task_fn_(void *arg) { static_cast<USBIPServer *>(arg)->tcp_task_loop_(); }

bool USBIPServer::send_all_(int sock, const uint8_t *data, size_t len) {
  while (len > 0) {
    int n = send(sock, data, len, 0);
    if (n <= 0) return false;
    data += n;
    len -= static_cast<size_t>(n);
  }
  return true;
}

bool USBIPServer::send_ret_submit_(int sock, uint32_t seqnum, uint32_t start_frame, uint32_t number_of_packets,
                                   int32_t status, uint32_t actual_length, const uint8_t *in_data) {
  usbip::RetSubmit r;
  r.seqnum = seqnum;
  r.status = status;
  r.actual_length = actual_length;
  r.start_frame = start_frame;
  r.number_of_packets = number_of_packets;
  uint8_t hdr[usbip::kUrbHeaderSize];
  usbip::encode_ret_submit(r, hdr);
  if (!send_all_(sock, hdr, sizeof(hdr))) return false;
  if (in_data != nullptr && actual_length > 0) return send_all_(sock, in_data, actual_length);
  return true;
}

bool USBIPServer::send_ret_unlink_(int sock, uint32_t seqnum, int32_t status) {
  uint8_t hdr[usbip::kUrbHeaderSize];
  usbip::encode_ret_unlink(seqnum, status, hdr);
  return send_all_(sock, hdr, sizeof(hdr));
}

// Mapping follows usbipdcpp_esp32 (trxstat2error), values are Linux errno.
int32_t USBIPServer::map_status_(usb_transfer_status_t status) {
  switch (status) {
    case USB_TRANSFER_STATUS_COMPLETED:
      return 0;
    case USB_TRANSFER_STATUS_CANCELED:
      return usbip::kLinuxECONNRESET;
    case USB_TRANSFER_STATUS_ERROR:
    case USB_TRANSFER_STATUS_STALL:
    case USB_TRANSFER_STATUS_TIMED_OUT:
    case USB_TRANSFER_STATUS_OVERFLOW:
      return usbip::kLinuxEPIPE;
    case USB_TRANSFER_STATUS_NO_DEVICE:
      return usbip::kLinuxESHUTDOWN;
    default:
      return usbip::kLinuxENOENT;
  }
}

USBIPServer::PendingSlot *USBIPServer::alloc_slot_() {
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    if (!this->slots_[i].used) {
      PendingSlot &s = this->slots_[i];
      USBIPServer *owner = s.owner;
      uint8_t index = s.index;
      s = PendingSlot{};
      s.owner = owner;
      s.index = index;
      s.used = true;
      return &s;
    }
  }
  return nullptr;
}

void USBIPServer::free_slot_(PendingSlot &slot) {
  if (slot.xfer != nullptr) usb_host_transfer_free(slot.xfer);
  slot.xfer = nullptr;
  if (slot.big_buf != nullptr) free(slot.big_buf);  // NOLINT
  slot.big_buf = nullptr;
  slot.chunked = false;
  slot.used = false;
}

void USBIPServer::tcp_task_loop_() {
  int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_sock < 0) {
    ESP_LOGE(TAG, "socket() failed: errno %d", errno);
    vTaskDelete(nullptr);
    return;
  }
  int opt = 1;
  setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->port_);
  if (bind(listen_sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0 ||
      listen(listen_sock, 1) != 0) {
    ESP_LOGE(TAG, "bind/listen on port %u failed: errno %d", this->port_, errno);
    close(listen_sock);
    vTaskDelete(nullptr);
    return;
  }
  this->tcp_listening_ = true;
  ESP_LOGI(TAG, "Listening on TCP port %u", this->port_);

  while (true) {
    struct sockaddr_in client {};
    socklen_t client_len = sizeof(client);
    int sock = accept(listen_sock, reinterpret_cast<struct sockaddr *>(&client), &client_len);
    if (sock < 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    char ip[16];
    inet_ntoa_r(client.sin_addr, ip, sizeof(ip));
    ESP_LOGI(TAG, "Client %s connected", ip);
    configure_session_socket_(sock);
    this->session_active_ = true;

    // A new import may take over the session; then we continue with the new socket.
    char takeover_busid[usbip::kBusIdSize];
    const char *pending = nullptr;
    while (sock >= 0) {
      int next = this->serve_client_(sock, listen_sock, pending);
      shutdown(sock, SHUT_RDWR);
      close(sock);
      ESP_LOGI(TAG, "Session closed");
      sock = next;
      pending = nullptr;
      if (sock >= 0 && this->shutdown_requested_) {
        shutdown(sock, SHUT_RDWR);
        close(sock);
        sock = -1;
      }
      if (sock >= 0) {
        memcpy(takeover_busid, this->takeover_busid_, usbip::kBusIdSize);
        pending = takeover_busid;
      }
    }
    this->session_closed_tick_ = xTaskGetTickCount();
    this->session_active_ = false;
    if (this->shutdown_requested_) {
      // A reboot is under way: stop accepting new connections.
      close(listen_sock);
      this->tcp_listening_ = false;
      vTaskSuspend(nullptr);
    }
  }
}

// TCP_NODELAY + keepalive: a dead client (network outage, HA restart) is detected
// within ~25 s instead of waiting forever (LWIP_TCP_KEEPALIVE=1 in ESP-IDF lwipopts.h).
void USBIPServer::configure_session_socket_(int sock) {
  int one = 1;
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
  int idle = 10, intvl = 5, cnt = 3;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
}

// Reads exactly len bytes (timeout per the socket SO_RCVTIMEO).
static bool recv_exact(int sock, uint8_t *buf, size_t len) {
  while (len > 0) {
    int n = recv(sock, buf, len, 0);
    if (n <= 0) return false;
    buf += n;
    len -= static_cast<size_t>(n);
  }
  return true;
}

int USBIPServer::handle_side_connection_(int listen_sock, char *busid_out) {
  struct sockaddr_in client {};
  socklen_t client_len = sizeof(client);
  int s = accept(listen_sock, reinterpret_cast<struct sockaddr *>(&client), &client_len);
  if (s < 0) return -1;
  char ip[16];
  inet_ntoa_r(client.sin_addr, ip, sizeof(ip));
  struct timeval tv {};
  tv.tv_sec = 1;  // short timeout so a slow client cannot block us
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  uint8_t hdr[usbip::kOpHeaderSize];
  if (!recv_exact(s, hdr, sizeof(hdr))) {
    close(s);
    return -1;
  }
  const usbip::OpHeader op = usbip::decode_op_header(hdr);
  if (op.version == usbip::kVersion && op.code == usbip::kOpReqDevlist) {
    usbip::DeviceInfo d;
    bool have = this->get_device_info_(d);
    uint8_t out[usbip::kOpHeaderSize + 4 + usbip::kDeviceInfoSize + usbip::kInterfaceInfoSize * usbip::kMaxInterfaces];
    size_t len = usbip::encode_rep_devlist(have ? &d : nullptr, out, sizeof(out));
    if (len > 0) send_all_(s, out, len);
    ESP_LOGI(TAG, "Client %s: OP_REQ_DEVLIST during active session -> answered", ip);
    shutdown(s, SHUT_RDWR);
    close(s);
    return -1;
  }
  uint8_t raw_busid[usbip::kBusIdSize];
  if (op.version == usbip::kVersion && op.code == usbip::kOpReqImport && recv_exact(s, raw_busid, sizeof(raw_busid)) &&
      usbip::decode_req_import_busid(raw_busid, busid_out)) {
    ESP_LOGW(TAG, "Client %s: OP_REQ_IMPORT '%s' takes over the active session", ip, busid_out);
    tv.tv_sec = 0;  // back to reading without a timeout (the session uses select)
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    configure_session_socket_(s);
    return s;
  }
  ESP_LOGW(TAG, "Client %s: unexpected request during active session, closed", ip);
  close(s);
  return -1;
}

int USBIPServer::serve_client_(int sock, int listen_sock, const char *takeover_busid) {
  usbip::StreamParser parser(this->rx_payload_, this->max_transfer_size_);
  int next_sock = -1;
  uint8_t buf[512];
  bool imported = false;
  bool keep = true;
  if (takeover_busid != nullptr) {
    // handle_side_connection_ already read the import; reply the same way as for kReqImport.
    usbip::DeviceInfo d;
    bool have = this->get_device_info_(d);
    bool match = have && strncmp(takeover_busid, d.busid, usbip::kBusIdSize) == 0;
    uint8_t out[usbip::kRepImportOkSize];
    size_t len = usbip::encode_rep_import(match ? &d : nullptr, out, sizeof(out));
    if (len == 0 || !send_all_(sock, out, len) || !match) {
      ESP_LOGW(TAG, "OP_REQ_IMPORT busid '%s' refused (%s)", takeover_busid, have ? "busid mismatch" : "no device");
      keep = false;
    } else {
      parser.set_imported();
      imported = true;
      this->intr_on_import_();
      this->note_import_(sock);
      ESP_LOGI(TAG, "OP_REQ_IMPORT busid '%s' -> imported %04X:%04X (takeover)", takeover_busid, d.idVendor,
               d.idProduct);
    }
  }

  while (keep) {
    if (this->shutdown_requested_) {
      ESP_LOGI(TAG, "Shutdown requested, closing USB/IP connection");
      break;
    }
    if (!this->drain_completions_(sock)) break;
    if (!this->intr_service_(sock)) break;
    if (imported && !this->device_ready_()) {
      ESP_LOGW(TAG, "USB device gone, closing USB/IP connection");
      break;
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    FD_SET(listen_sock, &rfds);
    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = SELECT_TIMEOUT_US;
    int r = select(std::max(sock, listen_sock) + 1, &rfds, nullptr, nullptr, &tv);
    if (r < 0) break;
    if (r == 0) continue;

    // New connection during a session: usbip list, or a new import after a client outage.
    if (FD_ISSET(listen_sock, &rfds)) {
      int s = this->handle_side_connection_(listen_sock, this->takeover_busid_);
      if (s >= 0) {
        next_sock = s;  // the new import takes over the device, this session ends
        break;
      }
    }
    if (!FD_ISSET(sock, &rfds)) continue;

    int n = recv(sock, buf, sizeof(buf), 0);
    if (n <= 0) {
      ESP_LOGV(TAG, "recv returned %d (errno %d), closing session", n, errno);
      break;
    }

    size_t off = 0;
    while (keep && off < static_cast<size_t>(n)) {
      usbip::Message m;
      size_t used = parser.feed(buf + off, static_cast<size_t>(n) - off, m);
      off += used;
      switch (m.type) {
        case usbip::MsgType::kNone:
          break;
        case usbip::MsgType::kReqDevlist: {
          usbip::DeviceInfo d;
          bool have = this->get_device_info_(d);
          uint8_t out[usbip::kOpHeaderSize + 4 + usbip::kDeviceInfoSize +
                      usbip::kInterfaceInfoSize * usbip::kMaxInterfaces];
          size_t len = usbip::encode_rep_devlist(have ? &d : nullptr, out, sizeof(out));
          ESP_LOGI(TAG, "OP_REQ_DEVLIST -> %u device(s), %u B", have ? 1u : 0u, (unsigned) len);
          keep = len != 0 && send_all_(sock, out, len);
          break;
        }
        case usbip::MsgType::kReqImport: {
          usbip::DeviceInfo d;
          bool have = this->get_device_info_(d);
          bool match = have && strncmp(m.busid, d.busid, usbip::kBusIdSize) == 0;
          uint8_t out[usbip::kRepImportOkSize];
          size_t len = usbip::encode_rep_import(match ? &d : nullptr, out, sizeof(out));
          if (len == 0 || !send_all_(sock, out, len) || !match) {
            ESP_LOGW(TAG, "OP_REQ_IMPORT busid '%s' refused (%s)", m.busid, have ? "busid mismatch" : "no device");
            keep = false;
            break;
          }
          parser.set_imported();
          imported = true;
          this->intr_on_import_();
          this->note_import_(sock);
          ESP_LOGI(TAG, "OP_REQ_IMPORT busid '%s' -> imported %04X:%04X", m.busid, d.idVendor, d.idProduct);
          break;
        }
        case usbip::MsgType::kCmdSubmit:
          keep = this->handle_submit_(sock, m);
          break;
        case usbip::MsgType::kCmdUnlink:
          keep = this->handle_unlink_(sock, m);
          break;
        case usbip::MsgType::kError:
          ESP_LOGW(TAG, "Protocol error: %s, closing connection", parse_error_str(m.error));
          keep = false;
          break;
      }
      if (used == 0) keep = false;
    }
  }
  this->end_session_();
  return next_sock;
}

esp_err_t USBIPServer::submit_locked_(usb_transfer_t *x, bool is_control) {
  DevLockHold hold(this, LockSite::SUBMIT);
  if (!this->dev_ready_) return ESP_ERR_INVALID_STATE;
  x->device_handle = this->device_handle_;
  return is_control ? usb_host_transfer_submit_control(this->handle_, x) : usb_host_transfer_submit(x);
}

// Cancels all transfers on the endpoint (ESP-IDF cannot cancel a single transfer). Bulk endpoints only:
// halting a periodic (interrupt) pipe can wait forever in ESP-IDF 5.5.
// Sequence halt -> flush -> clear follows usbipdcpp_esp32 (cancel_endpoint_all_transfers).
void USBIPServer::cancel_endpoint_(uint8_t ep_addr) {
  DevLockHold hold(this, LockSite::CANCEL);
  if (this->dev_ready_) {
    esp_err_t e1 = usb_host_endpoint_halt(this->device_handle_, ep_addr);
    esp_err_t e2 = usb_host_endpoint_flush(this->device_handle_, ep_addr);
    esp_err_t e3 = usb_host_endpoint_clear(this->device_handle_, ep_addr);
    if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
      ESP_LOGW(TAG, "Cancel ep %02X: halt %s, flush %s, clear %s", ep_addr, esp_err_to_name(e1), esp_err_to_name(e2),
               esp_err_to_name(e3));
    } else {
      ESP_LOGV(TAG, "Cancel ep %02X: ok", ep_addr);
    }
  }
}

uint8_t *USBIPServer::alloc_big_(size_t size) {
  void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (p == nullptr) p = malloc(size);  // NOLINT - without PSRAM try internal RAM
  return static_cast<uint8_t *>(p);
}

void USBIPServer::note_request_size_(uint32_t size) {
  if (size > this->max_seen_request_) {
    this->max_seen_request_ = size;
    ESP_LOGI(TAG, "Largest transfer requested so far: %u B", (unsigned) size);
  }
}

bool USBIPServer::handle_data_submit_(int sock, const usbip::Message &m) {
  const usbip::CmdSubmit &c = m.submit;
  const uint32_t seq = c.base.seqnum;
  const bool dir_in = c.base.direction == usbip::kDirIn;
  const uint8_t addr = usbip::endpoint_address(c.base.ep, c.base.direction);

  usbip::EndpointInfo e;
  if (!this->get_endpoint_(addr, e)) {
    ESP_LOGW(TAG, "seq %u: unknown endpoint %02X -> -EPIPE", (unsigned) seq, addr);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  if (e.type != usbip::kEpTypeBulk && e.type != usbip::kEpTypeInterrupt) {
    ESP_LOGW(TAG, "seq %u: endpoint %02X type %u not supported -> -EPIPE", (unsigned) seq, addr, e.type);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }

  const uint32_t requested = c.transfer_buffer_length;
  const uint32_t num_bytes = dir_in ? usbip::round_up_to_mps(requested, e.mps) : requested;
  this->note_request_size_(requested);
  if (requested > this->max_transfer_size_) {
    ESP_LOGW(TAG, "seq %u: ep %02X transfer %u B > max_transfer_size %u -> -EPIPE", (unsigned) seq, addr,
             (unsigned) requested, (unsigned) this->max_transfer_size_);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  // Bulk above dma_chunk_size is split into chunks; interrupt is not split.
  const bool chunked = e.type == usbip::kEpTypeBulk && num_bytes > this->dma_chunk_size_;
  if (!chunked && num_bytes > this->dma_chunk_size_) {
    ESP_LOGW(TAG, "seq %u: interrupt ep %02X transfer %u B > dma_chunk_size %u -> -EPIPE", (unsigned) seq, addr,
             (unsigned) num_bytes, (unsigned) this->dma_chunk_size_);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  // Interrupt IN: answered from the prefetch ring, no USB transfer per request (no HALT on UNLINK).
  if (e.type == usbip::kEpTypeInterrupt && dir_in) return this->handle_intr_in_submit_(sock, m, e);

  const bool zero_packet = (c.transfer_flags & usbip::kUrbZeroPacket) != 0;
  usbip::ChunkPlan plan(requested, this->dma_chunk_size_, e.mps, dir_in, zero_packet);

  PendingSlot *slot = this->alloc_slot_();
  if (slot == nullptr) {
    this->slot_exhausted_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(TAG, "seq %u: too many pending transfers (max_pending %u) -> -EPIPE", (unsigned) seq, this->max_pending_);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }

  uint8_t *big = nullptr;
  if (chunked) {
    big = alloc_big_(requested);
    if (big == nullptr) {
      slot->used = false;
      ESP_LOGW(TAG, "seq %u: buffer alloc of %u B failed -> -EPIPE", (unsigned) seq, (unsigned) requested);
      return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
    }
    // OUT data must be copied: the parser buffer is overwritten by the next message.
    if (!dir_in && m.payload != nullptr) memcpy(big, m.payload, std::min<uint32_t>(m.payload_len, requested));
  }

  usb_transfer_t *x = nullptr;
  // Chunked: DMA buffer for one chunk (cap is a multiple of MPS). Otherwise the whole transfer;
  // even a zero-length transfer needs a buffer.
  const size_t alloc = chunked ? plan.cap() : std::max<size_t>(num_bytes, e.mps);
  if (usb_host_transfer_alloc(alloc, 0, &x) != ESP_OK || x == nullptr) {
    if (big != nullptr) free(big);  // NOLINT
    slot->used = false;
    ESP_LOGW(TAG, "seq %u: transfer alloc of %u B failed -> -EPIPE", (unsigned) seq, (unsigned) alloc);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  if (chunked) {
    if (!dir_in) memcpy(x->data_buffer, big, plan.request_len());
    x->num_bytes = static_cast<int>(plan.submit_len());
    x->flags = plan.zero_pack() ? USB_TRANSFER_FLAG_ZERO_PACK : 0;
  } else {
    if (!dir_in && requested > 0 && m.payload != nullptr) {
      memcpy(x->data_buffer, m.payload, std::min<uint32_t>(m.payload_len, requested));
    }
    x->num_bytes = static_cast<int>(num_bytes);
    // Zero-length packet only if the client requests it (URB_ZERO_PACKET), as in Linux.
    x->flags = (!dir_in && zero_packet) ? USB_TRANSFER_FLAG_ZERO_PACK : 0;
  }
  x->bEndpointAddress = addr;
  x->callback = transfer_cb_;
  x->context = slot;

  slot->xfer = x;
  slot->seqnum = seq;
  slot->start_frame = c.start_frame;
  slot->number_of_packets = c.number_of_packets;
  slot->requested = requested;
  slot->dir_in = dir_in;
  slot->is_control = false;
  slot->is_intr = e.type == usbip::kEpTypeInterrupt;  // interrupt OUT (IN returned above)
  slot->ep_addr = addr;
  slot->chunked = chunked;
  slot->big_buf = big;
  slot->plan = plan;

  if (chunked) {
    ESP_LOGV(TAG, "seq %u: BULK ep %02X len %u (chunked by %u)", (unsigned) seq, addr, (unsigned) requested,
             (unsigned) plan.cap());
  } else {
    ESP_LOGV(TAG, "seq %u: %s ep %02X len %u (submit %u)", (unsigned) seq,
             e.type == usbip::kEpTypeBulk ? "BULK" : "INTR", addr, (unsigned) requested, (unsigned) num_bytes);
  }

  esp_err_t err = this->submit_locked_(x, false);
  if (err != ESP_OK) {
    this->free_slot_(*slot);
    ESP_LOGW(TAG, "seq %u: ep %02X submit failed: %s -> -EPIPE", (unsigned) seq, addr, esp_err_to_name(err));
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  if (dir_in) {
    this->in_submitted_.fetch_add(1, std::memory_order_relaxed);
  } else {
    this->out_submitted_.fetch_add(1, std::memory_order_relaxed);
  }
  return true;
}

bool USBIPServer::handle_submit_(int sock, const usbip::Message &m) {
  const usbip::CmdSubmit &c = m.submit;
  const uint32_t seq = c.base.seqnum;

  if (c.base.ep != 0) return this->handle_data_submit_(sock, m);

  const usbip::SetupInfo si = usbip::parse_setup(c.setup);
  const usbip::ControlAction a = usbip::classify_control(c.setup);
  ESP_LOGV(TAG, "seq %u: CTRL %02X %02X %02X%02X %02X%02X len %u", (unsigned) seq, c.setup[0], c.setup[1],
           c.setup[3], c.setup[2], c.setup[5], c.setup[4], si.wLength);

  // With an interface filter the server sends the config descriptor itself (the client must not
  // see the hidden interfaces). Copy under the lock, send without it.
  uint8_t cfg_index = 0;
  if (usbip::is_get_config_descriptor(c.setup, cfg_index)) {
    uint8_t cfg[MAX_CONFIG_DESC];
    uint16_t len = 0;
    xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
    if (this->dev_ready_ && this->cfg_filtered_len_ > 0) {
      len = std::min<uint16_t>(this->cfg_filtered_len_, si.wLength);
      memcpy(cfg, this->cfg_filtered_, len);
    }
    const bool filtered = this->cfg_filtered_len_ > 0;
    const bool ready = this->dev_ready_;
    xSemaphoreGive(this->dev_lock_);
    if (filtered) {
      if (!ready || cfg_index != 0) {
        ESP_LOGW(TAG, "seq %u: GET_DESCRIPTOR(CONFIG %u) -> -EPIPE", (unsigned) seq, cfg_index);
        return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
      }
      ESP_LOGV(TAG, "seq %u: GET_DESCRIPTOR(CONFIG) -> %u B filtered (not forwarded)", (unsigned) seq, len);
      return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, 0, len, cfg);
    }
  }

  uint8_t clear_halt_ep = 0;
  switch (a.kind) {
    case usbip::ControlKind::kSetConfiguration: {
      usbip::DeviceInfo d;
      bool ok = this->get_device_info_(d) && a.configuration == d.bConfigurationValue;
      ESP_LOGI(TAG, "SET_CONFIGURATION(%u) -> %s (not forwarded)", a.configuration, ok ? "ok" : "refused");
      return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, ok ? 0 : usbip::kLinuxEPIPE, 0, nullptr);
    }
    case usbip::ControlKind::kSetInterface: {
      bool ok = a.alternate == 0;
      ESP_LOGI(TAG, "SET_INTERFACE(%u, alt %u) -> %s (not forwarded)", a.interface, a.alternate,
               ok ? "ok" : "refused");
      return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, ok ? 0 : usbip::kLinuxEPIPE, 0, nullptr);
    }
    case usbip::ControlKind::kResetDevice:
      ESP_LOGW(TAG, "Port reset requested -> acknowledged without action");
      return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, 0, 0, nullptr);
    case usbip::ControlKind::kClearHalt: {
      // Interrupt IN with prefetch: the host pipe is only halted after a STALL. Otherwise the request
      // is answered here: forwarding it would reset the data toggle on the device but not on the host
      // pipe (usb_host_endpoint_clear needs a halted pipe), and the next packet would be lost.
      IntrInEp *iep = this->find_intr_in_(a.endpoint);
      if (iep != nullptr && !iep->stalled) {
        ESP_LOGD(TAG, "seq %u: CLEAR_FEATURE(HALT) ep %02X not stalled -> ok (not forwarded)", (unsigned) seq,
                 a.endpoint);
        return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, 0, 0, nullptr);
      }
      clear_halt_ep = a.endpoint;  // forward to the device and on success reset the pipe on the host
      break;
    }
    case usbip::ControlKind::kForward:
      break;
  }

  if (si.wLength > this->dma_chunk_size_) {
    ESP_LOGW(TAG, "seq %u: control wLength %u > dma_chunk_size -> -EPIPE", (unsigned) seq, si.wLength);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }

  PendingSlot *slot = this->alloc_slot_();
  if (slot == nullptr) {
    ESP_LOGW(TAG, "seq %u: too many pending transfers (max_pending %u) -> -EPIPE", (unsigned) seq, this->max_pending_);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }

  usb_transfer_t *x = nullptr;
  const size_t size = usbip::kSetupPacketSize + si.wLength;
  if (usb_host_transfer_alloc(size, 0, &x) != ESP_OK || x == nullptr) {
    slot->used = false;
    ESP_LOGW(TAG, "seq %u: transfer alloc of %u B failed -> -EPIPE", (unsigned) seq, (unsigned) size);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  memcpy(x->data_buffer, c.setup, usbip::kSetupPacketSize);
  if (!si.dir_in && si.wLength > 0 && m.payload != nullptr) {
    memcpy(x->data_buffer + usbip::kSetupPacketSize, m.payload, std::min<uint32_t>(m.payload_len, si.wLength));
  }
  x->num_bytes = static_cast<int>(size);
  x->bEndpointAddress = 0;
  x->callback = transfer_cb_;
  x->context = slot;

  slot->xfer = x;
  slot->seqnum = seq;
  slot->start_frame = c.start_frame;
  slot->number_of_packets = c.number_of_packets;
  slot->requested = c.transfer_buffer_length;
  slot->dir_in = si.dir_in;
  slot->is_control = true;
  slot->clear_halt_ep = clear_halt_ep;

  esp_err_t err = this->submit_locked_(x, true);
  if (err != ESP_OK) {
    this->free_slot_(*slot);
    ESP_LOGW(TAG, "seq %u: control submit failed: %s -> -EPIPE", (unsigned) seq, esp_err_to_name(err));
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  this->note_request_size_(si.wLength);
  return true;
}

bool USBIPServer::handle_unlink_(int sock, const usbip::Message &m) {
  // Counts every CMD_UNLINK received, regardless of whether the target turns out to be a
  // control (ep0) or data-endpoint transfer -- UNLINK itself isn't tied to one endpoint type
  // until the target slot is found below, so this one counter is not restricted to data
  // endpoints like the others. Flagging this choice per the task's "len pre data endpointy"
  // framing, which does not unambiguously cover this counter.
  this->unlink_rx_.fetch_add(1, std::memory_order_relaxed);
  const uint32_t target = m.unlink.unlink_seqnum;
  // Interrupt IN request still waiting for data: no USB transfer belongs to it, just drop it. It is
  // out of the FIFO now, so no RET_SUBMIT can follow this RET_UNLINK (Linux vhci_rx.c would drop the
  // connection on a RET_SUBMIT for an unknown seqnum).
  for (IntrInEp &ep : this->intr_in_) {
    if (ep.active && ep.q.unlink(target)) {
      this->intr_unlink_waiting_.fetch_add(1, std::memory_order_relaxed);
      ESP_LOGV(TAG, "UNLINK seq %u (interrupt IN, waiting) -> RET_UNLINK", (unsigned) target);
      return send_ret_unlink_(sock, m.unlink.base.seqnum, usbip::kLinuxECONNRESET);
    }
  }
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    PendingSlot &s = this->slots_[i];
    if (s.used && !s.orphan && s.seqnum == target) {
      s.unlink_pending = true;
      s.unlink_seqnum = m.unlink.base.seqnum;
      if (s.is_control) {
        // A control transfer on ep0 cannot be cancelled in ESP-IDF (usbh_ep_get_handle does not
        // know ep0). Wait for completion and send RET_UNLINK instead of RET_SUBMIT.
        ESP_LOGV(TAG, "UNLINK seq %u (ep0) -> reply on completion", (unsigned) target);
      } else if (s.is_intr) {
        // Interrupt OUT: never halt a periodic pipe. Wait for the completion (RET_UNLINK then), at most
        // OUT_UNLINK_WAIT_MS (intr_service_ answers and orphans the slot).
        s.unlink_deadline_ms = now_ms_() + OUT_UNLINK_WAIT_MS;
        ESP_LOGV(TAG, "UNLINK seq %u (interrupt OUT ep %02X) -> reply on completion", (unsigned) target, s.ep_addr);
      } else {
        // Bulk: cancels all transfers on the endpoint; those without unlink_pending are resubmitted.
        ESP_LOGV(TAG, "UNLINK seq %u -> cancel ep %02X", (unsigned) target, s.ep_addr);
        this->cancel_endpoint_(s.ep_addr);
      }
      return true;
    }
  }
  ESP_LOGV(TAG, "UNLINK seq %u -> already completed", (unsigned) target);
  return send_ret_unlink_(sock, m.unlink.base.seqnum, 0);
}

bool USBIPServer::drain_completions_(int sock) {
  uint8_t idx;
  bool ok = true;
  while (xQueueReceive(this->done_queue_, &idx, 0) == pdTRUE) {
    if (idx >= MAX_PENDING_LIMIT) {
      // Prefetch transfer of an interrupt IN endpoint. After a failed send no more replies go out.
      ok = this->handle_prefetch_done_(ok ? sock : -1, static_cast<uint8_t>(idx - MAX_PENDING_LIMIT)) && ok;
      continue;
    }
    PendingSlot &s = this->slots_[idx];
    usb_transfer_t *x = s.xfer;
    if (!s.used || x == nullptr) {
      // done_queue_ entry for a slot that is already free or has no xfer. The mechanism that would
      // produce this is not established (the next chunk is only submitted after the previous entry
      // is drained, so duplicates are not obvious); it is counted as a defensive guard. Not gated
      // on s.is_control because s may already have been reset/reused, so its fields are meaningless.
      this->stale_skip_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    // Transfer cancelled only because of an UNLINK of another transfer on the same endpoint: resubmit.
    // (Bulk only: interrupt endpoints are never cancelled.)
    if (x->status == USB_TRANSFER_STATUS_CANCELED && !s.is_control && !s.is_intr && !s.unlink_pending && !s.orphan) {
      // The cancelled completion is an IN completion with status != COMPLETED: counted here as
      // in_done_err because this branch `continue`s before the classification block below.
      if (s.dir_in) this->in_done_err_.fetch_add(1, std::memory_order_relaxed);
      x->status = USB_TRANSFER_STATUS_COMPLETED;
      if (this->submit_locked_(x, false) == ESP_OK) {
        if (s.dir_in) this->in_resubmitted_.fetch_add(1, std::memory_order_relaxed);
        ESP_LOGV(TAG, "seq %u: collateral cancel on ep %02X -> resubmitted", (unsigned) s.seqnum, s.ep_addr);
        continue;
      }
      ESP_LOGW(TAG, "seq %u: resubmit on ep %02X failed -> -EPIPE", (unsigned) s.seqnum, s.ep_addr);
      if (ok && sock >= 0) {
        ok = send_ret_submit_(sock, s.seqnum, s.start_frame, s.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
        if (!ok) this->send_errors_.fetch_add(1, std::memory_order_relaxed);
      }
      this->free_slot_(s);
      continue;
    }

    // Chunked: successful chunk -> store the data and submit the next chunk.
    bool chunk_submit_failed = false;
    if (s.chunked && x->status == USB_TRANSFER_STATUS_COMPLETED && !s.unlink_pending && !s.orphan) {
      const uint32_t off = s.plan.offset();
      const uint32_t got = s.plan.complete(x->actual_num_bytes < 0 ? 0 : static_cast<uint32_t>(x->actual_num_bytes));
      if (s.dir_in && got > 0) memcpy(s.big_buf + off, x->data_buffer, got);
      if (!s.plan.finished()) {
        x->num_bytes = static_cast<int>(s.plan.submit_len());
        x->flags = s.plan.zero_pack() ? USB_TRANSFER_FLAG_ZERO_PACK : 0;
        if (!s.dir_in) memcpy(x->data_buffer, s.big_buf + s.plan.offset(), s.plan.request_len());
        if (this->submit_locked_(x, false) == ESP_OK) continue;
        ESP_LOGW(TAG, "seq %u: next chunk on ep %02X failed -> -EPIPE after %u B", (unsigned) s.seqnum, s.ep_addr,
                 (unsigned) s.plan.done());
        chunk_submit_failed = true;
      }
    }

    // IN data endpoint, status COMPLETED, device delivered > 0 bytes (raw x->actual_num_bytes).
    // Same predicate as in_done_data, reused so that every data-bearing IN completion lands in
    // exactly one of in_ret_sent / in_send_failed / dropped_silent / unlink_discarded.
    const bool in_data_done =
        !s.is_control && s.dir_in && x->status == USB_TRANSFER_STATUS_COMPLETED && x->actual_num_bytes > 0;

    // Classify the raw USB completion itself, before deciding whether/how to notify the client
    // (data endpoints only, not ep0; the collateral-cancel-resubmit branch above already handled
    // its own completions via `continue` and never reaches here). For a chunked transfer this
    // reflects the *last* chunk's x->actual_num_bytes, not the total bytes moved so far
    // (s.plan.done()) -- counting the HW completion event as specified, not the client-visible
    // aggregate result.
    if (!s.is_control) {
      if (s.dir_in) {
        if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
          if (x->actual_num_bytes > 0) {
            this->in_done_data_.fetch_add(1, std::memory_order_relaxed);
          } else {
            this->in_done_empty_.fetch_add(1, std::memory_order_relaxed);
          }
        } else {
          this->in_done_err_.fetch_add(1, std::memory_order_relaxed);
        }
      } else if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
        this->out_done_ok_.fetch_add(1, std::memory_order_relaxed);
      }
    }

    const int32_t status = chunk_submit_failed ? usbip::kLinuxEPIPE : map_status_(x->status);
    uint32_t len;
    if (s.is_control) {
      len = usbip::control_data_len(x->actual_num_bytes, s.requested);
    } else if (s.chunked) {
      len = s.plan.done();
    } else {
      len = std::min<uint32_t>(x->actual_num_bytes < 0 ? 0 : x->actual_num_bytes, s.requested);
    }

    if (s.clear_halt_ep != 0 && x->status == USB_TRANSFER_STATUS_COMPLETED) {
      DevLockHold hold(this, LockSite::CLEAR);
      if (this->dev_ready_) {
        esp_err_t err = usb_host_endpoint_clear(this->device_handle_, s.clear_halt_ep);
        ESP_LOGD(TAG, "CLEAR_HALT ep %02X host side: %s", s.clear_halt_ep, esp_err_to_name(err));
        IntrInEp *iep = this->find_intr_in_(s.clear_halt_ep);
        if (iep != nullptr && err == ESP_OK) {
          // The pipe is active again: resume the prefetch (intr_service_ submits it).
          iep->stalled = false;
          iep->need_clear = false;
          ESP_LOGI(TAG, "Interrupt ep %02X: halt cleared, prefetch resumed", s.clear_halt_ep);
        }
      }
    }

    if (ok && !s.orphan && sock >= 0) {
      if (s.unlink_pending) {
        // Only RET_UNLINK goes out (no payload), so data the device already delivered is discarded.
        if (in_data_done) this->unlink_discarded_.fetch_add(1, std::memory_order_relaxed);
        ok = send_ret_unlink_(sock, s.unlink_seqnum, usbip::kLinuxECONNRESET);
      } else {
        const uint8_t *data = nullptr;
        if (s.dir_in) {
          if (s.chunked) {
            data = s.big_buf;
          } else {
            data = x->data_buffer + (s.is_control ? usbip::kSetupPacketSize : 0);
          }
        }
        ok = send_ret_submit_(sock, s.seqnum, s.start_frame, s.number_of_packets, status, len, data);
        if (in_data_done) {
          if (ok) {
            this->in_ret_sent_.fetch_add(1, std::memory_order_relaxed);
          } else {
            this->in_send_failed_.fetch_add(1, std::memory_order_relaxed);
          }
        }
      }
      // Covers both the RET_UNLINK and RET_SUBMIT sends above: any send_all_ failure on a
      // data-endpoint slot, regardless of whether it carried data.
      if (!s.is_control && !ok) this->send_errors_.fetch_add(1, std::memory_order_relaxed);
      ESP_LOGV(TAG, "seq %u: done, status %d, %u B", (unsigned) s.seqnum, (int) status, (unsigned) len);
    } else if (in_data_done) {
      // ok==false (an earlier send in this same drain batch already failed), or s.orphan (session
      // ended), or sock<0 (end_session_ drain): no RET_SUBMIT/RET_UNLINK is even attempted, so a
      // COMPLETED IN transfer that carried data is silently dropped here. Other drops (OUT, errors,
      // zero-length) are intentionally not counted.
      this->dropped_silent_.fetch_add(1, std::memory_order_relaxed);
    }
    this->free_slot_(s);
  }
  return ok;
}

void USBIPServer::end_session_() {
  this->client_addr_ = 0;
  // Interrupt IN: waiting requests have nobody to answer to; the prefetch keeps no session state
  // (completions are dropped and not resubmitted, see handle_prefetch_done_ / pump_prefetch_).
  this->intr_session_ = false;
  for (IntrInEp &ep : this->intr_in_) {
    if (ep.active) ep.q.clear_waiting();
  }
  uint8_t pending = 0;
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    if (this->slots_[i].used) {
      this->slots_[i].orphan = true;
      ++pending;
    }
  }
  if (pending == 0) return;

  // Bulk IN may wait for data forever - cancel pending transfers on bulk endpoints so the slots are
  // released (orphan -> no resubmission). Interrupt endpoints are never halted: interrupt OUT
  // finishes by itself (or stays an orphan), interrupt IN has no slots.
  uint8_t cancelled[MAX_PENDING_LIMIT];
  uint8_t n_cancelled = 0;
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    const PendingSlot &s = this->slots_[i];
    if (!s.used || s.is_control || s.is_intr) continue;
    bool seen = false;
    for (uint8_t j = 0; j < n_cancelled; ++j) seen = seen || cancelled[j] == s.ep_addr;
    if (!seen) {
      cancelled[n_cancelled++] = s.ep_addr;
      this->cancel_endpoint_(s.ep_addr);
    }
  }

  const uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SESSION_DRAIN_TIMEOUT_MS);
  while (xTaskGetTickCount() < deadline) {
    this->drain_completions_(-1);
    pending = 0;
    for (uint8_t i = 0; i < this->max_pending_; ++i) {
      if (this->slots_[i].used) ++pending;
    }
    if (pending == 0) return;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // Unfinished slots stay occupied (orphan) and are released when they complete.
  ESP_LOGW(TAG, "%u transfer(s) still pending after session end", pending);
}


// ---------------------------------------------------------------- interrupt IN without HALT
//
// ESP-IDF 5.5 can wait forever in usb_host_endpoint_halt on a periodic pipe (_pipe_cmd_halt waits for
// a HALT_REQ channel interrupt the DWC controller does not raise for periodic channels; esp-usb PR
// #577). Interrupt IN endpoints therefore keep up to PREFETCH_PER_EP server-owned transfers of one
// MPS each, store the packets in an IntrRing and answer the client's CMD_SUBMIT from it. CMD_UNLINK
// only removes a waiting request (usbip_intrq.h). All of this runs in the TCP task.

USBIPServer::IntrInEp *USBIPServer::find_intr_in_(uint8_t addr) {
  for (IntrInEp &ep : this->intr_in_) {
    if (ep.active && ep.addr == addr) return &ep;
  }
  return nullptr;
}

uint16_t USBIPServer::pf_in_flight_(const IntrInEp &ep) const {
  uint16_t n = 0;
  for (const PrefetchXfer *pf : ep.pf) {
    if (pf != nullptr && pf->in_flight) ++n;
  }
  return n;
}

bool USBIPServer::setup_intr_eps_() {
  usbip::EndpointInfo eps[usbip::kMaxEndpoints];
  size_t n = 0;
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
  const bool ready = this->dev_ready_;
  const uint32_t gen = this->dev_generation_.load(std::memory_order_relaxed);
  if (ready) {
    n = this->ep_count_;
    for (size_t i = 0; i < n; ++i) eps[i] = this->eps_[i];
  }
  xSemaphoreGive(this->dev_lock_);
  if (!ready) return false;
  if (this->intr_configured_ && gen == this->intr_generation_) return true;

  // Transfers of the previous device can only be freed after their completion was drained
  // (NO_DEVICE). Normally done by drain_completions_ before this call; otherwise retry later.
  for (const PrefetchXfer &pf : this->prefetch_) {
    if (pf.in_flight) {
      ESP_LOGW(TAG, "Prefetch transfer of the previous device still in flight, interrupt setup postponed");
      this->intr_configured_ = false;
      return false;
    }
  }
  for (PrefetchXfer &pf : this->prefetch_) {
    if (pf.xfer != nullptr) usb_host_transfer_free(pf.xfer);
    pf.xfer = nullptr;
    pf.ep = nullptr;
  }
  for (IntrInEp &ep : this->intr_in_) {
    ep.active = false;
    ep.stalled = false;
    ep.need_clear = false;
    ep.blocked_full = false;
    for (PrefetchXfer *&p : ep.pf) p = nullptr;
  }

  uint8_t used = 0;
  for (size_t i = 0; i < n; ++i) {
    const usbip::EndpointInfo &e = eps[i];
    if (e.type != usbip::kEpTypeInterrupt || (e.address & 0x80) == 0 || e.mps == 0) continue;
    if (used >= MAX_INTR_IN_EPS) {
      ESP_LOGW(TAG, "Interrupt IN ep %02X: more than %u interrupt IN endpoints, not served", e.address,
               MAX_INTR_IN_EPS);
      continue;
    }
    IntrInEp &ep = this->intr_in_[used];
    if (!ep.q.init(this->interrupt_queue_depth_, e.mps)) {
      ESP_LOGE(TAG, "Interrupt IN ep %02X: queue allocation failed, not served", e.address);
      continue;
    }
    ep.q.reset();
    bool alloc_ok = true;
    for (uint8_t k = 0; k < PREFETCH_PER_EP; ++k) {
      PrefetchXfer &pf = this->prefetch_[used * PREFETCH_PER_EP + k];
      usb_transfer_t *x = nullptr;
      if (usb_host_transfer_alloc(e.mps, 0, &x) != ESP_OK || x == nullptr) {
        alloc_ok = false;
        break;
      }
      x->bEndpointAddress = e.address;
      x->num_bytes = e.mps;
      x->flags = 0;
      x->callback = prefetch_cb_;
      x->context = &pf;
      pf.xfer = x;
      pf.ep = &ep;
      pf.in_flight = false;
      ep.pf[k] = &pf;
    }
    if (!alloc_ok) {
      for (PrefetchXfer *&p : ep.pf) {
        if (p != nullptr && p->xfer != nullptr) usb_host_transfer_free(p->xfer);
        if (p != nullptr) {
          p->xfer = nullptr;
          p->ep = nullptr;
        }
        p = nullptr;
      }
      ESP_LOGE(TAG, "Interrupt IN ep %02X: transfer allocation failed, not served", e.address);
      continue;
    }
    ep.addr = e.address;
    ep.mps = e.mps;
    ep.active = true;
    ++used;
    ESP_LOGI(TAG, "Interrupt IN ep %02X: prefetch without HALT (%u transfers, queue %u x %u B)", e.address,
             PREFETCH_PER_EP, ep.q.ring().depth(), e.mps);
  }
  this->intr_generation_ = gen;
  this->intr_configured_ = true;
  return true;
}

void USBIPServer::intr_on_import_() {
  this->intr_session_ = false;
  this->drain_completions_(-1);  // completions from before this import (counted as no-session drops)
  this->setup_intr_eps_();
  uint32_t flushed = 0;
  for (IntrInEp &ep : this->intr_in_) {
    if (!ep.active) continue;
    flushed += ep.q.ring().size();
    ep.q.reset();
    ep.blocked_full = false;
  }
  if (flushed > 0) {
    this->intr_flushed_import_.fetch_add(flushed, std::memory_order_relaxed);
    ESP_LOGD(TAG, "Import: %u stored interrupt packet(s) dropped", (unsigned) flushed);
  }
  this->intr_session_ = true;
}

bool USBIPServer::handle_intr_in_submit_(int sock, const usbip::Message &m, const usbip::EndpointInfo &e) {
  const usbip::CmdSubmit &c = m.submit;
  const uint32_t seq = c.base.seqnum;
  IntrInEp *ep = this->find_intr_in_(e.address);
  if (ep == nullptr) {
    ESP_LOGW(TAG, "seq %u: interrupt IN ep %02X has no prefetch -> -EPIPE", (unsigned) seq, e.address);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  if (ep->stalled) {
    // As on a real host: transfers to a halted endpoint fail until CLEAR_FEATURE(HALT).
    ESP_LOGV(TAG, "seq %u: interrupt ep %02X stalled -> -EPIPE", (unsigned) seq, e.address);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  const uint32_t now = now_ms_();
  if (!ep->q.add_request(seq, c.transfer_buffer_length, now, c.start_frame, c.number_of_packets)) {
    this->slot_exhausted_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(TAG, "seq %u: too many waiting interrupt IN requests -> -EPIPE", (unsigned) seq);
    return send_ret_submit_(sock, seq, c.start_frame, c.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
  }
  ESP_LOGV(TAG, "seq %u: INTR IN ep %02X len %u (queued %u)", (unsigned) seq, e.address,
           (unsigned) c.transfer_buffer_length, (unsigned) ep->q.ring().size());
  const bool ok = this->serve_intr_ep_(sock, *ep);  // from the ring at once, if it holds the data
  this->pump_prefetch_(*ep, now);                   // otherwise make sure the prefetch is running
  return ok;
}

bool USBIPServer::serve_intr_ep_(int sock, IntrInEp &ep) {
  bool ok = true;
  uint32_t seq = 0;
  usbip::IntrServePlan plan;
  while (ok && sock >= 0 && ep.q.front_ready(&seq, &plan)) {
    const usbip::IntrRequest r = ep.q.waiting().front();
    ep.q.complete_front(plan, this->intr_out_buf_);  // plan.length <= request length <= dma_chunk_size_
    int32_t status = 0;
    if (plan.overflow) {
      // The packet was longer than the request: a Linux HCD reports babble as -EOVERFLOW.
      status = usbip::kLinuxEOVERFLOW;
      this->intr_overflow_.fetch_add(1, std::memory_order_relaxed);
    }
    ok = send_ret_submit_(sock, seq, r.start_frame, r.number_of_packets, status, plan.length, this->intr_out_buf_);
    if (plan.length > 0) {
      if (ok) {
        this->in_ret_sent_.fetch_add(1, std::memory_order_relaxed);
      } else {
        this->in_send_failed_.fetch_add(1, std::memory_order_relaxed);
      }
    }
    if (!ok) this->send_errors_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGV(TAG, "seq %u: INTR IN ep %02X done, status %d, %u B (%u packet(s))", (unsigned) seq, ep.addr,
             (int) status, (unsigned) plan.length, plan.packets);
  }
  return ok;
}

bool USBIPServer::fail_waiting_(int sock, IntrInEp &ep, int32_t status) {
  bool ok = true;
  usbip::IntrRequest r;
  while (ep.q.pop_request(&r)) {
    if (ok && sock >= 0) {
      ok = send_ret_submit_(sock, r.seqnum, r.start_frame, r.number_of_packets, status, 0, nullptr);
      if (!ok) this->send_errors_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  return ok;
}

void USBIPServer::pump_prefetch_(IntrInEp &ep, uint32_t now) {
  if (!ep.active || !this->intr_session_ || ep.stalled) return;
  if (ep.need_clear) {
    // A transfer error halted the pipe (the stack flushed the other transfer). Clear it on the host
    // once every completion is drained; the device is not halted, so nothing goes to the device.
    if (this->pf_in_flight_(ep) != 0) return;
    esp_err_t err;
    {
      DevLockHold hold(this, LockSite::CLEAR);
      err = this->dev_ready_ ? usb_host_endpoint_clear(this->device_handle_, ep.addr) : ESP_ERR_INVALID_STATE;
    }
    ep.need_clear = false;
    if (err != ESP_OK) {
      // Not seen so far; stop the prefetch until the client clears the halt (CLEAR_FEATURE is forwarded).
      ep.stalled = true;
      ESP_LOGW(TAG, "Interrupt ep %02X: clear after transfer error failed: %s, prefetch stopped", ep.addr,
               esp_err_to_name(err));
      return;
    }
  }
  for (PrefetchXfer *pf : ep.pf) {
    if (pf == nullptr || pf->xfer == nullptr || pf->in_flight) continue;
    // Every transfer in flight may still bring a packet: count them as taken ring entries.
    if (!ep.q.should_prefetch(true, now, usbip::kDefaultPrefetchWindowMs, this->pf_in_flight_(ep))) break;
    usb_transfer_t *x = pf->xfer;
    x->num_bytes = ep.mps;
    x->flags = 0;
    pf->in_flight = true;
    this->prefetch_in_flight_.fetch_add(1, std::memory_order_relaxed);  // before the submit: the callback decrements
    const esp_err_t err = this->submit_locked_(x, false);
    if (err != ESP_OK) {
      pf->in_flight = false;
      this->prefetch_in_flight_.fetch_sub(1, std::memory_order_relaxed);
      ESP_LOGV(TAG, "Interrupt ep %02X: prefetch submit failed: %s", ep.addr, esp_err_to_name(err));
      break;
    }
    this->in_submitted_.fetch_add(1, std::memory_order_relaxed);
  }
  // intr_q_full: counted once each time the ring starts holding the prefetch back.
  const bool full = static_cast<uint32_t>(ep.q.ring().size()) + this->pf_in_flight_(ep) >= ep.q.ring().depth();
  const bool blocked = full && ep.q.request_active(now, usbip::kDefaultPrefetchWindowMs);
  if (blocked && !ep.blocked_full) this->intr_q_full_.fetch_add(1, std::memory_order_relaxed);
  ep.blocked_full = blocked;
}

bool USBIPServer::handle_prefetch_done_(int sock, uint8_t pf_index) {
  if (pf_index >= PREFETCH_COUNT) {
    this->stale_skip_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }
  PrefetchXfer &pf = this->prefetch_[pf_index];
  if (!pf.in_flight || pf.xfer == nullptr || pf.ep == nullptr) {
    this->stale_skip_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }
  pf.in_flight = false;
  IntrInEp &ep = *pf.ep;
  const usb_transfer_t *x = pf.xfer;
  bool ok = true;
  switch (x->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
      if (x->actual_num_bytes > 0) {
        this->in_done_data_.fetch_add(1, std::memory_order_relaxed);
        const uint16_t len = static_cast<uint16_t>(std::min<int>(x->actual_num_bytes, ep.mps));
        if (!this->intr_session_) {
          this->intr_dropped_nosession_.fetch_add(1, std::memory_order_relaxed);
        } else if (!ep.q.on_packet(x->data_buffer, len)) {
          // Should not happen: pump_prefetch_ reserves a ring entry for every transfer in flight.
          this->intr_q_full_.fetch_add(1, std::memory_order_relaxed);
          ESP_LOGW(TAG, "Interrupt ep %02X: ring full, packet dropped", ep.addr);
        }
      } else {
        // 0 B completion: not stored (not established that it is always a real zero-length packet).
        this->in_done_empty_.fetch_add(1, std::memory_order_relaxed);
        this->intr_zlp_dropped_.fetch_add(1, std::memory_order_relaxed);
      }
      break;
    case USB_TRANSFER_STATUS_STALL:
      // The device halted the endpoint; the stack halted and flushed the pipe. Waiting requests fail
      // with -EPIPE (the client then sends CLEAR_FEATURE(HALT), which is forwarded).
      this->in_done_err_.fetch_add(1, std::memory_order_relaxed);
      if (!ep.stalled) ESP_LOGW(TAG, "Interrupt ep %02X: STALL, prefetch paused until CLEAR_FEATURE(HALT)", ep.addr);
      ep.stalled = true;
      ok = this->fail_waiting_(sock, ep, usbip::kLinuxEPIPE);
      break;
    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:  // flushed together with a STALL/error of the other transfer
      this->in_done_err_.fetch_add(1, std::memory_order_relaxed);
      break;
    default:
      // ERROR / TIMED_OUT / OVERFLOW: the stack halted and flushed the pipe, the device did not.
      // The packet is lost; the pipe is cleared on the host and the prefetch continues.
      this->in_done_err_.fetch_add(1, std::memory_order_relaxed);
      if (!ep.stalled) ep.need_clear = true;
      ESP_LOGW(TAG, "Interrupt ep %02X: prefetch transfer status %d, clearing the pipe", ep.addr, (int) x->status);
      break;
  }
  if (ok && sock >= 0 && this->intr_session_) ok = this->serve_intr_ep_(sock, ep);
  this->pump_prefetch_(ep, now_ms_());
  return ok;
}

bool USBIPServer::intr_service_(int sock) {
  const uint32_t now = now_ms_();
  if (this->intr_session_ && !this->intr_configured_) this->setup_intr_eps_();  // postponed at import
  uint32_t peak = 0;
  for (IntrInEp &ep : this->intr_in_) {
    if (!ep.active) continue;
    this->pump_prefetch_(ep, now);  // also re-evaluates the 50 ms window
    peak = std::max<uint32_t>(peak, ep.q.ring().size());
  }
  uint32_t cur = this->intr_q_depth_max_.load(std::memory_order_relaxed);
  while (peak > cur && !this->intr_q_depth_max_.compare_exchange_weak(cur, peak, std::memory_order_relaxed)) {
  }

  // Interrupt OUT after CMD_UNLINK: RET_UNLINK at the latest OUT_UNLINK_WAIT_MS later. The slot stays
  // an orphan, so its later completion sends nothing (no RET_SUBMIT after RET_UNLINK).
  bool ok = true;
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    PendingSlot &s = this->slots_[i];
    if (!s.used || !s.is_intr || !s.unlink_pending || s.orphan) continue;
    if (static_cast<int32_t>(now - s.unlink_deadline_ms) < 0) continue;
    this->out_unlink_wait_timeout_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(TAG, "seq %u: interrupt OUT ep %02X not finished %u ms after UNLINK -> RET_UNLINK, orphaned",
             (unsigned) s.seqnum, s.ep_addr, (unsigned) OUT_UNLINK_WAIT_MS);
    if (ok && sock >= 0) {
      ok = send_ret_unlink_(sock, s.unlink_seqnum, usbip::kLinuxECONNRESET);
      if (!ok) this->send_errors_.fetch_add(1, std::memory_order_relaxed);
    }
    s.orphan = true;
  }
  return ok;
}

}  // namespace esphome::usbip_server
