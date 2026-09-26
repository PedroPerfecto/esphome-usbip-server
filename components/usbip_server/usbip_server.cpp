#include "usbip_server.h"
#include "usbip_control.h"
#include "usbip_urb.h"
#include "esphome/components/network/util.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <esp_heap_caps.h>
#include <lwip/sockets.h>

namespace esphome::usbip_server {

static const char *const TAG = "usbip_server";
static constexpr uint32_t TCP_TASK_STACK_SIZE = 6144;
static constexpr UBaseType_t TCP_TASK_PRIORITY = 4;  // below the USB task (5), above the main loop
static constexpr uint32_t SELECT_TIMEOUT_US = 2000;
static constexpr uint32_t SESSION_DRAIN_TIMEOUT_MS = 2000;

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

// ---------------------------------------------------------------- ESPHome

void USBIPServer::setup() {
  if (this->max_pending_ > MAX_PENDING_LIMIT) this->max_pending_ = MAX_PENDING_LIMIT;
  this->dev_lock_ = xSemaphoreCreateMutex();
  this->done_queue_ = xQueueCreate(this->max_pending_, sizeof(uint8_t));
  this->rx_payload_ = alloc_big_(this->max_transfer_size_);
  if (this->dev_lock_ == nullptr || this->done_queue_ == nullptr || this->rx_payload_ == nullptr) {
    ESP_LOGE(TAG, "Allocation failed");
    this->mark_failed();
    return;
  }
  for (uint8_t i = 0; i < MAX_PENDING_LIMIT; ++i) {
    this->slots_[i].owner = this;
    this->slots_[i].index = i;
  }
  usb_host::USBClient::setup();
  // Entities may only be changed from the main loop; loop() may be disabled
  // (disable_loop), the scheduler runs regardless.
  this->set_interval("entities", 1000, [this]() { this->publish_entities_(); });
}

void USBIPServer::publish_entities_() {
  const bool usb = this->device_ready_();
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
                "  Max pending: %u",
                this->port_, this->busid_.c_str(), (unsigned) this->max_transfer_size_,
                (unsigned) this->dma_chunk_size_, this->max_pending_);
  ESP_LOGCONFIG(TAG, "  TCP listening: %s", this->tcp_listening_.load() ? "YES" : "NO");
  usbip::DeviceInfo d;
  if (this->dev_lock_ != nullptr && this->get_device_info_(d)) {
    ESP_LOGCONFIG(TAG, "  Exported device: %04X:%04X (%u interfaces)", d.idVendor, d.idProduct, d.bNumInterfaces);
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

  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
  if (filtered_len > 0) memcpy(this->cfg_filtered_, filtered, filtered_len);
  this->cfg_filtered_len_ = static_cast<uint16_t>(filtered_len);
  this->dev_info_ = d;
  for (size_t i = 0; i < ep_count; ++i) this->eps_[i] = eps[i];
  this->ep_count_ = ep_count;
  this->dev_ready_ = true;
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
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
  this->dev_ready_ = false;
  xSemaphoreGive(this->dev_lock_);
  this->release_interfaces_();
  ESP_LOGW(TAG, "USB device gone, nothing to export");
  usb_host::USBClient::on_disconnected();
}

bool USBIPServer::get_device_info_(usbip::DeviceInfo &out) {
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
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
  xQueueSend(slot->owner->done_queue_, &slot->index, 0);  // the queue has room for every slot
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
  esp_err_t err;
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
  if (!this->dev_ready_) {
    err = ESP_ERR_INVALID_STATE;
  } else {
    x->device_handle = this->device_handle_;
    err = is_control ? usb_host_transfer_submit_control(this->handle_, x) : usb_host_transfer_submit(x);
  }
  xSemaphoreGive(this->dev_lock_);
  return err;
}

// Cancels all transfers on the endpoint (ESP-IDF cannot cancel a single transfer).
// Sequence halt -> flush -> clear follows usbipdcpp_esp32 (cancel_endpoint_all_transfers).
void USBIPServer::cancel_endpoint_(uint8_t ep_addr) {
  xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
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
  xSemaphoreGive(this->dev_lock_);
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
  const bool zero_packet = (c.transfer_flags & usbip::kUrbZeroPacket) != 0;
  usbip::ChunkPlan plan(requested, this->dma_chunk_size_, e.mps, dir_in, zero_packet);

  PendingSlot *slot = this->alloc_slot_();
  if (slot == nullptr) {
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
    case usbip::ControlKind::kClearHalt:
      clear_halt_ep = a.endpoint;  // forward to the device and on success reset the pipe on the host
      break;
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
  const uint32_t target = m.unlink.unlink_seqnum;
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    PendingSlot &s = this->slots_[i];
    if (s.used && !s.orphan && s.seqnum == target) {
      s.unlink_pending = true;
      s.unlink_seqnum = m.unlink.base.seqnum;
      if (s.is_control) {
        // A control transfer on ep0 cannot be cancelled in ESP-IDF (usbh_ep_get_handle does not
        // know ep0). Wait for completion and send RET_UNLINK instead of RET_SUBMIT.
        ESP_LOGV(TAG, "UNLINK seq %u (ep0) -> reply on completion", (unsigned) target);
      } else {
        // Cancels all transfers on the endpoint; those without unlink_pending are resubmitted.
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
    PendingSlot &s = this->slots_[idx];
    usb_transfer_t *x = s.xfer;
    if (!s.used || x == nullptr) continue;

    // Transfer cancelled only because of an UNLINK of another transfer on the same endpoint: resubmit.
    if (x->status == USB_TRANSFER_STATUS_CANCELED && !s.is_control && !s.unlink_pending && !s.orphan) {
      x->status = USB_TRANSFER_STATUS_COMPLETED;
      if (this->submit_locked_(x, false) == ESP_OK) {
        ESP_LOGV(TAG, "seq %u: collateral cancel on ep %02X -> resubmitted", (unsigned) s.seqnum, s.ep_addr);
        continue;
      }
      ESP_LOGW(TAG, "seq %u: resubmit on ep %02X failed -> -EPIPE", (unsigned) s.seqnum, s.ep_addr);
      if (ok && sock >= 0) {
        ok = send_ret_submit_(sock, s.seqnum, s.start_frame, s.number_of_packets, usbip::kLinuxEPIPE, 0, nullptr);
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
      xSemaphoreTake(this->dev_lock_, portMAX_DELAY);
      if (this->dev_ready_) {
        esp_err_t err = usb_host_endpoint_clear(this->device_handle_, s.clear_halt_ep);
        ESP_LOGD(TAG, "CLEAR_HALT ep %02X host side: %s", s.clear_halt_ep, esp_err_to_name(err));
      }
      xSemaphoreGive(this->dev_lock_);
    }

    if (ok && !s.orphan && sock >= 0) {
      if (s.unlink_pending) {
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
      }
      ESP_LOGV(TAG, "seq %u: done, status %d, %u B", (unsigned) s.seqnum, (int) status, (unsigned) len);
    }
    this->free_slot_(s);
  }
  return ok;
}

void USBIPServer::end_session_() {
  this->client_addr_ = 0;
  uint8_t pending = 0;
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    if (this->slots_[i].used) {
      this->slots_[i].orphan = true;
      ++pending;
    }
  }
  if (pending == 0) return;

  // Interrupt IN waits for data forever - cancel pending transfers on all
  // non-control endpoints so the slots are released (orphan -> no resubmission).
  uint8_t cancelled[MAX_PENDING_LIMIT];
  uint8_t n_cancelled = 0;
  for (uint8_t i = 0; i < this->max_pending_; ++i) {
    const PendingSlot &s = this->slots_[i];
    if (!s.used || s.is_control) continue;
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

}  // namespace esphome::usbip_server
