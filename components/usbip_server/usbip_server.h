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
#include "usbip_stream.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <atomic>
#include <string>

namespace esphome::usbip_server {

static constexpr uint8_t MAX_PENDING_LIMIT = 32;

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
#endif
#ifdef USE_SENSOR
  SUB_SENSOR(import_count)
#endif

  void set_port(uint16_t port) { this->port_ = port; }
  void set_busid(const std::string &busid) { this->busid_ = busid; }
  void set_max_transfer_size(uint32_t size) { this->max_transfer_size_ = size; }
  void set_max_pending(uint8_t count) { this->max_pending_ = count; }
  void set_dma_chunk_size(uint32_t size) { this->dma_chunk_size_ = size; }
  void set_interface_mask(uint32_t mask) { this->interface_mask_ = mask; }

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

  bool get_device_info_(usbip::DeviceInfo &out);
  bool get_endpoint_(uint8_t address, usbip::EndpointInfo &out);
  bool device_ready_();
  void note_import_(int sock);   // TCP task: successful import (client + counter)
  void publish_entities_();      // main loop: state to entities, only on change
  void release_interfaces_();

  uint16_t port_{3240};
  std::string busid_{"1-1"};
  uint32_t max_transfer_size_{131072};
  uint8_t max_pending_{16};
  uint32_t dma_chunk_size_{16384};
  uint32_t interface_mask_{usbip::kAllInterfaces};

  SemaphoreHandle_t dev_lock_{nullptr};
  usbip::DeviceInfo dev_info_{};  // guarded by dev_lock_
  bool dev_ready_{false};         // guarded by dev_lock_
  usbip::EndpointInfo eps_[usbip::kMaxEndpoints];  // guarded by dev_lock_
  size_t ep_count_{0};                              // guarded by dev_lock_
  uint32_t claimed_mask_{0};                        // claimed interfaces (bit = number)
  // Filtered config descriptor that the server sends to the client itself;
  // length 0 = no filter, GET_DESCRIPTOR(CONFIG) is forwarded to the device.
  static constexpr size_t MAX_CONFIG_DESC = 512;
  uint8_t cfg_filtered_[MAX_CONFIG_DESC];  // guarded by dev_lock_
  uint16_t cfg_filtered_len_{0};           // guarded by dev_lock_

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
  // Last published values (main loop only).
  int8_t pub_usb_{-1};
  int8_t pub_client_{-1};
  uint32_t pub_addr_{0xFFFFFFFFu};
  int64_t pub_imports_{-1};
  char takeover_busid_[usbip::kBusIdSize]{};  // busid from the taken-over import (TCP task only)
};

}  // namespace esphome::usbip_server
