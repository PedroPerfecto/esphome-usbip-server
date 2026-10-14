# Changelog

## v0.2.0 – 2026-10-14

### Fixed
- **Interrupt endpoints no longer use HALT.** `usb_host_endpoint_halt` on a periodic pipe can
  wait forever in ESP-IDF 5.5 (esp-usb PR #577), which hung the TCP task and the device restarted.
  Interrupt IN is now served from server-owned prefetch transfers into a per-endpoint queue
  (`interrupt_queue_depth`, PSRAM); `CMD_UNLINK` only drops the waiting request. Interrupt OUT is
  never cancelled; its unlink is answered when the transfer completes, at the latest after 5 s.
- Control OUT with `wLength` not matching `transfer_buffer_length` or the payload is refused with
  `-EPIPE` instead of sending uninitialised or truncated data; control IN replies (also the
  filtered configuration descriptor) are limited to `min(data, wLength, transfer_buffer_length)`.
- An `OP_REQ_IMPORT` on a second connection takes the session over only if it would succeed
  (exact busid, device present); otherwise it is refused and the running session continues.
- A second connection must deliver its whole request within 2 s (was up to ~40 s byte by byte).
- Bulk transfers queued behind a chunked transfer on the same endpoint wait in a per-endpoint FIFO
  instead of being sent to the device between its chunks; unlinking a waiting one needs no USB
  operation.
- Every unlink is answered within 5 s (control and bulk too); the slot then becomes an orphan.
- Bulk IN that brings more than requested reports `-EOVERFLOW`; zero-length interrupt IN packets
  end a request like a short packet.
- `send()` on the session socket is limited to 5 s; the main loop no longer waits for the device
  lock without a limit (3 s, then a recorded reboot).
- Race in the completion callback (slot read after it was handed back), wrap-safe time
  arithmetic everywhere (32-bit clocks, readings from different tasks).

### Added
- **Stuck transfer detector** (`out_stuck_timeout`, default 3 s): an interrupt OUT in flight that
  long, or an unlinked/abandoned transfer still in flight, is recorded in RTC memory (`OUT_STUCK`,
  with an ep0 probe result) and the device reboots. **Reboot guard:** after 3 such reboots within
  an hour of run time it only reports (`OUT_STUCK_SUPPRESSED`). Soft stage: an interrupt OUT in
  flight for more than 3 s is logged and counted (`stall_soft`, `stall_recovered`, `stall_max_ms`).
- `dev_lock_` watchdog and a record of the previous boot (`lock_diag` text sensor:
  `WATCHDOG_REBOOT`, `INTR_SETUP_STUCK`, `CONNECT_WAIT`/`DISCONNECT_WAIT`, `OUT_STUCK`, ...).
- Options `interrupt_queue_depth`, `out_stuck_timeout`, `prefetch_transfers`.
- Diagnostic sensors: transfer, unlink, interrupt queue and detector counters
  (`out_stuck_detected`, `unlink_orphaned`, `send_timeouts`, `stall_*`, `intr_zlp`, ...), see
  `components/usbip_server/sensor.py`.

### Known limitations
- Bulk unlink is not exact (collateral cancel and resubmit, possible reordering); bulk and Mass
  Storage are experimental.
- A silent stop of the periodic transfers is only detected through interrupt OUT.
- A second connection can delay the running session by up to 2 s.
- Interrupt IN packets received between sessions are dropped.
- At most four interrupt IN endpoints are served.

See README, section "Known limitations", for details.

## v0.1.0
First release.
