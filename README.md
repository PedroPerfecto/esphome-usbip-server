# esphome-usbip-server

A [USB/IP](https://docs.kernel.org/usb/usbip_protocol.html) server as an
ESPHome external component. It exports one USB device plugged into the native
USB OTG port of an ESP32-S2/S3 (or other `usb_host`-capable ESP32) over the
network. A Linux client, for example Home Assistant with a USB/IP client
add-on, attaches it with the standard `usbip` tools and uses it as if it were
plugged in locally.

Typical use: a USB device that must stay in one place (an alarm panel, a
meter, a PLC) while the server that uses it runs elsewhere, with the device
managed, updated over the air and monitored like any other ESPHome node.

## Status

Working and in daily use with one device. Tested with:

- **Board:** ESP32-S3-DevKitC-1 (N8R8, 8 MB flash, 8 MB octal PSRAM)
- **ESPHome:** 2026.9.0
- **Device:** Jablotron JA-100 Flexi alarm panel (composite HID + Mass Storage,
  full speed), used by the
  [jablotron100](https://github.com/kukulich/home-assistant-jablotron100)
  Home Assistant integration
- **Clients:** Raspberry Pi OS (kernel 6.18) with `usbip`; Home Assistant OS
  18.3 (kernel 6.18) with the HA USB/IP Client add-on

Other boards, devices and clients should work but have not been tested.

## Features

- USB/IP protocol version 1.1.1 as implemented by Linux `usbip-host`/`vhci-hcd`:
  device list, import, control, interrupt and bulk transfers, unlink
- Large bulk transfers (Linux `usb-storage` requests up to ~120 KB) split into
  DMA-sized chunks
- Optional export of selected interfaces only (`interfaces:`), e.g. only the HID
  interface of a composite device, so the client never binds drivers to the rest
- A new import takes over a stale session (client rebooted, network dropped);
  TCP keepalive detects a dead client within ~25 s
- Clean disconnect before planned reboots (OTA, restart from Home Assistant),
  so the client releases its virtual port immediately
- Optional diagnostic entities: USB device connected, client connected, client
  address, number of client connections since boot

## Requirements and hardware notes

- An ESP32 variant supported by ESPHome `usb_host` (ESP32-S2, S3, S31, P4, H4).
  Only one USB device can be connected (an `usb_host` limitation).
- **The device needs 5 V on VBUS.** The native USB port of ESP32-S3-DevKitC-1
  does not supply it. Use an OTG Y-cable with a power input, or otherwise feed
  5 V to the device. Without VBUS, enumeration fails.
- On the S2/S3 the OTG port and USB-Serial-JTAG share one PHY. Boards with a
  separate USB-UART bridge (like the DevKitC "UART" port) can keep serial
  logging with `logger: hardware_uart: UART0`.
- PSRAM is recommended with the default `max_transfer_size` (128 KB). Without
  PSRAM the buffers fall back to internal RAM; lower `max_transfer_size` then
  (a HID-only device needs only a few KB). Not tested without PSRAM.

## Security

**USB/IP has no authentication and no encryption.** Anyone who can reach the TCP
port can list and attach the device. Keep the node on a trusted network
segment and restrict port 3240 to the client with a firewall rule.

## Installation

```yaml
external_components:
  - source: github://PedroPerfecto/esphome-usbip-server@main
    components: [usbip_server]

usbip_server:
```

See [examples/](examples/) for complete configurations. Pin a tag or commit
instead of `@main` for a node you depend on.

## Configuration

| Option | Default | Range | Description |
|---|---|---|---|
| `port` | `3240` | 1-65535 | TCP port of the USB/IP server |
| `busid` | `"1-1"` | 1-31 characters | Bus ID reported to clients |
| `vid` / `pid` | `0` / `0` | `0x0000`-`0xFFFF` | Accept only the device with exactly this VID and PID; both `0` = any device |
| `max_transfer_size` | `131072` | 64-1048576 | Largest single transfer accepted from the client, in bytes; larger requests fail with `-EPIPE` |
| `dma_chunk_size` | `16384` | 64-65536 | Largest single USB transfer, in bytes. Bulk transfers above it are split into chunks of this size (rounded down to a multiple of the endpoint's max packet size); interrupt transfers and control requests (`wLength`) above it fail with `-EPIPE` |
| `max_pending` | `16` | 1-32 | Maximum transfers in flight at once |
| `interfaces` | all | 0-31, at least one | List of `bInterfaceNumber` values to export |

### Exporting selected interfaces

```yaml
usbip_server:
  interfaces: [0]
```

The client receives a configuration descriptor containing only the listed
interfaces (with their class-specific and endpoint descriptors), requests to
other endpoints are rejected, and only the listed interfaces are claimed.
Linux accepts non-contiguous interface numbers but logs a notice about them.

### Diagnostic entities

```yaml
binary_sensor:
  - platform: usbip_server
    usb_connected:
      name: "USB device connected"
    client_connected:
      name: "USBIP client connected"

text_sensor:
  - platform: usbip_server
    client_address:
      name: "USBIP client"

sensor:
  - platform: usbip_server
    import_count:
      name: "USBIP connections"
```

All entities are optional and in the diagnostic category. `import_count` counts
successful imports since boot (`state_class: total_increasing`).

## Using the device on a Linux client

```sh
sudo modprobe vhci-hcd
usbip list -r <esp32-ip>
sudo usbip attach -r <esp32-ip> -b 1-1
usbip port                     # shows the attached device
sudo usbip detach -p <port>
```

In Home Assistant OS use a USB/IP client add-on and point it at the node's
address and bus ID.

## Logging

Per-transfer messages are logged at `VERBOSE` and are hidden at the default
`DEBUG` level. To trace transfers:

```yaml
logger:
  logs:
    usbip_server: VERBOSE
```

This produces hundreds of lines per second under load.

## Implementation notes

- The protocol layers (message encoding, stream parsing, descriptor handling,
  control request classification, chunking, interface filtering) are plain C++
  without ESPHome dependencies and are unit tested on the host:
  `make -C tests test`.
- ESP-IDF cannot cancel a single transfer. An unlink halts, flushes and clears
  the whole endpoint; other transfers cancelled as a side effect are submitted
  again. Control transfers on endpoint 0 cannot be cancelled and are completed.
- A zero-length packet is added only when the client sets `URB_ZERO_PACKET`,
  matching Linux behaviour.
- `SET_CONFIGURATION` for the active configuration is answered locally, as the
  Linux USB/IP host driver does.

## License

MIT, see [LICENSE](LICENSE). ESPHome itself is licensed under MIT (Python) and
GPLv3 (C++ runtime).
