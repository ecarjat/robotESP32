## Overview
- Provide a Wi-Fi passthrough mode so a Python client can send/receive Robot Protocol frames via the ESP32.
- Passthrough is a transport bridge only: the ESP32 does not interpret frames while active.
- Mode is selectable via controller, auto-enabled when no controller connects after a timeout, or forced at boot for debug.

## Goals
- Expose the existing Robot Protocol (COBS + CRC32 framing) over TCP without changing the frame format.
- Keep a single, reliable client connection to avoid conflicting control sources.
- Make entry/exit explicit and visible on the OLED.
- Ship an initial CLI with a terminal-style command interface for RPC GET/SET.

## Non-Goals
- No encryption or authentication beyond Wi-Fi security.
- No protocol translation (no JSON or REST layer).
- No multi-client arbitration.

## Configuration (compile-time / build flags)
- `RP_PASSTHROUGH_FORCE_BOOT` (0/1): force passthrough at boot for debugging.
- `RP_PASSTHROUGH_NO_CTRL_TIMEOUT_MS` (default 15000): auto-enter passthrough if no controller connects.
- `RP_PASSTHROUGH_PORT` (default 7777): TCP server port.
- `RP_WIFI_STA_TIMEOUT_MS` (default 10000): time to wait for STA connection before AP fallback.
- `RP_AP_SSID_PREFIX` (default "Robot2W-PT"): SSID prefix for SoftAP mode.
- `RP_AP_PASSWORD` (default "robot2wheel"): SoftAP password (change for non-lab use).
- `RP_CONFIG_PORT` (default 80): HTTP config portal port when in AP mode.
- `RP_PASSTHROUGH_IDLE_TIMEOUT_MS` (default 60000): close client if no data is exchanged.
- `RP_BRIDGE_RX_BUF_SIZE` / `RP_BRIDGE_TX_BUF_SIZE` (default 1024): socket <-> UART buffers.

## Mode Selection
### Manual (controller)
- Add a menu item: `Tools -> RP Passthrough`.
- Enter requires confirmation (e.g., "Hold A to confirm").
- Exit requires a separate action (e.g., "Hold B to exit") to avoid accidental drop.

### Auto (no controller)
- On boot, if no controller connects within `RP_PASSTHROUGH_NO_CTRL_TIMEOUT_MS`,
  enter passthrough automatically.
- If a controller connects while in passthrough, remain in passthrough until the user exits
  via the menu (no implicit takeover).

### Forced at boot
- If `RP_PASSTHROUGH_FORCE_BOOT=1`, bypass controller detection and enter passthrough
  immediately after Wi-Fi bring-up.

## Wi-Fi Behavior
- Wi-Fi is **only** started when passthrough becomes active; it is shut down on exit.
- Boot sequence:
  1) Try STA mode with saved credentials.
  2) If no credentials or no connection within `RP_WIFI_STA_TIMEOUT_MS`, start SoftAP.
- SoftAP SSID: `${RP_AP_SSID_PREFIX}-${last4(MAC)}`; password: `RP_AP_PASSWORD`.
- When in SoftAP mode, a config portal is available at `http://<ap-ip>:<RP_CONFIG_PORT>/`
  to set SSID/password; credentials are stored in NVS and the ESP32 attempts STA connect.
- OLED displays current mode + IP address (STA IP or AP IP).
- Optional: advertise mDNS `robot-esp32.local` with service `_robotproto._tcp`.

## Transport & Framing
- TCP server listens on `RP_PASSTHROUGH_PORT`.
- Byte stream is **raw Robot Protocol frames**:
  - Same COBS framing + `0x00` delimiters as UART.
  - Same CRC32 and header fields as `../common/shared_protocol/robot_protocol.h`.
- ESP32 acts as a transparent bridge:
  - Network -> UART: forward bytes as-is.
  - UART -> Network: forward bytes as-is.
  - No re-framing or message inspection.
- Single client at a time:
  - New connection closes the previous one.
  - If the client disconnects, ESP32 stays in passthrough and awaits the next client.

## Passthrough Runtime Behavior
- When entering:
  - Stop sending controller teleop and heartbeat frames.
  - Flush UART and socket buffers to start on frame boundaries.
  - Update OLED to "RP Passthrough" + connection status.
- While active:
  - The ESP32 does not originate CMD/TELEM/RPC/FILE messages.
  - UART traffic is exclusively from the Wi-Fi client.
- When exiting:
  - Close TCP client, reset bridge buffers.
  - Resume controller teleop/heartbeat behavior.

## CLI Tool (cli/)
### Scope (initial)
- Provide a terminal-style REPL over TCP for RPC GET/SET.
- Use the same Robot Protocol framing and RPC payloads as firmware.

### Terminal experience
- Launch: `python3 cli/rp_cli.py --host <ip|mdns>` opens a prompt (e.g., `rp>`).
- Commands (case-insensitive):
  - `GET <param>`: fetch a parameter by name and print its value.
  - `SET <param> <value>`: set a parameter by name.
  - `GET ALL`: dump all known parameters in name=value form.
  - `SAVE`: persist current parameters to flash.
  - Optional: `HELP`, `EXIT` for discoverability.

### Param name mapping
- CLI loads a local param map (name -> offset/type) that mirrors `robot_params_t`.
- Source of truth is `tools/param_map.def` + `tools/gen_params_json.py`, which generates
  `cli/params.json` from `firmware/Drivers/param_storage.h`. The CLI must not require
  on-device discovery.

### RPC payload layout (from robot_protocol.h)
```c
typedef struct __attribute__((packed)) {
  uint8_t  method;  // ROBOT_RPC_METHOD_GET_PARAM / SET_PARAM
  uint8_t  flags;   // request: ROBOT_RPC_FLAG_SAVE
  uint16_t offset;  // byte offset into robot_params_t
  uint16_t length;  // bytes of data for SET/GET
} robot_rpc_param_t;
```

### CLI behavior
- Each request uses a new `seq` and sets `ACK_REQ`.
- `GET <param>`:
  - Send `RPC_REQ` with method `GET_PARAM`.
  - Print typed value plus raw hex bytes.
- `SET <param> <value>`:
  - Encode value based on param type and send `SET_PARAM` with `flags=0`.
  - Print status from the `RPC_RESP` flags.
- `GET ALL`:
  - Iterate through the param map and issue `GET` for each entry.
- `SAVE`:
  - Send a `SET_PARAM` request with `flags=ROBOT_RPC_FLAG_SAVE` and `length=0`.
  - Firmware interprets this as "persist current params".
- Retries: 3 attempts with 50 ms backoff for missing ACK or response.

## Diagnostics
- Serial log lines:
  - Wi-Fi mode (STA/AP), IP, client connect/disconnect.
  - Passthrough enter/exit and current client IP.
  - Bridge buffer overflow counts (if any).

## Open Items
- Confirm controller input chord or menu path for mode toggling.
- Confirm desired defaults for timeouts and port.
