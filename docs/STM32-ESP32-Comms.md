## Overview
- STM32H723 runs the core robot stack; ESP32 acts as wireless/gamepad gateway.
- Link is UART-based with COBS framing and CRC32 integrity; channels multiplex CMD/TELEM/FILE/RPC.
- Folder split: `common/` (shared protocol code), `stm32/` (robot brain), `esp32/` (gateway).

## Architecture Split
1) **common/** (platform-agnostic)
   - framing + CRC; packet schemas; parsing/encoding helpers
   - shared state machines (file transfer, RPC, telemetry schema)
2) **stm32/** (robot brain)
   - UART DMA drivers (all links), I2C IMU/mag, LiDAR (TFmini)
   - logging (SD), UI (TFT), bare-metal scheduler (control tick + coop tasks)
3) **esp32/** (gateway/coproc)
   - Xbox gamepad acquisition
   - Wi-Fi portal HTTP server
   - UART transport to STM32 + routing CMD/TELEM/FILE/RPC

## ESP32 Firmware Responsibilities
- Bluetooth Xbox controller mapping → `CMD` messages:
  - forward/back velocity, turn/yaw
  - mode buttons (arm/disarm, mode cycle), e-stop/disarm
- Send CMD at fixed rate (e.g., 100 Hz) plus heartbeat.
- **Passthrough mode**: ESP32 can be commanded to bypass routing and expose a transparent UART bridge (e.g., for STM32 flashing/debug). Enter/exit via a dedicated RPC or GPIO-safe-boot latch; all protocol parsing is paused while passthrough is active.

## Telemetry Frame (TELEM_FRAME proposal)
- Target rate: 50 Hz (adjustable).
- Packed, little-endian, IEEE754 floats.
```c
typedef struct __attribute__((packed)) {
  uint8_t  version;      // 1
  uint8_t  status;       // bit0 armed, bit1 estop, bit2 fault, bit3 link_ok
  uint16_t faults;       // bitfield of active faults
  uint32_t timestamp_ms; // system time
  float    pose_x_m;
  float    pose_y_m;
  float    yaw_rad;
  float    vx_mps;
  float    vy_mps;       // optional, set 0 if not used
  float    wz_radps;
  float    ax_mps2;
  float    ay_mps2;
  float    az_mps2;
  float    batt_v;
  float    batt_a;
  float    batt_pct;     // 0..100
  float    temp_c;       // MCU or board temp
} telem_frame_v1_t;
```
- Size: 58 bytes payload; leaves ample room within `MAX_PAYLOAD=240`.
- Reserve `telem_frame_v2` for future fields (GPS, per-motor currents) while staying under the payload cap.

## STM32 ↔ ESP32 Link (Agreed)
### Physical link
- UART @ 921600 baud, full duplex; RTS/CTS **enabled** on both ends.
- STM32 pins (USART2 spine → ESP32): PA2 = TX, PA3 = RX, PA1 = RTS, PA0 = CTS (per `Pinmap.md`). Keep this mapping authoritative.

### Framing + integrity
- COBS framing with 0x00 delimiter; CRC32 over channel/type/payload.
- Channels:
  - **CMD** (teleop, mode changes)
  - **TELEM** (telemetry stream)
  - **FILE** (log listing + chunk download)
  - **RPC** (calibration, arm/disarm, config, etc.)

### Link behavior
- STM32 RX non-blocking: UART DMA circular + IDLE interrupt; parsing in main loop.
- Sequence numbers on CMD/FILE/RPC.
- Heartbeat: ESP32 sends `CMD_HEARTBEAT` ~10 Hz; STM32 marks link stale if no heartbeat > ~250 ms.

## Protocol Spec (STM32↔ESP32)
### Frame format (proposed)
- Outer: COBS framing with 0x00 delimiter.
- Chosen limits/params (fit in ≤256 bytes on the wire):
  - `HEADER_SIZE = 10` bytes
  - `CRC_SIZE = 4` bytes
  - `MAX_PAYLOAD = 240` bytes (decoded)
  - `MAX_DECODED = 254` bytes (`HEADER + PAYLOAD + CRC`)
  - `MAX_ENCODED = 256` bytes max (COBS overhead + delimiter)
  - CRC32: poly 0x04C11DB7, init 0xFFFFFFFF, refIn/refOut true, xorOut 0xFFFFFFFF
- Payload layout (decoded):
  - `u8  channel`   (CMD=1, TELEM=2, FILE=3, RPC=4)
  - `u8  msgType`
  - `u16 payloadLen`
  - `u32 seq`
  - `payload[payloadLen]`
  - `u32 crc32` (over channel..payload)

### Required message types
- **CMD**
  - `CMD_TELEOP` (forward, turn, flags)
  - `CMD_MODE` (requested mode)
  - `CMD_ARM` / `CMD_DISARM`
  - `CMD_HEARTBEAT`
- **TELEM**
  - `TELEM_FRAME` (fixed layout, versioned)
  - `TELEM_EVENT` (faults, transitions)
- **FILE**
  - `FILE_LIST_REQ` / `FILE_LIST_RESP`
  - `FILE_READ_REQ` (name, offset, length)
  - `FILE_READ_RESP` (offset, data)
  - `FILE_ERR`
- **RPC**
  - `RPC_REQ` / `RPC_RESP`
  - Methods: `CAL_IMU`, `ZERO_ESTIMATOR`, `SET_PARAM` (optional), `GET_PARAM` (optional), `GET_STATUS`

### RPC payloads (SET_PARAM / GET_PARAM)

`RPC_REQ`/`RPC_RESP` payload begins with:

```c
typedef struct __attribute__((packed)) {
  uint8_t  method;  // ROBOT_RPC_METHOD_*
  uint8_t  flags;   // request: ROBOT_RPC_FLAG_* / response: ROBOT_RPC_STATUS_*
  uint16_t offset;  // byte offset into robot_params_t
  uint16_t length;  // bytes of data for SET/GET
} robot_rpc_param_t;
```

- **GET_PARAM** request:
  - payload: `robot_rpc_param_t` only
  - flags = 0
  - response: `robot_rpc_param_t` + `length` bytes
- **SET_PARAM** request:
  - payload: `robot_rpc_param_t` + `length` bytes
  - flags bit0 = persist (save to flash)
  - response: `robot_rpc_param_t` only (status in flags)

Status codes (response `flags`):
- 0x00 OK
- 0x01 BAD_LEN
- 0x02 BAD_OFFSET
- 0x03 STORAGE_ERR
- 0x04 BAD_METHOD

Note: `robot_params_t` is larger than a single frame, so hosts must chunk
requests by `offset`/`length` (max payload is 240 bytes).

### RPC payloads (SET_PARAM / GET_PARAM)

`RPC_REQ`/`RPC_RESP` payload begins with:

```c
typedef struct __attribute__((packed)) {
  uint8_t  method;  // ROBOT_RPC_METHOD_*
  uint8_t  flags;   // request: ROBOT_RPC_FLAG_* / response: ROBOT_RPC_STATUS_*
  uint16_t offset;  // byte offset into robot_params_t
  uint16_t length;  // bytes of data for SET/GET
} robot_rpc_param_t;
```

- **GET_PARAM** request:
  - payload: `robot_rpc_param_t` only
  - flags = 0
  - response: `robot_rpc_param_t` + `length` bytes
- **SET_PARAM** request:
  - payload: `robot_rpc_param_t` + `length` bytes
  - flags bit0 = persist (save to flash)
  - response: `robot_rpc_param_t` only (status in flags)

Status codes (response `flags`):
- 0x00 OK
- 0x01 BAD_LEN
- 0x02 BAD_OFFSET
- 0x03 STORAGE_ERR
- 0x04 BAD_METHOD

Note: `robot_params_t` is larger than a single frame, so hosts must chunk
requests by `offset`/`length` (max payload is 240 bytes).

### Reliability / ACK Policy
- CRC32 on every packet; drop on CRC fail.
- **No ACK**: CMD, TELEM (fire-and-forget; TELEM can be rate-limited).
- **ACK required**: FILE and RPC messages set `flags.ACK_REQ`. Receiver replies with `IS_ACK` echoing `seq`.
- Retry guidance: resend FILE/RPC if no ACK within 50 ms; max 3 retries. Duplicates must be ACKed and dropped idempotently.

### Example C Structs (packed, little-endian)
```c
typedef struct __attribute__((packed)) {
  uint16_t magic;    // 0x4B56 ("VK")
  uint8_t  version;  // 0x01
  uint8_t  type;     // channel-specific msgType
  uint16_t seq;      // wraps at 65535
  uint16_t len;      // payload length
  uint16_t flags;    // bit0 ACK_REQ, bit1 IS_ACK
} link_hdr_t;

typedef struct __attribute__((packed)) {
  link_hdr_t hdr;
  uint8_t    payload[MAX_PAYLOAD]; // only hdr.len bytes are valid
  uint32_t   crc32;                // over hdr + payload[0..len-1]
} link_frame_t;

typedef struct __attribute__((packed)) {
  float vx_mps;
  float wz_radps;
  uint8_t flags;   // bit0 arm, bit1 estop, bit2 mode_cycle, etc.
} cmd_teleop_t;
```

# UART COBS Framing Protocol Specification

## Overview

This document specifies a binary framing protocol for full-duplex UART communication between an ESP32 and an STM32H723 using:

- UART at 921600 baud with RTS/CTS flow control
- Frame delimiting with 0x00 and payload transport via COBS
- CRC32 for integrity
- Optional ACK semantics for lightweight reliability

The spec is designed for deterministic, low-allocation implementations suitable for interrupt/DMA-based receivers.

## Normative Terms

The key words MUST, MUST NOT, REQUIRED, SHOULD, SHOULD NOT, and MAY are to be interpreted as described in RFC 2119.

## Link Layer

### Physical and UART Parameters

- Medium: UART, full-duplex
- Baud: 921600
- Flow control: RTS/CTS enabled on both ends
- Parity: None
- Data bits: 8
- Stop bits: 1

### Endianness

All multi-byte integer fields in the decoded packet format MUST be little-endian.

## Framing

### Frame Delimiter

- Each frame on the wire MUST end with a single delimiter byte 0x00.
- The delimiter 0x00 MUST NOT appear within the encoded portion of a frame.
- Receivers MUST treat 0x00 as the sole end-of-frame marker and MUST resynchronize by scanning for the next 0x00 after any error.

Wire representation:

- ENCODED_FRAME := COBS_ENCODE( DECODED_PACKET ) + 0x00

### COBS Variant

- The protocol MUST use standard COBS (Consistent Overhead Byte Stuffing).
- Encoders/decoders MUST implement the canonical COBS block format where each code byte describes a run of non-zero bytes (maximum run length 254 non-zero bytes per code).
- The encoded output MUST contain no 0x00 bytes.

### Size Limits

Implementations MUST define and share the same size constants:

- MAX_PAYLOAD: maximum payload bytes per packet
- HEADER_SIZE: 10 bytes
- CRC_SIZE: 4 bytes
- MAX_DECODED := HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE
- MAX_ENCODED := MAX_DECODED + CEIL(MAX_DECODED / 254) + 1

Notes:

- The +1 accounts for the mandatory 0x00 delimiter byte on the wire.
- Receivers MUST drop any candidate frame whose encoded length exceeds MAX_ENCODED before attempting COBS decode.

## Packet Format

### Decoded Packet Layout

The decoded packet is the byte sequence produced after COBS decoding and before delimiter handling.

Structure:

- header (10 bytes)
- payload (len bytes)
- crc32 (4 bytes)

The CRC32 covers header + payload and excludes the crc32 field itself.

### Header

The header is fixed-length (10 bytes):

- Offset 0, size 2: magic
- Offset 2, size 1: version
- Offset 3, size 1: type
- Offset 4, size 2: seq
- Offset 6, size 2: len
- Offset 8, size 2: flags

Field definitions:

- magic: MUST be 0x4B56 (ASCII "VK" in little-endian byte order: 0x56 0x4B on the wire)
- version: MUST be 0x01 for this spec version
- type: application message type identifier
- seq: sender sequence number, incrementing per transmitted packet per direction, wrapping at 65535 to 0
- len: payload length in bytes, MUST satisfy 0 <= len <= MAX_PAYLOAD
- flags: bitfield as defined below

### Flags

Flags is a 16-bit field:

- Bit 0: ACK_REQ
- Bit 1: IS_ACK
- Bits 2..15: reserved, MUST be transmitted as 0, and MUST be ignored on receipt if non-zero

Semantics:

- ACK_REQ = 1 indicates the sender requests an ACK for this packet.
- IS_ACK = 1 indicates this packet is an ACK packet.

### Payload

- The payload is an opaque byte array of length len.
- Payload interpretation is defined by the message type registry.

### CRC32 Field

- The trailing crc32 field is 4 bytes, little-endian.
- The crc32 value MUST be computed over the exact bytes of header + payload in the decoded packet.
- Receivers MUST validate crc32 before accepting a packet.

## CRC32 Definition

This protocol uses CRC-32/ISO-HDLC (also known as Ethernet/PKZIP CRC32) with:

- Polynomial: 0x04C11DB7
- Reflected input: yes
- Reflected output: yes
- Init value: 0xFFFFFFFF
- Final XOR: 0xFFFFFFFF

Implementations MUST ensure both ends compute identical CRC values for identical byte sequences.

Implementation notes:

- STM32H723: CRC peripheral SHOULD be configured to match the reflected CRC-32/ISO-HDLC variant, including init and output behavior. If the peripheral cannot directly apply the final XOR, software MUST apply the final XOR before comparing or transmitting.
- ESP32: software CRC32 MAY use ROM or table-based implementations, but MUST match the reflected/init/final XOR parameters above.

## Sender Requirements

### Packet Construction

For each outbound packet, the sender MUST:

1. Populate the header fields:
   - magic = 0x4B56
   - version = 0x01
   - type = application-defined
   - seq = next sequence number for this direction
   - len = payload length
   - flags = as needed (including ACK_REQ if requesting ACK)
2. Append payload bytes of length len.
3. Compute crc32 over header + payload and append crc32 (4 bytes, little-endian).
4. COBS-encode the complete decoded packet (header + payload + crc32).
5. Transmit the encoded bytes followed by a single delimiter byte 0x00.

### Sequence Numbering

- seq MUST increment by 1 for each packet transmitted by a sender on a given direction.
- seq MUST wrap from 65535 to 0.

## Receiver Requirements

### Candidate Frame Acquisition

Receivers MUST implement byte-stream parsing as follows:

- Accumulate received bytes until a 0x00 delimiter is encountered.
- The bytes preceding the delimiter form the candidate encoded frame.

Rules:

- If the candidate encoded frame length is 0, the receiver SHOULD ignore it.
- If the candidate encoded frame length exceeds MAX_ENCODED - 1 (excluding the delimiter), the receiver MUST drop it and resynchronize by continuing to scan for the next 0x00 delimiter.

### Decode and Validate

For each candidate encoded frame, the receiver MUST:

1. COBS-decode the candidate into a decoded buffer.
2. Verify decoded length is at least HEADER_SIZE + CRC_SIZE.
3. Parse and validate header:
   - magic MUST equal 0x4B56
   - version MUST equal 0x01, otherwise the frame MUST be dropped (or routed to a version handler if supported)
   - len MUST be <= MAX_PAYLOAD
4. Verify decoded length equals HEADER_SIZE + len + CRC_SIZE; otherwise drop.
5. Compute crc32 over the decoded bytes [0 .. HEADER_SIZE + len - 1] and compare to the trailing crc32.
   - On mismatch, drop.
6. If valid, deliver the packet (type, seq, flags, payload) to the application handler.

### Error Handling and Resynchronization

- On any decode error or validation failure, the receiver MUST drop the candidate frame without attempting partial recovery within that frame.
- Resynchronization MUST rely solely on scanning for the next 0x00 delimiter in the incoming stream.

## Optional Reliability: ACK Semantics

ACK behavior is optional. If enabled, both ends MUST implement the rules below.

### ACK Message Definition

- ACK packets MUST use type = 0x7F.
- ACK packets MUST set flags:
  - IS_ACK = 1
  - ACK_REQ = 0
- ACK packets MUST carry:
  - seq equal to the seq of the packet being acknowledged
  - len = 0
  - payload empty

### Receiver ACK Behavior

- If a non-ACK packet is received with ACK_REQ = 1, the receiver SHOULD transmit an ACK packet as defined above after validating CRC.
- Receivers MUST NOT ACK invalid packets.

### Sender Retry Policy

- If a sender transmits a packet with ACK_REQ = 1, it MAY retry transmission if an ACK is not received within a configured timeout.
- Retries SHOULD be bounded by N_RETRY.
- Implementations SHOULD expose configurable parameters:
  - T_ACK_MS: ACK timeout in milliseconds
  - N_RETRY: maximum number of retransmissions per packet

### Duplicate Handling

If ACK-based retries are used, receivers SHOULD handle duplicates:

- A receiver MAY track the most recent seq per peer direction and type (or per stream) and MAY drop duplicates while still sending an ACK for them.
- Exact duplicate handling strategy is application-defined but SHOULD avoid reapplying side effects.

## Message Type Registry

Initial type assignments:

- 0x01: Telemetry
- 0x02: Command
- 0x03: Log
- 0x7F: ACK (reserved)

Unknown types:

- Receivers SHOULD ignore unknown types after validating CRC and MAY count/trace them for diagnostics.

## Performance and Implementation Guidance

### Memory and Allocation

Implementations SHOULD avoid dynamic allocation in the receive path:

- Use fixed-size buffers sized to MAX_ENCODED and MAX_DECODED.
- Maintain a ring buffer for UART RX and scan for delimiter 0x00.

### Recommended RX Architecture

- Use UART RX DMA (STM32) or driver ring buffer (ESP32) to collect bytes continuously.
- Parse frames by locating 0x00 delimiters in the stream and extracting candidate frames.

### Fast-Fail Checks

Receivers SHOULD perform checks in this order for efficiency:

1. Encoded length bounds
2. COBS decode success
3. Minimum decoded length
4. magic and version
5. len bounds and exact decoded length match
6. CRC32 validation

### Statistics

Implementations SHOULD maintain counters for:

- frames_received
- frames_dropped_encoded_too_large
- frames_dropped_cobs_decode_error
- frames_dropped_bad_magic
- frames_dropped_bad_version
- frames_dropped_length_mismatch
- frames_dropped_crc_fail
- frames_accepted
- acks_sent
- acks_received
- retries

## Compliance Tests

A compliant implementation MUST pass the following:

- Correctly delimit frames by 0x00 and resync after injected noise.
- Correctly encode and decode COBS such that encoded data contains no 0x00.
- Correctly validate header fields and payload length.
- Correctly compute CRC32 per the defined parameters and reject corrupted frames.
- If ACK enabled, correctly generate and recognize ACK packets and match seq.

## Versioning

- This spec defines version 0x01.
- Future versions MUST change the version field.
- Receivers that do not support a version MUST drop frames with unknown version.

## Open Design Choices / Decisions Needed
- **RPC surface**: confirm required/optional RPC methods and error codes.
