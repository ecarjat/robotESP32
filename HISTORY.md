# robotESP32-Arduino

## 2026-01-04 (v0.1, "working version")
- Expanded the Robot Protocol CLI with calibration, balance/arm, disarm, and motor run controls.
- Improved CLI usability (new RPC status names, tab completion entries, updated help) and adjusted CRC32 handling.
- Added Python packaging metadata via Poetry.
- Refined ESP32 firmware behavior in passthrough/UI/link handling and tuned configuration defaults.

## 2026-01-02 (Initial commit)
- Bootstrapped the ESP32 Arduino firmware with STM32 link handling, gamepad menu/UI, and RP passthrough mode.
- Added Robot Protocol CLI, parameter map tooling, and protocol documentation.
- Established project structure, build configuration, and tooling scripts.
