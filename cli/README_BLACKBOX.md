# Blackbox Flight Recorder - Usage Guide

The STM32H723 firmware includes a complete blackbox flight recorder system that logs control loop data at 400Hz to QSPI flash memory.

## Overview

- **Storage**: W25Q64 8MB QSPI flash (ring buffer, ~128 seconds)
- **Log Rate**: 400 Hz (matches control loop)
- **Record Size**: 160 bytes (fixed)
- **Dump Format**: Binary files `DUMP_0001.BIN`, `DUMP_0002.BIN`, etc.

## Logged Data

Each record contains (when enabled via `log_fields_mask`):

### IMU Raw Data (LOGF_IMU_RAW)
- 3-axis accelerometer (int16, sensor units)
- 3-axis gyroscope (int16, sensor units)

### Dual-IMU Health (LOGF_IMU2_HEALTH)
- Gyro disagreement (deg/s)
- Accelerometer angle disagreement (deg)
- Vibration RMS (g)

### EKF State (LOGF_EKF)
- Pitch angle θ (rad)
- Pitch rate θ̇ (rad/s)
- Gyro bias estimate (rad/s)
- Position x (m)
- Velocity ẋ (m/s)

### Wheel Velocities (LOGF_WHEELS)
- Left wheel ωL (rad/s)
- Right wheel ωR (rad/s)
- Linear velocity v (m/s)

### PID Controller (LOGF_PID)
- Pitch target θ_ref (rad)
- Pitch error e_θ (rad)
- P, I, D terms
- Motor commands uL, uR (Iq proxy)

### Status Flags
- `ARMED`: Robot is balancing
- `FALLEN`: Robot has fallen
- `SATURATED`: Motor output saturated
- `ACCEL_GATED`: High vibration detected
- `IMU_FALLBACK`: Using secondary IMU

## Triggering Dumps

### Automatic (Fall Detection)
When the robot falls, a dump is automatically triggered:
- Logs last N seconds (configurable via `dump_seconds_default`)
- Writes to SD card as `DUMP_0001.BIN`
- Non-blocking background operation

### Manual (X Button)
Press the X button on the ESP32 controller:
- Sends `ROBOT_TELEOP_FLAG_DUMP` (0x08)
- Dumps last N seconds to SD card
- Useful for capturing successful runs

## Configuration Parameters

Set via ESP32 CLI (after regenerating params.json):

```bash
# Enable/disable field groups (bitmask)
log_fields_mask = 0x0000001F  # All fields enabled (default)
                  # 0x01 = IMU_RAW
                  # 0x02 = IMU2_HEALTH
                  # 0x04 = EKF
                  # 0x08 = WHEELS
                  # 0x10 = PID

# Dump window size (seconds)
dump_seconds_default = 30  # Last 30 seconds (default)
```

## Parsing Dump Files

Use the Python parser to analyze dump files:

### Installation

```bash
pip install matplotlib  # Optional, for plotting
```

### Usage

**View dump information:**
```bash
python tools/parse_blackbox.py DUMP_0001.BIN --info
```

**Export to CSV:**
```bash
python tools/parse_blackbox.py DUMP_0001.BIN --csv data.csv
```

**Plot key signals:**
```bash
python tools/parse_blackbox.py DUMP_0001.BIN --plot
```

**Export and plot:**
```bash
python tools/parse_blackbox.py DUMP_0001.BIN --csv data.csv --plot
```

### CSV Output

The exported CSV contains:
- Timestamp (µs and seconds)
- All telemetry fields (scaled to SI units)
- Status flags as boolean columns
- Pitch angle in both radians and degrees

Perfect for:
- Loading into Excel/Google Sheets
- Analysis with pandas/NumPy
- Custom plotting scripts

### Plotting

The `--plot` option generates 4 subplots:
1. **Pitch Angle**: θ vs θ_ref with armed regions highlighted
2. **Pitch Rate**: θ̇ over time
3. **Wheel Velocities**: ωL and ωR
4. **Motor Commands**: uL and uR

## File Format

```
DUMP_0001.BIN format:
├── LogMeta (68 bytes)
│   ├── Magic: "R2WLOG1\0"
│   ├── Version, record size, rate
│   ├── Field mask, ring parameters
│   └── CRC32
├── LogRecord[N] (160 bytes each)
│   ├── Header (seq, timestamp, flags)
│   ├── IMU data
│   ├── EKF state
│   ├── Wheels
│   ├── PID
│   └── CRC32
└── LogDumpTrailer (16 bytes)
    ├── Start/end sequence numbers
    ├── Record count
    └── CRC32 (reserved, not computed)
```

## Troubleshooting

**No dumps appearing on SD card:**
- Check SD card is inserted and formatted (FAT32)
- Verify SD card is mounted at boot (check logs)
- Ensure robot has been armed/balanced before dump

**Parse errors:**
- Verify file is a complete dump (not truncated)
- Check CRC warnings in parser output
- Ensure correct file format (DUMP_*.BIN from robot)

**Missing data in records:**
- Check `log_fields_mask` parameter
- Disabled fields are zero-filled in records
- Default is all fields enabled (0x1F)

## Performance Impact

The blackbox system is designed for minimal impact:
- **Control loop**: ~50 µs per record (@ 400Hz)
  - Zero-copy push to RAM queue
  - ISR-safe with critical sections
- **Background tasks**:
  - Writer: Coalesces to 4KB chunks
  - Eraser: Pre-erases 8 sectors ahead
  - Dump: Non-blocking state machine
- **Memory**:
  - RAM queue: 256 KB (.dma_buffer)
  - Flash: 8188 KB ring buffer

## Integration with ESP32

To enable X button dump trigger, update ESP32 firmware to send the flag:

```cpp
// In your button handler
if (x_button_pressed) {
  teleop_flags |= ROBOT_TELEOP_FLAG_DUMP;  // 0x08
}
```

The STM32 will acknowledge and start the dump in the background.

## Developer Notes

- Records are always 160 bytes (fixed size for ring buffer math)
- Unpopulated fields are zero-filled
- CRC is computed per-record (not for entire dump due to lack of incremental API)
- Metadata uses dual-slot redundancy for power-loss tolerance
- Flash erase is pre-emptive (8 sectors ahead) to avoid blocking
- Dump state machine runs cooperatively in `app_idle_tick()`

## Credits

Blackbox system designed for R2W self-balancing robot project.
Generated with assistance from Claude Code.
