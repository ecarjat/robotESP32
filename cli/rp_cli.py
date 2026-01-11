#!/usr/bin/env python3
import argparse
import json
import os
import shlex
import socket
import struct
import sys
import threading
import time
import zlib

try:
    import readline  # noqa: F401
except ImportError:  # pragma: no cover
    readline = None

ROBOT_FRAME_MAGIC = 0x4B56
ROBOT_FRAME_VERSION = 0x01
ROBOT_FRAME_MAX_PAYLOAD = 240

ROBOT_MSG_RPC_REQ = 0x30
ROBOT_MSG_RPC_RESP = 0x31
ROBOT_MSG_ACK = 0x7F
ROBOT_MSG_CMD_HEARTBEAT = 0x01

ROBOT_MSG_FILE_LIST_REQ = 0x20
ROBOT_MSG_FILE_LIST_RESP = 0x21
ROBOT_MSG_FILE_READ_REQ = 0x22
ROBOT_MSG_FILE_READ_RESP = 0x23
ROBOT_MSG_FILE_ERR = 0x24

ROBOT_FLAG_ACK_REQ = 0x0001
ROBOT_FLAG_IS_ACK = 0x0002

ROBOT_RPC_METHOD_GET_PARAM = 0x01
ROBOT_RPC_METHOD_SET_PARAM = 0x02
ROBOT_RPC_METHOD_IMU_CALIB_FACE = 0x10
ROBOT_RPC_METHOD_MOTOR_ENABLE = 0x20
ROBOT_RPC_METHOD_MOTOR_DISABLE = 0x21
ROBOT_RPC_METHOD_MOTOR_RUN = 0x22
ROBOT_RPC_METHOD_BALANCE_ENABLE = 0x23
ROBOT_RPC_METHOD_BALANCE_DISABLE = 0x24

ROBOT_RPC_FLAG_SAVE = 0x01

RPC_STATUS_NAMES = {
    0x00: "OK",
    0x01: "BAD_LEN",
    0x02: "BAD_OFFSET",
    0x03: "STORAGE_ERR",
    0x04: "BAD_METHOD",
    0x05: "BAD_PARAM",
    0x06: "NOT_READY",
    0x07: "TIMEOUT",
    0x08: "INCOMPLETE",
}

HEADER_FMT = "<HBBHHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

RPC_HDR_FMT = "<BBHH"
RPC_HDR_SIZE = struct.calcsize(RPC_HDR_FMT)
CALIB_FACE_FMT = "<BBH"
CALIB_FACE_SIZE = struct.calcsize(CALIB_FACE_FMT)
MOTOR_RUN_FMT = "<BBf"
MOTOR_RUN_SIZE = struct.calcsize(MOTOR_RUN_FMT)

ROBOT_IMU_FACE_X_POS_UP = 0
ROBOT_IMU_FACE_X_NEG_UP = 1
ROBOT_IMU_FACE_Y_POS_UP = 2
ROBOT_IMU_FACE_Y_NEG_UP = 3
ROBOT_IMU_FACE_Z_POS_UP = 4
ROBOT_IMU_FACE_Z_NEG_UP = 5

ROBOT_MOTOR_SIDE_LEFT = 0
ROBOT_MOTOR_SIDE_RIGHT = 1

TYPE_FORMATS = {
    "i8": ("b", 1),
    "u8": ("B", 1),
    "i16": ("h", 2),
    "u16": ("H", 2),
    "i32": ("i", 4),
    "u32": ("I", 4),
    "float": ("f", 4),
}

CALIB_TIMEOUT_S = 5.0
FILE_TRANSFER_TIMEOUT_S = 5.0
ROBOT_FILE_MAX_FILENAME = 64
ROBOT_FILE_CHUNK_SIZE = 200

ROBOT_FILE_ERR_NOT_FOUND = 1
ROBOT_FILE_ERR_READ_ERROR = 2
ROBOT_FILE_ERR_BUSY = 3
ROBOT_FILE_ERR_INVALID_REQ = 4

FILE_ERR_NAMES = {
    ROBOT_FILE_ERR_NOT_FOUND: "NOT_FOUND",
    ROBOT_FILE_ERR_READ_ERROR: "READ_ERROR",
    ROBOT_FILE_ERR_BUSY: "BUSY",
    ROBOT_FILE_ERR_INVALID_REQ: "INVALID_REQUEST",
}

FACE_NAME_MAP = {
    "UP": ROBOT_IMU_FACE_Z_POS_UP,
    "DOWN": ROBOT_IMU_FACE_Z_NEG_UP,
    "FRONT": ROBOT_IMU_FACE_X_NEG_UP,
    "BACK": ROBOT_IMU_FACE_X_POS_UP,
    "REAR": ROBOT_IMU_FACE_X_POS_UP,
    "LEFT": ROBOT_IMU_FACE_Y_NEG_UP,
    "RIGHT": ROBOT_IMU_FACE_Y_POS_UP,
    "X+": ROBOT_IMU_FACE_X_POS_UP,
    "+X": ROBOT_IMU_FACE_X_POS_UP,
    "X-": ROBOT_IMU_FACE_X_NEG_UP,
    "-X": ROBOT_IMU_FACE_X_NEG_UP,
    "Y+": ROBOT_IMU_FACE_Y_POS_UP,
    "+Y": ROBOT_IMU_FACE_Y_POS_UP,
    "Y-": ROBOT_IMU_FACE_Y_NEG_UP,
    "-Y": ROBOT_IMU_FACE_Y_NEG_UP,
    "Z+": ROBOT_IMU_FACE_Z_POS_UP,
    "+Z": ROBOT_IMU_FACE_Z_POS_UP,
    "Z-": ROBOT_IMU_FACE_Z_NEG_UP,
    "-Z": ROBOT_IMU_FACE_Z_NEG_UP,
}


class ParamEntry:
    def __init__(self, name, offset, ptype):
        if ptype not in TYPE_FORMATS:
            raise ValueError("Unsupported type: {}".format(ptype))
        self.name = name
        self.offset = offset
        self.ptype = ptype
        self.fmt = "<" + TYPE_FORMATS[ptype][0]
        self.size = TYPE_FORMATS[ptype][1]

    def encode(self, value_str):
        if self.ptype == "float":
            value = float(value_str)
        else:
            value = int(value_str, 0)
        return struct.pack(self.fmt, value)

    def decode(self, data):
        if len(data) != self.size:
            raise ValueError("Expected {} bytes, got {}".format(self.size, len(data)))
        return struct.unpack(self.fmt, data)[0]

    def format_value(self, value):
        if self.ptype == "float":
            return "{:.6g}".format(value)
        return str(value)


def load_params(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    params = []
    for item in data.get("params", []):
        params.append(ParamEntry(item["name"], int(item["offset"]), item["type"]))
    params.sort(key=lambda p: p.offset)
    return params


def robot_crc32(data):
    # zlib.crc32 already implements CRC-32/ISO-HDLC with init/xorout baked in.
    return zlib.crc32(data) & 0xFFFFFFFF


def cobs_encode(data):
    if not data:
        return b"\x01"
    out = bytearray()
    code_index = 0
    out.append(0)
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data):
    if not data:
        return b""
    out = bytearray()
    idx = 0
    length = len(data)
    while idx < length:
        code = data[idx]
        idx += 1
        if code == 0:
            return None
        for _ in range(1, code):
            if idx >= length:
                return None
            out.append(data[idx])
            idx += 1
        if code < 0xFF and idx < length:
            out.append(0)
    return bytes(out)


class RobotFrame:
    def __init__(self, msg_type, seq, flags, payload):
        self.msg_type = msg_type
        self.seq = seq
        self.flags = flags
        self.payload = payload


def encode_frame(msg_type, seq, flags, payload):
    if len(payload) > ROBOT_FRAME_MAX_PAYLOAD:
        raise ValueError("Payload too large: {}".format(len(payload)))
    header = struct.pack(
        HEADER_FMT,
        ROBOT_FRAME_MAGIC,
        ROBOT_FRAME_VERSION,
        msg_type,
        seq,
        len(payload),
        flags,
    )
    crc = robot_crc32(header + payload) & 0xFFFFFFFF
    decoded = header + payload + struct.pack("<I", crc)
    encoded = cobs_encode(decoded) + b"\x00"
    return encoded


def decode_frame(encoded):
    if not encoded or encoded[-1] != 0:
        return None
    decoded = cobs_decode(encoded[:-1])
    if decoded is None or len(decoded) < HEADER_SIZE + 4:
        return None
    hdr = struct.unpack_from(HEADER_FMT, decoded, 0)
    magic, version, msg_type, seq, length, flags = hdr
    if magic != ROBOT_FRAME_MAGIC or version != ROBOT_FRAME_VERSION:
        return None
    if length > ROBOT_FRAME_MAX_PAYLOAD:
        return None
    expected = HEADER_SIZE + length + 4
    if len(decoded) != expected:
        return None
    payload = decoded[HEADER_SIZE : HEADER_SIZE + length]
    crc_rx = struct.unpack_from("<I", decoded, HEADER_SIZE + length)[0]
    crc_calc = robot_crc32(decoded[: HEADER_SIZE + length]) & 0xFFFFFFFF
    if crc_rx != crc_calc:
        return None
    return RobotFrame(msg_type, seq, flags, payload)


class RobotClient:
    def __init__(self, host, port, timeout, retries):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.rx_buf = bytearray()
        self.seq = 0
        self.timeout = timeout
        self.retries = retries
        self.lock = threading.Lock()
        self.running = True
        self.heartbeat_enabled = True
        self.hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.hb_thread.start()

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass

    def _heartbeat_loop(self):
        while self.running:
            try:
                if self.heartbeat_enabled:
                    # Heartbeat is a command message with empty payload
                    # Using seq=0 as it doesn't require ACK or ordering
                    self.send_frame(ROBOT_MSG_CMD_HEARTBEAT, 0, 0, b"")
            except (OSError, ValueError):
                pass
            time.sleep(0.1)

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        if self.seq == 0:
            self.seq = 1
        return self.seq

    def send_frame(self, msg_type, seq, flags, payload):
        data = encode_frame(msg_type, seq, flags, payload)
        with self.lock:
            self.sock.sendall(data)

    def send_ack(self, seq):
        self.send_frame(ROBOT_MSG_ACK, seq, ROBOT_FLAG_IS_ACK, b"")

    def read_frame(self, timeout=None):
        deadline = None
        if timeout is not None:
            deadline = time.time() + timeout
        while True:
            idx = self.rx_buf.find(b"\x00")
            if idx != -1:
                raw = bytes(self.rx_buf[: idx + 1])
                del self.rx_buf[: idx + 1]
                frame = decode_frame(raw)
                if frame is None:
                    continue
                if frame.flags & ROBOT_FLAG_ACK_REQ:
                    self.send_ack(frame.seq)
                return frame
            if deadline is not None:
                remaining = deadline - time.time()
                if remaining <= 0:
                    raise TimeoutError("Timed out waiting for frame")
                self.sock.settimeout(remaining)
            data = self.sock.recv(4096)
            if not data:
                raise ConnectionError("Connection closed")
            self.rx_buf.extend(data)

    def set_heartbeat_enabled(self, enabled):
        self.heartbeat_enabled = enabled

    def rpc_exchange(self, method, offset, length, data, save_flag=False):
        flags = ROBOT_RPC_FLAG_SAVE if save_flag else 0
        payload = struct.pack(RPC_HDR_FMT, method, flags, offset, length) + data
        for _ in range(self.retries):
            seq = self.next_seq()
            self.send_frame(ROBOT_MSG_RPC_REQ, seq, ROBOT_FLAG_ACK_REQ, payload)
            acked = False
            resp = None
            deadline = time.time() + self.timeout
            while time.time() < deadline:
                try:
                    frame = self.read_frame(timeout=deadline - time.time())
                except TimeoutError:
                    break
                if frame.msg_type == ROBOT_MSG_ACK and frame.seq == seq:
                    if frame.flags & ROBOT_FLAG_IS_ACK:
                        acked = True
                elif frame.msg_type == ROBOT_MSG_RPC_RESP and frame.seq == seq:
                    resp = frame
                if resp is not None and acked:
                    return resp
                if resp is not None and not acked:
                    return resp
            # retry
        raise TimeoutError("RPC request timed out")


def parse_rpc_response(frame):
    if frame.msg_type != ROBOT_MSG_RPC_RESP:
        raise ValueError("Unexpected frame type: 0x{:02X}".format(frame.msg_type))
    if len(frame.payload) < RPC_HDR_SIZE:
        raise ValueError("RPC response too short")
    method, status, offset, length = struct.unpack_from(RPC_HDR_FMT, frame.payload, 0)
    data = frame.payload[RPC_HDR_SIZE:]
    return method, status, offset, length, data


def parse_face_name(name):
    key = name.strip().upper()
    if key in FACE_NAME_MAP:
        return FACE_NAME_MAP[key]
    raise ValueError("Unknown face: {}".format(name))


def parse_imu_name(name):
    if name is None:
        return 0
    key = name.strip().upper()
    if key in ("0", "IMU1", "PRIMARY", "BMI", "BMI270"):
        return 0
    if key in ("1", "IMU2", "SECONDARY", "ICM", "ICM42688"):
        return 1
    raise ValueError("Unknown IMU: {}".format(name))


def read_param_value(client, entry):
    frame = client.rpc_exchange(
        ROBOT_RPC_METHOD_GET_PARAM,
        entry.offset,
        entry.size,
        b"",
        save_flag=False,
    )
    method, status, offset, length, data = parse_rpc_response(frame)
    if status != 0:
        raise ValueError("status {}".format(RPC_STATUS_NAMES.get(status, "ERR")))
    if method != ROBOT_RPC_METHOD_GET_PARAM or offset != entry.offset:
        raise ValueError("unexpected response")
    if length != entry.size or len(data) != entry.size:
        raise ValueError("length mismatch")
    return entry.decode(data)


def write_param_value(client, entry, data, save_flag=False):
    frame = client.rpc_exchange(
        ROBOT_RPC_METHOD_SET_PARAM,
        entry.offset,
        entry.size,
        data,
        save_flag=save_flag,
    )
    method, status, offset, length, _data = parse_rpc_response(frame)
    if method != ROBOT_RPC_METHOD_SET_PARAM or offset != entry.offset or length != entry.size:
        return False, "unexpected response"
    if status != 0:
        return False, RPC_STATUS_NAMES.get(status, "ERR")
    return True, "OK"


def warn_balance_limits(client, param_map, entry, new_value):
    if entry.name == "balance.max_tilt_ref":
        other_name = "balance.thetaKill"
        max_tilt = new_value
        theta_kill = None
    elif entry.name == "balance.thetaKill":
        other_name = "balance.max_tilt_ref"
        max_tilt = None
        theta_kill = new_value
    else:
        return

    other_entry = param_map.get(other_name)
    if other_entry is None:
        print("WARN: ensure balance.max_tilt_ref < balance.thetaKill.")
        return
    try:
        other_value = read_param_value(client, other_entry)
    except (TimeoutError, ConnectionError, ValueError, struct.error) as exc:
        print("WARN: unable to check {} ({}). Ensure balance.max_tilt_ref < balance.thetaKill.".format(
            other_name, exc))
        return

    if max_tilt is None:
        max_tilt = other_value
    if theta_kill is None:
        theta_kill = other_value

    if theta_kill > 0.0 and max_tilt >= theta_kill:
        print("WARN: balance.max_tilt_ref ({:.6g}) >= balance.thetaKill ({:.6g}); keep max_tilt_ref < thetaKill."
              .format(max_tilt, theta_kill))


def load_param_file(path):
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    params = payload.get("params") if isinstance(payload, dict) else None
    if params is None:
        params = payload
    if not isinstance(params, dict):
        raise ValueError("param file must be a JSON object or contain a 'params' object")
    return params


def cmd_export(client, params, path):
    data = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "params": {},
    }
    errors = {}
    for entry in params:
        try:
            value = read_param_value(client, entry)
        except (TimeoutError, ConnectionError, ValueError, struct.error) as exc:
            errors[entry.name] = str(exc)
            continue
        data["params"][entry.name] = value
    if errors:
        data["errors"] = errors
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
    except OSError as exc:
        print("EXPORT failed: {}".format(exc))
        return
    if errors:
        print("EXPORT {}: OK ({} errors)".format(path, len(errors)))
    else:
        print("EXPORT {}: OK".format(path))


def cmd_import(client, param_map, path, save_flag=False):
    try:
        values = load_param_file(path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print("IMPORT failed: {}".format(exc))
        return

    applied = 0
    failed = 0
    skipped = 0
    for name, value in values.items():
        entry = param_map.get(name)
        if entry is None:
            skipped += 1
            continue
        if isinstance(value, bool):
            value_str = "1" if value else "0"
        elif isinstance(value, (int, float)):
            value_str = str(value)
        elif isinstance(value, str):
            value_str = value
        else:
            print("IMPORT {}: invalid value type {}".format(name, type(value).__name__))
            failed += 1
            continue
        try:
            data = entry.encode(value_str)
        except (ValueError, struct.error) as exc:
            print("IMPORT {}: invalid value ({})".format(name, exc))
            failed += 1
            continue
        if entry.name in ("balance.max_tilt_ref", "balance.thetaKill"):
            try:
                new_value = entry.decode(data)
            except (ValueError, struct.error):
                new_value = None
            if new_value is not None:
                warn_balance_limits(client, param_map, entry, new_value)
        try:
            ok, msg = write_param_value(client, entry, data, save_flag=False)
        except (TimeoutError, ConnectionError, ValueError) as exc:
            print("IMPORT {}: failed ({})".format(name, exc))
            failed += 1
            continue
        if not ok:
            print("IMPORT {}: {}".format(name, msg))
            failed += 1
            continue
        applied += 1

    if save_flag and applied > 0:
        cmd_save(client)

    print("IMPORT {}: applied={} skipped={} failed={}".format(path, applied, skipped, failed))


def print_help():
    print("Commands:")
    print("  GET <param>          Read a parameter by name")
    print("  GET ALL              Dump all known parameters")
    print("  GET <prefix>.*       Dump parameters by prefix (e.g., balance.*)")
    print("  SET <param> <value> [SAVE]  Set a parameter (optional SAVE persists)")
    print("  EXPORT <file>        Save all parameters to a local JSON file")
    print("  IMPORT <file> [SAVE] Load parameters from a local JSON file")
    print("  CALIB <face> [imu]   Capture IMU calibration face (use SAVE to persist)")
    print("  CALIBRATE <face> [imu]  Alias for CALIB")
    print("  BALANCE | ARM        Put robot in balancing mode (arm)")
    print("  DISARM               Disarm and disable motors")
    print("  MOTOR ENABLE|DISABLE Enable or disable motors for manual run")
    print("  RUN LEFT|RIGHT <intensity>  Manual run (-1..1, scaled by IqMax)")
    print("  FILES | LIST         List files on SD card")
    print("  PULL <file> [dest]   Download file from SD card (robot must not be balancing)")
    print("  SAVE                 Persist parameters to flash (firmware support required)")
    print("  HELP                 Show this help")
    print("  EXIT | QUIT          Exit the CLI")
    print("")
    print("Faces: up, down, front, back, left, right, x+/x-, y+/y-, z+/z-")
    print("  front/back/left/right assume that face is DOWN (e.g., front = -X up).")
    print("IMU: imu1/bmi270 or imu2/icm42688 (default: imu1)")


def setup_readline(param_names):
    if readline is None:
        return
    readline.parse_and_bind("tab: complete")
    readline.set_completer_delims(" \t\n")
    commands = [
        "GET",
        "SET",
        "EXPORT",
        "IMPORT",
        "CALIB",
        "CALIBRATE",
        "BALANCE",
        "ARM",
        "MOTOR",
        "RUN",
        "DISARM",
        "FILES",
        "LIST",
        "PULL",
        "SAVE",
        "HELP",
        "EXIT",
        "QUIT",
    ]
    param_list = sorted(param_names)

    def completer(text, state):
        line = readline.get_line_buffer()
        beg = readline.get_begidx()
        tokens = line[:beg].split()
        options = []
        if not tokens:
            options = commands
        elif len(tokens) == 1:
            cmd = tokens[0].upper()
            if cmd in ("GET", "SET"):
                options = param_list + (["ALL", "balance.*"] if cmd == "GET" else [])
            elif cmd in ("CALIB", "CALIBRATE"):
                options = sorted(set(FACE_NAME_MAP.keys()))
            elif cmd == "MOTOR":
                options = ["ENABLE", "DISABLE"]
            elif cmd == "RUN":
                options = ["LEFT", "RIGHT"]
            elif cmd in ("EXPORT", "IMPORT"):
                options = []
            elif cmd in commands:
                options = []
            else:
                options = commands
        else:
            cmd = tokens[0].upper()
            if cmd == "GET":
                options = param_list + ["ALL", "balance.*"]
            elif cmd == "SET" and len(tokens) == 2:
                options = param_list
            elif cmd in ("CALIB", "CALIBRATE") and len(tokens) == 2:
                options = sorted(set(FACE_NAME_MAP.keys()))
            elif cmd == "MOTOR" and len(tokens) == 2:
                options = ["ENABLE", "DISABLE"]
            elif cmd == "RUN" and len(tokens) == 2:
                options = ["LEFT", "RIGHT"]
            elif cmd == "IMPORT" and len(tokens) == 3:
                options = ["SAVE"]
            else:
                options = []
        matches = [opt for opt in options if opt.upper().startswith(text.upper())]
        try:
            return matches[state]
        except IndexError:
            return None

    readline.set_completer(completer)


def cmd_get(client, param_map, name):
    if name.endswith(".*"):
        prefix = name[:-1]
        matches = [entry.name for entry in param_map.values() if entry.name.startswith(prefix)]
        if not matches:
            print("Unknown param prefix: {}".format(prefix))
            return
        for entry_name in sorted(matches):
            cmd_get(client, param_map, entry_name)
        return

    entry = param_map.get(name)
    if entry is None:
        print("Unknown param: {}".format(name))
        return
    try:
        frame = client.rpc_exchange(
            ROBOT_RPC_METHOD_GET_PARAM,
            entry.offset,
            entry.size,
            b"",
            save_flag=False,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("GET failed: {}".format(exc))
        return
    try:
        method, status, offset, length, data = parse_rpc_response(frame)
    except ValueError as exc:
        print("GET failed: {}".format(exc))
        return
    if status != 0:
        print("GET {}: {}".format(name, RPC_STATUS_NAMES.get(status, "ERR")))
        return
    if method != ROBOT_RPC_METHOD_GET_PARAM or offset != entry.offset:
        print("GET {}: unexpected response".format(name))
        return
    if length != entry.size or len(data) != entry.size:
        print("GET {}: length mismatch".format(name))
        return
    try:
        value = entry.decode(data)
    except (ValueError, struct.error) as exc:
        print("GET {}: decode error ({})".format(name, exc))
        return
    print("{} = {} (0x{})".format(name, entry.format_value(value), data.hex()))


def cmd_set(client, param_map, name, value_str, save_flag=False):
    entry = param_map.get(name)
    if entry is None:
        print("Unknown param: {}".format(name))
        return
    try:
        data = entry.encode(value_str)
    except (ValueError, struct.error) as exc:
        print("SET {}: invalid value ({})".format(name, exc))
        return
    if entry.name in ("balance.max_tilt_ref", "balance.thetaKill"):
        try:
            new_value = entry.decode(data)
        except (ValueError, struct.error):
            new_value = None
        if new_value is not None:
            warn_balance_limits(client, param_map, entry, new_value)
    try:
        ok, msg = write_param_value(client, entry, data, save_flag=save_flag)
    except (TimeoutError, ConnectionError, ValueError) as exc:
        print("SET failed: {}".format(exc))
        return
    if not ok:
        print("SET {}: {}".format(name, msg))
        return
    print("SET {}: OK".format(name))


def cmd_save(client):
    try:
        frame = client.rpc_exchange(
            ROBOT_RPC_METHOD_SET_PARAM,
            0,
            0,
            b"",
            save_flag=True,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("SAVE failed: {}".format(exc))
        return
    try:
        method, status, _offset, _length, _data = parse_rpc_response(frame)
    except ValueError as exc:
        print("SAVE failed: {}".format(exc))
        return
    if method != ROBOT_RPC_METHOD_SET_PARAM:
        print("SAVE: unexpected response")
        return
    if status == 0x01:
        print("SAVE: not supported by firmware (requires save RPC)")
        return
    if status != 0:
        print("SAVE: {}".format(RPC_STATUS_NAMES.get(status, "ERR")))
        return
    print("SAVE: OK")


def cmd_calib(client, face_name, imu_name=None):
    try:
        face = parse_face_name(face_name)
    except ValueError as exc:
        print("CALIB: {}".format(exc))
        return
    try:
        imu = parse_imu_name(imu_name)
    except ValueError as exc:
        print("CALIB: {}".format(exc))
        return
    payload = struct.pack(CALIB_FACE_FMT, imu, face, 0)
    prev_timeout = client.timeout
    if client.timeout < CALIB_TIMEOUT_S:
        client.timeout = CALIB_TIMEOUT_S
    try:
        frame = client.rpc_exchange(
            ROBOT_RPC_METHOD_IMU_CALIB_FACE,
            0,
            0,
            payload,
            save_flag=False,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("CALIB failed: {}".format(exc))
        client.timeout = prev_timeout
        return
    client.timeout = prev_timeout
    try:
        method, status, _offset, _length, _data = parse_rpc_response(frame)
    except ValueError as exc:
        print("CALIB failed: {}".format(exc))
        return
    if method != ROBOT_RPC_METHOD_IMU_CALIB_FACE:
        print("CALIB: unexpected response")
        return
    if status == 0:
        print("CALIB {}: OK".format(face_name))
        return
    if status == 0x08:
        print("CALIB {}: captured (need all 6 faces)".format(face_name))
        return
    print("CALIB {}: {}".format(face_name, RPC_STATUS_NAMES.get(status, "ERR")))


def cmd_balance(client):
    try:
        frame = client.rpc_exchange(
            ROBOT_RPC_METHOD_BALANCE_ENABLE,
            0,
            0,
            b"",
            save_flag=False,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("BALANCE failed: {}".format(exc))
        return
    try:
        resp_method, status, _offset, _length, _data = parse_rpc_response(frame)
    except ValueError as exc:
        print("BALANCE failed: {}".format(exc))
        return
    if resp_method != ROBOT_RPC_METHOD_BALANCE_ENABLE:
        print("BALANCE: unexpected response")
        return
    if status != 0:
        print("BALANCE: {}".format(RPC_STATUS_NAMES.get(status, "ERR")))
        return
    print("BALANCE: OK")


def cmd_disarm(client):
    try:
        frame = client.rpc_exchange(
            ROBOT_RPC_METHOD_BALANCE_DISABLE,
            0,
            0,
            b"",
            save_flag=False,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("DISARM failed: {}".format(exc))
        return
    try:
        resp_method, status, _offset, _length, _data = parse_rpc_response(frame)
    except ValueError as exc:
        print("DISARM failed: {}".format(exc))
        return
    if resp_method != ROBOT_RPC_METHOD_BALANCE_DISABLE:
        print("DISARM: unexpected response")
        return
    if status != 0:
        print("DISARM: {}".format(RPC_STATUS_NAMES.get(status, "ERR")))
        return
    print("DISARM: OK")


def cmd_motor_enable(client, enable):
    method = ROBOT_RPC_METHOD_MOTOR_ENABLE if enable else ROBOT_RPC_METHOD_MOTOR_DISABLE
    label = "ENABLE" if enable else "DISABLE"
    try:
        frame = client.rpc_exchange(
            method,
            0,
            0,
            b"",
            save_flag=False,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("MOTOR {} failed: {}".format(label, exc))
        return
    try:
        resp_method, status, _offset, _length, _data = parse_rpc_response(frame)
    except ValueError as exc:
        print("MOTOR {} failed: {}".format(label, exc))
        return
    if resp_method != method:
        print("MOTOR {}: unexpected response".format(label))
        return
    if status != 0:
        print("MOTOR {}: {}".format(label, RPC_STATUS_NAMES.get(status, "ERR")))
        return
    print("MOTOR {}: OK".format(label))


def cmd_run(client, side_name, value_str):
    side_key = side_name.strip().upper()
    if side_key == "LEFT":
        side = ROBOT_MOTOR_SIDE_LEFT
    elif side_key == "RIGHT":
        side = ROBOT_MOTOR_SIDE_RIGHT
    else:
        print("RUN: side must be LEFT or RIGHT")
        return
    try:
        intensity = float(value_str)
    except ValueError:
        print("RUN: invalid intensity")
        return
    payload = struct.pack(MOTOR_RUN_FMT, side, 0, intensity)
    try:
        frame = client.rpc_exchange(
            ROBOT_RPC_METHOD_MOTOR_RUN,
            0,
            0,
            payload,
            save_flag=False,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("RUN failed: {}".format(exc))
        return
    try:
        method, status, _offset, _length, _data = parse_rpc_response(frame)
    except ValueError as exc:
        print("RUN failed: {}".format(exc))
        return
    if method != ROBOT_RPC_METHOD_MOTOR_RUN:
        print("RUN: unexpected response")
        return
    if status != 0:
        print("RUN: {}".format(RPC_STATUS_NAMES.get(status, "ERR")))
        return
    print("RUN {}: OK".format(side_name))


def cmd_file_list(client):
    """List files on SD card"""
    try:
        seq = client.next_seq()
        payload = struct.pack("<B", 0)  # reserved byte
        client.send_frame(ROBOT_MSG_FILE_LIST_REQ, seq, ROBOT_FLAG_ACK_REQ, payload)

        # Wait for response
        deadline = time.time() + FILE_TRANSFER_TIMEOUT_S
        while time.time() < deadline:
            try:
                frame = client.read_frame(timeout=deadline - time.time())
            except TimeoutError:
                break

            if frame.msg_type == ROBOT_MSG_FILE_ERR and frame.seq == seq:
                if len(frame.payload) >= 1:
                    error_code = struct.unpack_from("<B", frame.payload, 0)[0]
                    print("FILE LIST: {}".format(FILE_ERR_NAMES.get(error_code, "ERR")))
                else:
                    print("FILE LIST: error")
                return

            if frame.msg_type == ROBOT_MSG_FILE_LIST_RESP and frame.seq == seq:
                if len(frame.payload) < 2:
                    print("FILE LIST: invalid response")
                    return

                count, more = struct.unpack_from("<BB", frame.payload, 0)
                offset = 2

                if count == 0:
                    print("No files found")
                    return

                print("Files on SD card:")
                for i in range(count):
                    if offset + ROBOT_FILE_MAX_FILENAME + 4 > len(frame.payload):
                        break
                    filename_bytes = frame.payload[offset:offset + ROBOT_FILE_MAX_FILENAME]
                    filename = filename_bytes.split(b'\x00', 1)[0].decode('utf-8', errors='replace')
                    size = struct.unpack_from("<I", frame.payload, offset + ROBOT_FILE_MAX_FILENAME)[0]
                    print("  {:32s}  {:10d} bytes".format(filename, size))
                    offset += ROBOT_FILE_MAX_FILENAME + 4

                if more:
                    print("  (more files not shown)")
                return

        print("FILE LIST: timeout")
    except (TimeoutError, ConnectionError) as exc:
        print("FILE LIST failed: {}".format(exc))


def cmd_file_pull(client, filename, dest_path=None):
    """Download a file from SD card"""
    if dest_path is None:
        dest_path = filename

    # Validate filename length
    if len(filename) >= ROBOT_FILE_MAX_FILENAME:
        print("PULL: filename too long (max {} chars)".format(ROBOT_FILE_MAX_FILENAME - 1))
        return

    try:
        client.set_heartbeat_enabled(False)
        # First request to get file size
        seq = client.next_seq()
        filename_bytes = filename.encode('utf-8').ljust(ROBOT_FILE_MAX_FILENAME, b'\x00')
        payload = filename_bytes + struct.pack("<IH", 0, 0)  # offset=0, length=0 to get size
        client.send_frame(ROBOT_MSG_FILE_READ_REQ, seq, ROBOT_FLAG_ACK_REQ, payload)

        # Wait for first response
        deadline = time.time() + FILE_TRANSFER_TIMEOUT_S
        frame = None
        while time.time() < deadline:
            try:
                frame = client.read_frame(timeout=deadline - time.time())
            except TimeoutError:
                break

            if frame.msg_type == ROBOT_MSG_FILE_ERR and frame.seq == seq:
                if len(frame.payload) >= 1:
                    error_code = struct.unpack_from("<B", frame.payload, 0)[0]
                    print("PULL: {}".format(FILE_ERR_NAMES.get(error_code, "ERR")))
                else:
                    print("PULL: error")
                return

            if frame.msg_type == ROBOT_MSG_FILE_READ_RESP and frame.seq == seq:
                if len(frame.payload) < 10:
                    print("PULL: short FILE_READ_RESP (len={})".format(len(frame.payload)))
                    frame = None
                    continue
                break

        if frame is None or frame.msg_type != ROBOT_MSG_FILE_READ_RESP:
            print("PULL: timeout waiting for response")
            return

        if len(frame.payload) < 10:
            print("PULL: invalid response")
            return

        offset, total_size, chunk_len = struct.unpack_from("<IIH", frame.payload, 0)

        print("Downloading {} ({} bytes)...".format(filename, total_size))

        # Open output file
        try:
            out_file = open(dest_path, "wb")
        except OSError as exc:
            print("PULL: failed to create output file: {}".format(exc))
            return

        try:
            # Download file in chunks
            bytes_received = 0
            current_offset = 0

            while current_offset < total_size:
                # Request next chunk
                seq = client.next_seq()
                chunk_size = min(ROBOT_FILE_CHUNK_SIZE, total_size - current_offset)
                payload = filename_bytes + struct.pack("<IH", current_offset, chunk_size)
                client.send_frame(ROBOT_MSG_FILE_READ_REQ, seq, ROBOT_FLAG_ACK_REQ, payload)

                # Wait for response
                deadline = time.time() + FILE_TRANSFER_TIMEOUT_S
                frame = None
                while time.time() < deadline:
                    try:
                        frame = client.read_frame(timeout=deadline - time.time())
                    except TimeoutError:
                        break

                    if frame.msg_type == ROBOT_MSG_FILE_ERR and frame.seq == seq:
                        if len(frame.payload) >= 1:
                            error_code = struct.unpack_from("<B", frame.payload, 0)[0]
                            print("\nPULL: {} at offset {}".format(
                                FILE_ERR_NAMES.get(error_code, "ERR"), current_offset))
                        else:
                            print("\nPULL: error at offset {}".format(current_offset))
                        return

                    if frame.msg_type == ROBOT_MSG_FILE_READ_RESP and frame.seq == seq:
                        if len(frame.payload) < 10:
                            print("\nPULL: short FILE_READ_RESP at offset {} (len={})".format(
                                current_offset, len(frame.payload)))
                            frame = None
                            continue
                        break

                if frame is None or frame.msg_type != ROBOT_MSG_FILE_READ_RESP:
                    print("\nPULL: timeout at offset {}".format(current_offset))
                    return

                if len(frame.payload) < 10:
                    print("\nPULL: invalid response at offset {}".format(current_offset))
                    return

                resp_offset, resp_total_size, resp_chunk_len = struct.unpack_from("<IIH", frame.payload, 0)

                if resp_offset != current_offset:
                    print("\nPULL: offset mismatch (expected {}, got {})".format(
                        current_offset, resp_offset))
                    return

                if resp_chunk_len > 0:
                    chunk_data = frame.payload[10:10 + resp_chunk_len]
                    out_file.write(chunk_data)
                    bytes_received += resp_chunk_len
                    current_offset += resp_chunk_len

                    # Progress indicator
                    progress = (bytes_received * 100) // total_size if total_size > 0 else 100
                    print("\r  Progress: {}% ({}/{} bytes)".format(
                        progress, bytes_received, total_size), end='', flush=True)
                else:
                    # Empty chunk, shouldn't happen unless EOF
                    break

            print("\nPULL: OK - saved to {}".format(dest_path))

        finally:
            out_file.close()

    except (TimeoutError, ConnectionError) as exc:
        print("\nPULL failed: {}".format(exc))
    finally:
        client.set_heartbeat_enabled(True)


def repl(client, params):
    param_map = {p.name: p for p in params}
    setup_readline(param_map.keys())
    print("Connected. Type HELP for commands.")
    while True:
        try:
            line = input("rp> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        line = line.strip()
        if not line:
            continue
        try:
            tokens = shlex.split(line)
        except ValueError:
            print("Parse error")
            continue
        if not tokens:
            continue
        cmd = tokens[0].upper()
        if cmd in ("EXIT", "QUIT"):
            break
        if cmd == "HELP":
            print_help()
            continue
        if cmd == "GET":
            if len(tokens) == 2 and tokens[1].upper() == "ALL":
                for entry in params:
                    cmd_get(client, param_map, entry.name)
                continue
            if len(tokens) != 2:
                print("Usage: GET <param> | GET ALL")
                continue
            cmd_get(client, param_map, tokens[1])
            continue
        if cmd == "SET":
            if len(tokens) < 3 or len(tokens) > 4:
                print("Usage: SET <param> <value> [SAVE]")
                continue
            save_flag = False
            if len(tokens) == 4:
                if tokens[3].upper() != "SAVE":
                    print("Usage: SET <param> <value> [SAVE]")
                    continue
                save_flag = True
            cmd_set(client, param_map, tokens[1], tokens[2], save_flag=save_flag)
            continue
        if cmd == "EXPORT":
            if len(tokens) != 2:
                print("Usage: EXPORT <file>")
                continue
            cmd_export(client, params, tokens[1])
            continue
        if cmd == "IMPORT":
            if len(tokens) < 2 or len(tokens) > 3:
                print("Usage: IMPORT <file> [SAVE]")
                continue
            save_flag = False
            if len(tokens) == 3:
                if tokens[2].upper() != "SAVE":
                    print("Usage: IMPORT <file> [SAVE]")
                    continue
                save_flag = True
            cmd_import(client, param_map, tokens[1], save_flag=save_flag)
            continue
        if cmd == "MOTOR":
            if len(tokens) != 2 or tokens[1].upper() not in ("ENABLE", "DISABLE"):
                print("Usage: MOTOR ENABLE|DISABLE")
                continue
            cmd_motor_enable(client, tokens[1].upper() == "ENABLE")
            continue
        if cmd == "RUN":
            if len(tokens) != 3:
                print("Usage: RUN LEFT|RIGHT <intensity>")
                continue
            cmd_run(client, tokens[1], tokens[2])
            continue
        if cmd in ("CALIB", "CALIBRATE"):
            if len(tokens) < 2 or len(tokens) > 3:
                print("Usage: CALIB <face> [imu]")
                continue
            imu_name = tokens[2] if len(tokens) == 3 else None
            cmd_calib(client, tokens[1], imu_name)
            continue
        if cmd in ("BALANCE", "ARM"):
            if len(tokens) != 1:
                print("Usage: BALANCE")
                continue
            cmd_balance(client)
            continue
        if cmd == "DISARM":
            if len(tokens) != 1:
                print("Usage: DISARM")
                continue
            cmd_disarm(client)
            continue
        if cmd in ("FILES", "LIST"):
            if len(tokens) != 1:
                print("Usage: FILES")
                continue
            cmd_file_list(client)
            continue
        if cmd == "PULL":
            if len(tokens) < 2 or len(tokens) > 3:
                print("Usage: PULL <filename> [dest]")
                continue
            dest = tokens[2] if len(tokens) == 3 else None
            cmd_file_pull(client, tokens[1], dest)
            continue
        if cmd == "SAVE":
            cmd_save(client)
            continue
        print("Unknown command: {}".format(tokens[0]))


def main():
    parser = argparse.ArgumentParser(description="Robot Protocol passthrough CLI")
    parser.add_argument("--host", required=True, help="ESP32 IP or hostname")
    parser.add_argument("--port", type=int, default=7777, help="TCP port (default: 7777)")
    parser.add_argument("--params", default=None, help="Path to params.json")
    parser.add_argument("--timeout", type=float, default=1.0, help="RPC timeout in seconds")
    parser.add_argument("--retries", type=int, default=3, help="RPC retries")
    args = parser.parse_args()

    if args.params is None:
        args.params = os.path.join(os.path.dirname(__file__), "params.json")

    try:
        params = load_params(args.params)
    except (OSError, ValueError, KeyError) as exc:
        print("Failed to load params: {}".format(exc))
        return 1

    try:
        client = RobotClient(args.host, args.port, args.timeout, args.retries)
    except OSError as exc:
        print("Connection failed: {}".format(exc))
        return 1

    try:
        repl(client, params)
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
