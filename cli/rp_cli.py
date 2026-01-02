#!/usr/bin/env python3
import argparse
import json
import os
import shlex
import socket
import struct
import sys
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

ROBOT_FLAG_ACK_REQ = 0x0001
ROBOT_FLAG_IS_ACK = 0x0002

ROBOT_RPC_METHOD_GET_PARAM = 0x01
ROBOT_RPC_METHOD_SET_PARAM = 0x02

ROBOT_RPC_FLAG_SAVE = 0x01

RPC_STATUS_NAMES = {
    0x00: "OK",
    0x01: "BAD_LEN",
    0x02: "BAD_OFFSET",
    0x03: "STORAGE_ERR",
    0x04: "BAD_METHOD",
}

HEADER_FMT = "<HBBHHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

RPC_HDR_FMT = "<BBHH"
RPC_HDR_SIZE = struct.calcsize(RPC_HDR_FMT)

TYPE_FORMATS = {
    "i8": ("b", 1),
    "u8": ("B", 1),
    "i16": ("h", 2),
    "u16": ("H", 2),
    "i32": ("i", 4),
    "u32": ("I", 4),
    "float": ("f", 4),
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
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


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

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        if self.seq == 0:
            self.seq = 1
        return self.seq

    def send_frame(self, msg_type, seq, flags, payload):
        self.sock.sendall(encode_frame(msg_type, seq, flags, payload))

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


def print_help():
    print("Commands:")
    print("  GET <param>          Read a parameter by name")
    print("  GET ALL              Dump all known parameters")
    print("  SET <param> <value> [SAVE]  Set a parameter (optional SAVE persists)")
    print("  SAVE                 Persist parameters to flash (firmware support required)")
    print("  HELP                 Show this help")
    print("  EXIT | QUIT          Exit the CLI")


def setup_readline(param_names):
    if readline is None:
        return
    readline.parse_and_bind("tab: complete")
    readline.set_completer_delims(" \t\n")
    commands = ["GET", "SET", "SAVE", "HELP", "EXIT", "QUIT"]
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
                options = param_list + (["ALL"] if cmd == "GET" else [])
            elif cmd in commands:
                options = []
            else:
                options = commands
        else:
            cmd = tokens[0].upper()
            if cmd == "GET":
                options = param_list + ["ALL"]
            elif cmd == "SET" and len(tokens) == 2:
                options = param_list
            else:
                options = []
        matches = [opt for opt in options if opt.upper().startswith(text.upper())]
        try:
            return matches[state]
        except IndexError:
            return None

    readline.set_completer(completer)


def cmd_get(client, param_map, name):
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
    try:
        frame = client.rpc_exchange(
            ROBOT_RPC_METHOD_SET_PARAM,
            entry.offset,
            entry.size,
            data,
            save_flag=save_flag,
        )
    except (TimeoutError, ConnectionError) as exc:
        print("SET failed: {}".format(exc))
        return
    try:
        method, status, offset, length, _data = parse_rpc_response(frame)
    except ValueError as exc:
        print("SET failed: {}".format(exc))
        return
    if method != ROBOT_RPC_METHOD_SET_PARAM or offset != entry.offset or length != entry.size:
        print("SET {}: unexpected response".format(name))
        return
    if status != 0:
        print("SET {}: {}".format(name, RPC_STATUS_NAMES.get(status, "ERR")))
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
