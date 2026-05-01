"""
Serial load test for SBCP.

Configurable rates for HELLO/STATUS/INTENT to reproduce traffic patterns.
"""

import sys
from pathlib import Path

# Add project paths.
sys.path.insert(0, str(Path(__file__).parent.parent))

import argparse
import json
import time
import platform
from typing import Dict, Optional

import serial
import threading

DEFAULT_PORT = "COM3" if platform.system() == "Windows" else "/dev/ttyACM0"
DEFAULT_BAUD = 230400
PROTOCOL_VERSION = "0.3.0"


def make_cmd(cmd: str, seq: int, args: Optional[dict] = None) -> dict:
    payload = {"cmd": cmd, "args": args or {}}
    return {"type": "CMD", "ver": PROTOCOL_VERSION, "seq": seq, "data": payload}


def make_intent(seq: int, ts_ms: int, fields: Dict[str, object], pad_len: int) -> dict:
    payload = {"ts": ts_ms}
    payload.update(fields)
    if pad_len > 0:
        payload["pad"] = "x" * pad_len
    return {"type": "INTENT", "ver": PROTOCOL_VERSION, "seq": seq, "ts": ts_ms, "data": payload}


def encode_line(msg: dict) -> bytes:
    return (json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8")


def clamp_pad_to_size(msg: dict, pad_len: int, max_bytes: int) -> dict:
    if pad_len <= 0:
        return msg
    line = encode_line(msg)
    if len(line) <= max_bytes:
        return msg

    # Shrink the pad field to fit.
    overflow = len(line) - max_bytes
    new_pad = max(0, pad_len - overflow)
    msg["data"]["pad"] = "x" * new_pad
    return msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SBCP serial load test")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--write-timeout", type=float, default=0.5)
    parser.add_argument("--hello-hz", type=float, default=1.0)
    parser.add_argument("--status-hz", type=float, default=1.0)
    parser.add_argument("--intent-hz", type=float, default=20.0)
    parser.add_argument("--intent", action="store_true", help="Enable INTENT traffic")
    parser.add_argument("--intent-fields", default="v,w,auger,salt,chute,headlight",
                        help="Comma list: v,w,auger,salt,chute,headlight,status")
    parser.add_argument("--toggle-hz", type=float, default=1.0,
                        help="Toggle intent values at this rate (0 disables)")
    parser.add_argument("--pad-bytes", type=int, default=0)
    parser.add_argument("--max-line-bytes", type=int, default=240)
    parser.add_argument("--mode", choices=["", "IDLE", "MANUAL", "AUTO"], default="",
                        help="Send MODE once after startup")
    parser.add_argument("--print-every", type=float, default=5.0)
    parser.add_argument("--quiet", action="store_true", help="Do not print every RX line")
    parser.add_argument("--reader", choices=["line", "chunk"], default="line",
                        help="Read mode: line=readline, chunk=read raw chunks")
    parser.add_argument("--reader-thread", action="store_true",
                        help="Run reader in background thread")
    return parser.parse_args()


def build_intent_fields(field_names, toggle_state: bool) -> Dict[str, object]:
    fields: Dict[str, object] = {}
    if "v" in field_names:
        fields["v"] = 0.5 if toggle_state else -0.5
    if "w" in field_names:
        fields["w"] = 0.4 if toggle_state else -0.4
    if "auger" in field_names:
        fields["auger_en"] = toggle_state
    if "salt" in field_names:
        fields["salt_en"] = toggle_state
    if "chute" in field_names:
        fields["chute_angle"] = 45 if toggle_state else 135
    if "headlight" in field_names:
        fields["headlight"] = toggle_state
    if "status" in field_names:
        fields["status_light"] = "ON" if toggle_state else "OFF"
    return fields


def main() -> int:
    args = parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=args.timeout,
        write_timeout=args.write_timeout,
    )
    time.sleep(1.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    seq = 0
    now = time.monotonic()
    next_hello = now
    next_status = now
    next_intent = now
    next_toggle = now
    last_stats = now
    toggle_state = False

    sent_hello = 0
    sent_status = 0
    sent_intent = 0
    rx_lines = 0
    rx_acks = 0
    rx_stats = 0
    rx_errors = 0
    empty_reads = 0

    field_names = {f.strip().lower() for f in args.intent_fields.split(",") if f.strip()}
    stop_event = threading.Event()
    rx_buffer = b""

    def send_line(msg: dict, label: str):
        nonlocal rx_errors
        try:
            ser.write(encode_line(msg))
            return True
        except Exception as e:
            rx_errors += 1
            print(f"[send {label}] error: {e}")
            return False

    def handle_rx_line(text: str):
        nonlocal rx_lines, rx_acks, rx_stats
        rx_lines += 1
        if not args.quiet:
            print(text)
        try:
            msg = json.loads(text)
            if msg.get("type") == "ACK":
                rx_acks += 1
            elif msg.get("type") in ("STAT", "S"):
                rx_stats += 1
        except Exception:
            pass

    def read_once_line():
        nonlocal empty_reads
        line = ser.readline()
        if line:
            try:
                text = line.decode("utf-8", errors="replace").strip()
            except Exception:
                text = str(line)
            if text:
                handle_rx_line(text)
        else:
            empty_reads += 1

    def read_once_chunk():
        nonlocal rx_buffer, empty_reads
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            empty_reads += 1
            time.sleep(0.001)
            return
        rx_buffer += chunk
        while b"\n" in rx_buffer:
            line, rx_buffer = rx_buffer.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                text = line.decode("utf-8", errors="replace").strip()
            except Exception:
                text = str(line)
            if text:
                handle_rx_line(text)

    def reader_loop():
        while not stop_event.is_set():
            if args.reader == "chunk":
                read_once_chunk()
            else:
                read_once_line()

    if args.mode:
        mode_cmd = make_cmd("MODE", seq, {"mode": args.mode})
        seq = (seq + 1) % 65536
        send_line(mode_cmd, "MODE")

    try:
        reader_thread = None
        if args.reader_thread:
            reader_thread = threading.Thread(target=reader_loop, daemon=True)
            reader_thread.start()

        while True:
            now = time.monotonic()

            if args.hello_hz > 0 and now >= next_hello:
                hello = make_cmd("HELLO", seq, {"version": PROTOCOL_VERSION})
                seq = (seq + 1) % 65536
                if send_line(hello, "HELLO"):
                    sent_hello += 1
                next_hello = now + (1.0 / args.hello_hz)

            if args.status_hz > 0 and now >= next_status:
                status = make_cmd("STATUS", seq)
                seq = (seq + 1) % 65536
                if send_line(status, "STATUS"):
                    sent_status += 1
                next_status = now + (1.0 / args.status_hz)

            if args.intent and args.intent_hz > 0 and now >= next_intent:
                if args.toggle_hz > 0 and now >= next_toggle:
                    toggle_state = not toggle_state
                    next_toggle = now + (1.0 / args.toggle_hz)
                fields = build_intent_fields(field_names, toggle_state)
                ts_ms = int(time.time() * 1000)
                intent = make_intent(seq, ts_ms, fields, args.pad_bytes)
                intent = clamp_pad_to_size(intent, args.pad_bytes, args.max_line_bytes)
                seq = (seq + 1) % 65536
                if send_line(intent, "INTENT"):
                    sent_intent += 1
                next_intent = now + (1.0 / args.intent_hz)

            if not args.reader_thread:
                if args.reader == "chunk":
                    read_once_chunk()
                else:
                    read_once_line()
            else:
                time.sleep(0.001)

            if args.print_every > 0 and now - last_stats >= args.print_every:
                print(
                    f"[stats] sent: HELLO={sent_hello} STATUS={sent_status} "
                    f"INTENT={sent_intent} rx: lines={rx_lines} ACK={rx_acks} "
                    f"STAT={rx_stats} errors={rx_errors} empty_reads={empty_reads}"
                )
                last_stats = now
    except KeyboardInterrupt:
        return 0
    finally:
        stop_event.set()
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
