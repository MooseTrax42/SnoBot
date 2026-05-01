"""
Standalone single-encoder test for Arduino serial and SBCP link.

Usage examples:
  python src/tests/test_single_encoder.py --mode arduino --port COM12 --encoder left
  python src/tests/test_single_encoder.py --mode sbcp --port COM12 --encoder right
  python src/tests/test_single_encoder.py --mode both --port COM12 --encoder left

Pass condition:
  Selected encoder changes by at least --min-delta ticks within --timeout seconds.
"""

import argparse
import asyncio
import json
import platform
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import serial

# Add project path.
sys.path.insert(0, str(Path(__file__).parent.parent))

from sbcp.control_loop import ControlLoop
from sbcp.transport import AsyncSerialTransport


DEFAULT_PORT = "COM12" if platform.system() == "Windows" else "/dev/ttyACM0"
DEFAULT_BAUD = 115200


def _extract_encoders_from_message(msg: dict) -> Optional[Tuple[int, int]]:
    """Extract (left, right) encoder counts from supported telemetry shapes."""
    payload = msg.get("data") if isinstance(msg, dict) else None
    if not isinstance(payload, dict):
        payload = msg if isinstance(msg, dict) else {}

    enc = payload.get("enc")
    if isinstance(enc, list) and len(enc) >= 2:
        return int(enc[0]), int(enc[1])

    left = payload.get("encoder_left")
    right = payload.get("encoder_right")
    if left is not None and right is not None:
        return int(left), int(right)

    return None


def _print_sample(tag: str, left: int, right: int, prev_left: Optional[int], prev_right: Optional[int]):
    dl = 0 if prev_left is None else left - prev_left
    dr = 0 if prev_right is None else right - prev_right
    print(f"[{tag}] L={left:8d} ({dl:+5d})  R={right:8d} ({dr:+5d})")


def run_arduino_serial_test(
    port: str,
    baud: int,
    encoder: str,
    min_delta: int,
    timeout_s: float,
    read_timeout_s: float,
) -> bool:
    print("\n=== Direct Arduino Serial Test ===")
    print(f"Port={port} Baud={baud} Encoder={encoder} MinDelta={min_delta} Timeout={timeout_s}s")

    baseline = None
    prev_left = None
    prev_right = None
    deadline = time.monotonic() + timeout_s

    with serial.Serial(port=port, baudrate=baud, timeout=read_timeout_s, write_timeout=0.2) as ser:
        time.sleep(0.8)
        ser.reset_input_buffer()

        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue

            try:
                text = raw.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                msg = json.loads(text)
            except Exception:
                continue

            enc = _extract_encoders_from_message(msg)
            if not enc:
                continue

            left, right = enc
            _print_sample("ARD", left, right, prev_left, prev_right)
            prev_left, prev_right = left, right

            current = left if encoder == "left" else right
            if baseline is None:
                baseline = current
                print(f"[ARD] Baseline {encoder}={baseline}")
                continue

            moved = abs(current - baseline)
            if moved >= min_delta:
                print(f"[ARD] PASS: {encoder} moved {moved} ticks (>= {min_delta})")
                return True

    print(f"[ARD] FAIL: {encoder} did not move by {min_delta} ticks before timeout")
    return False


async def run_sbcp_link_test(
    port: str,
    baud: int,
    encoder: str,
    min_delta: int,
    timeout_s: float,
) -> bool:
    print("\n=== SBCP Link Test ===")
    print(f"Port={port} Baud={baud} Encoder={encoder} MinDelta={min_delta} Timeout={timeout_s}s")

    transport = AsyncSerialTransport(
        port=port,
        baud=baud,
        timeout=0.05,
        read_mode="auto",
        use_rx_thread=(platform.system() != "Windows"),
    )
    control = ControlLoop(
        transport,
        publish_rate_hz=10.0,
        subscribe_rate_hz=12.0,
        enable_odometry=False,
    )

    baseline = None
    prev_left = None
    prev_right = None
    deadline = time.monotonic() + timeout_s

    try:
        await control.start()
        ok = await control.initialize()
        if not ok:
            print("[SBCP] FAIL: initialize() failed")
            return False

        while time.monotonic() < deadline:
            enc = control.get_encoder_data()
            if enc is None:
                await asyncio.sleep(0.05)
                continue

            left = int(enc["left"])
            right = int(enc["right"])
            _print_sample("SBCP", left, right, prev_left, prev_right)
            prev_left, prev_right = left, right

            current = left if encoder == "left" else right
            if baseline is None:
                baseline = current
                print(f"[SBCP] Baseline {encoder}={baseline}")
                await asyncio.sleep(0.05)
                continue

            moved = abs(current - baseline)
            if moved >= min_delta:
                print(f"[SBCP] PASS: {encoder} moved {moved} ticks (>= {min_delta})")
                return True

            await asyncio.sleep(0.05)

        print(f"[SBCP] FAIL: {encoder} did not move by {min_delta} ticks before timeout")
        return False
    finally:
        await control.stop()


async def run(args: argparse.Namespace) -> int:
    results = []

    if args.mode in ("arduino", "both"):
        arduino_ok = run_arduino_serial_test(
            port=args.port,
            baud=args.baud,
            encoder=args.encoder,
            min_delta=args.min_delta,
            timeout_s=args.timeout,
            read_timeout_s=args.read_timeout,
        )
        results.append(("arduino", arduino_ok))

    if args.mode in ("sbcp", "both"):
        sbcp_ok = await run_sbcp_link_test(
            port=args.port,
            baud=args.baud,
            encoder=args.encoder,
            min_delta=args.min_delta,
            timeout_s=args.timeout,
        )
        results.append(("sbcp", sbcp_ok))

    print("\n=== Summary ===")
    all_ok = True
    for name, ok in results:
        print(f"{name:8s}: {'PASS' if ok else 'FAIL'}")
        all_ok = all_ok and ok

    return 0 if all_ok else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone single-encoder test")
    parser.add_argument("--mode", choices=["arduino", "sbcp", "both"], default="both")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port (e.g., COM12 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--encoder", choices=["left", "right"], default="left")
    parser.add_argument("--min-delta", type=int, default=20, help="Required tick change to pass")
    parser.add_argument("--timeout", type=float, default=15.0, help="Seconds to wait for movement")
    parser.add_argument("--read-timeout", type=float, default=0.2, help="Serial read timeout for Arduino mode")
    return parser.parse_args()


if __name__ == "__main__":
    cli_args = parse_args()
    try:
        raise SystemExit(asyncio.run(run(cli_args)))
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        raise SystemExit(130)
