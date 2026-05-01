"""
SnoBot Remote Control Entry Point.

Starts the ControlLoop in MANUAL mode and launches a Bluetooth RFCOMM
server for an ESP32-based handheld controller.

Usage:
    python src/main_remote.py                     # Real hardware
    python src/main_remote.py --dummy             # Simulated transport
    python src/main_remote.py --bt-channel 2      # Custom RFCOMM channel
"""

import sys
import asyncio
import argparse
import platform
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from sbcp.control_loop import ControlLoop
from sbcp.transport import AsyncSerialTransport, DummyTransport
from sbcp.commands import RobotMode
from sbcp.types import RobotState
from sban.perimeter import PerimeterRecorder
from sbrc.api import ControllerAPI
from sbrc.server import BluetoothServer

SERIAL_PORT = "COM3" if platform.system() == "Windows" else "/dev/ttyACM0"
BAUD_RATE = 115200
BT_CHANNEL_DEFAULT = 4 if platform.system() == "Windows" else 1


async def main(args):
    # Build transport.
    if args.dummy:
        transport = DummyTransport(enable_telemetry=True)
        print("[Main] Using DummyTransport")
    else:
        transport = AsyncSerialTransport(port=args.port, baud=args.baud)
        print(f"[Main] Using serial {args.port} @ {args.baud}")

    ctrl = ControlLoop(transport, enable_odometry=True)

    # Start control loop and initialise.
    await ctrl.start()
    if not await ctrl.initialize():
        print("[Main] Initialization failed — is the Arduino connected?")
        await ctrl.stop()
        return

    # Transition to MANUAL mode.
    ctrl.set_mode(RobotMode.MANUAL)
    if not await ctrl.wait_for_state(RobotState.MANUAL, timeout=3.0):
        print(f"[Main] Warning: state is {ctrl.state_machine.state.value}, expected MANUAL")

    # Create perimeter recorder.
    recorder = PerimeterRecorder(control_loop=ctrl)

    # Create API + Bluetooth server.
    api = ControllerAPI(ctrl, recorder=recorder)
    bt = BluetoothServer(
        api,
        channel=args.bt_channel,
        telemetry_hz=args.telemetry_hz,
        disconnect_watchdog_s=args.bt_watchdog_s,
        log_controller_inputs=args.show_inputs,
        log_vel_every_s=max(0.0, args.show_inputs_vel_ms / 1000.0),
    )
    await bt.start()

    print()
    print("=" * 50)
    print("  SnoBot Remote Control Server")
    print("=" * 50)
    print(f"  Bluetooth RFCOMM channel: {args.bt_channel}")
    print(f"  Telemetry push rate:      {args.telemetry_hz} Hz")
    print(f"  RX watchdog timeout:      {args.bt_watchdog_s:.1f} s")
    print(f"  Input trace:              {'on' if args.show_inputs else 'off'}")
    print(f"  Perimeter output:         {args.output}")
    print()
    print("  Waiting for ESP32 controller connection...")
    print("  Press Ctrl+C to stop.")
    print()

    try:
        while True:
            # Tick perimeter recorder if it is actively recording.
            if recorder.is_recording():
                recorder.update()
            await asyncio.sleep(0.05)  # 20 Hz
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\n[Main] Shutting down...")

    # Auto-save perimeter if one was recorded.
    perimeter = recorder.get_perimeter()
    if perimeter is not None:
        import os, time as _t
        os.makedirs(args.output, exist_ok=True)
        ts = _t.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(args.output, f"perimeter_{ts}.json")
        perimeter.save(path)
        print(f"[Main] Perimeter saved: {path}")

    # Clean shutdown.
    await bt.stop()
    ctrl.stop_motion()
    ctrl.set_auger(False)
    ctrl.set_salt(False)
    await asyncio.sleep(0.3)
    await ctrl.stop()
    print("[Main] Done")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SnoBot Remote Control Server")
    parser.add_argument("--dummy", action="store_true", help="Use simulated transport")
    parser.add_argument("--port", default=SERIAL_PORT, help=f"Serial port (default: {SERIAL_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Baud rate")
    parser.add_argument(
        "--bt-channel",
        type=int,
        default=BT_CHANNEL_DEFAULT,
        help=f"RFCOMM channel (default: {BT_CHANNEL_DEFAULT})",
    )
    parser.add_argument("--telemetry-hz", type=float, default=5.0, help="Telemetry push rate (default: 5)")
    parser.add_argument(
        "--bt-watchdog-s",
        type=float,
        default=6.0,
        help="Disconnect timeout when controller RX is silent (default: 6.0)",
    )
    parser.add_argument("--show-inputs", action="store_true", help="Print controller commands received over BT")
    parser.add_argument(
        "--show-inputs-vel-ms",
        type=int,
        default=250,
        help="Velocity print interval in ms when --show-inputs is on (default: 250)",
    )
    parser.add_argument("--output", default="data/sban/perimeters", help="Perimeter output directory")
    args = parser.parse_args()

    asyncio.run(main(args))
