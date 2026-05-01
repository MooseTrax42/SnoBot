"""
SnoBot Autonomous Entry Point.

Wires the vision pipeline to the control loop via EventBus.

Usage:
    python src/main_auto.py              # Real hardware
    python src/main_auto.py --dummy      # Simulated transport (no Arduino)
    python src/main_auto.py --no-vision  # Control loop only (no cameras)
"""

import sys
import asyncio
import argparse
import threading
import time
from pathlib import Path

# Add project source root.
sys.path.insert(0, str(Path(__file__).parent))

from sbcp.control_loop import ControlLoop
from sbcp.transport import AsyncSerialTransport, DummyTransport
from sbcp.commands import RobotMode
from sbcp.types import RobotState
from sban.behavior.obstacle_reactor import ObstacleReactor


def vision_thread(shutdown_event: threading.Event):
    """
    Blocking vision loop.  Runs in its own thread.

    Captures stereo frames, runs depth + object detection, and emits
    detection events to the global EventBus.
    """
    from sbvs.camera.stereo_camera import StereoCamera
    from sbvs.stereo.stereo_processor import StereoProcessor
    from sbvs.object.object_processor import ObjectProcessor
    from sbvs.object.object_events import emit_detections

    stereo = StereoProcessor()
    obj_proc = ObjectProcessor(
        crop_factor=0.2,
        confidence_history_len=25,
        min_hits=5,
        use_builtin_tracking=False,
    )
    camera = StereoCamera(sync_tolerance_ms=15.0, max_queue=2, vpi_convert=True)

    prev_track_ids: set = set()
    print("[Vision] Starting camera...")

    try:
        with camera:
            for fL, fR, _tL, _tR in camera.frames():
                if shutdown_event.is_set():
                    break

                stereo_result = stereo.run(fL, fR)
                _, detections = obj_proc.process(fL, stereo_result)
                prev_track_ids = emit_detections(detections, prev_track_ids)
    except Exception as e:
        print(f"[Vision] Error: {e}")
    finally:
        camera.stop()
        print("[Vision] Stopped")


async def main(args):
    # Build transport.
    if args.dummy:
        transport = DummyTransport(enable_telemetry=True)
        print("[Main] Using DummyTransport")
    else:
        transport = AsyncSerialTransport(port=args.port, baud=args.baud)
        print(f"[Main] Using serial {args.port} @ {args.baud}")

    ctrl = ControlLoop(transport)

    # Start control loop and initialize handshake.
    await ctrl.start()
    if not await ctrl.initialize():
        print("[Main] Initialization failed — is the Arduino connected?")
        await ctrl.stop()
        return

    # Transition to AUTO mode.
    ctrl.set_mode(RobotMode.AUTO)
    if not await ctrl.wait_for_state(RobotState.AUTO, timeout=3.0):
        # Non-fatal: the robot may already be in AUTO from a previous session.
        print(f"[Main] Warning: state is {ctrl.state_machine.state.value}, expected AUTO")

    # Start obstacle reactor.
    reactor = ObstacleReactor(ctrl)
    reactor_task = asyncio.create_task(reactor.run())

    # Start vision in a background thread (unless disabled).
    shutdown = threading.Event()
    vis_thread = None
    if not args.no_vision:
        vis_thread = threading.Thread(target=vision_thread, args=(shutdown,), daemon=True)
        vis_thread.start()
        print("[Main] Vision thread started")
    else:
        print("[Main] Vision disabled (--no-vision)")

    print("[Main] Running — press Ctrl+C to stop")

    try:
        while True:
            await asyncio.sleep(1.0)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\n[Main] Shutting down...")

    # Clean shutdown.
    shutdown.set()
    reactor.stop()
    try:
        await reactor_task
    except asyncio.CancelledError:
        pass

    ctrl.stop_motion()
    ctrl.set_auger(False)
    await asyncio.sleep(0.3)  # Let final stop intent propagate.
    await ctrl.stop()

    if vis_thread is not None:
        vis_thread.join(timeout=3.0)

    print("[Main] Done")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SnoBot Autonomous Runner")
    parser.add_argument("--dummy", action="store_true", help="Use simulated transport (no Arduino)")
    parser.add_argument("--no-vision", action="store_true", help="Disable camera / vision pipeline")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    asyncio.run(main(args))
