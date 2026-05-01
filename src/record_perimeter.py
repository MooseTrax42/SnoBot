"""
SnoBot Perimeter Recording Tool.

Drive the robot in MANUAL mode around the boundary of the area to plow.
The system records odometry waypoints and saves the perimeter polygon.

Usage:
    python src/record_perimeter.py                 # Real hardware
    python src/record_perimeter.py --dummy         # Simulated transport
    python src/record_perimeter.py --output DIR    # Custom output directory
    python src/record_perimeter.py --set           # Record outer + holes into one file
"""

import sys
import asyncio
import argparse
import time
import os
import platform
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

if platform.system() == "Windows":
    import msvcrt
else:
    import tty
    import termios
    import select

from sbcp.control_loop import ControlLoop
from sbcp.transport import DummyTransport, AsyncSerialTransport
from sbcp.commands import Mode, RobotMode
from sbcp.types import RobotState
from sban.perimeter import PerimeterRecorder, RecorderState, PerimeterSet

SERIAL_PORT = "COM3" if platform.system() == "Windows" else "/dev/ttyACM0"
BAUD_RATE = 115200


class PerimeterRecordingSession:
    """
    Interactive session for recording a perimeter boundary.

    Keyboard:
        ENTER  Start / stop recording
        v      View current status
        q      Quit (saves if recording)
        n      Save perimeter set (when --set is used)
    """

    def __init__(
        self,
        use_dummy: bool = False,
        port: str = SERIAL_PORT,
        baud: int = BAUD_RATE,
        output_dir: str = "data/sban/perimeters",
        sample_distance: float = 0.3,
        closure_threshold: float = 0.5,
        set_mode: bool = False,
        set_name: str | None = None,
    ):
        self.use_dummy = use_dummy
        self.port = port
        self.baud = baud
        self.output_dir = output_dir
        self.sample_distance = sample_distance
        self.closure_threshold = closure_threshold
        self.set_mode = set_mode
        self.set_name = set_name

        self.controller: ControlLoop = None  # type: ignore[assignment]
        self.recorder: PerimeterRecorder = None  # type: ignore[assignment]

        self.is_windows = platform.system() == "Windows"
        self.old_settings = None

        self._outer = None
        self._holes = []
        self._stage = "outer" if self.set_mode else "single"

    # ------------------------------------------------------------------
    # Terminal helpers (same pattern as test_sbcp_link.py)
    # ------------------------------------------------------------------

    def _kbhit(self) -> bool:
        if self.is_windows:
            return msvcrt.kbhit()
        return select.select([sys.stdin], [], [], 0)[0] != []

    def _getch(self) -> str:
        if self.is_windows:
            return msvcrt.getch().decode("utf-8", errors="replace").lower()
        return sys.stdin.read(1).lower()

    def _setup_terminal(self):
        if not self.is_windows and sys.stdin.isatty():
            try:
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
            except termios.error:
                pass

    def _restore_terminal(self):
        if not self.is_windows and self.old_settings and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except termios.error:
                pass

    # ------------------------------------------------------------------
    # Main session
    # ------------------------------------------------------------------

    async def run(self):
        # Build transport.
        if self.use_dummy:
            transport = DummyTransport(enable_telemetry=True)
            print("[Session] Using DummyTransport")
            print("[Session] WARNING: DummyTransport sends static encoder data.")
            print("          Odometry will not advance — useful for testing controls only.\n")
        else:
            transport = AsyncSerialTransport(port=self.port, baud=self.baud)
            print(f"[Session] Using serial {self.port} @ {self.baud}")

        # Start control loop.
        self.controller = ControlLoop(transport, enable_odometry=True)
        await self.controller.start()
        if not await self.controller.initialize():
            print("[Session] Initialization failed — is the Arduino connected?")
            await self.controller.stop()
            return

        # Transition to MANUAL mode.
        self.controller.set_mode(RobotMode.MANUAL)
        if not await self.controller.wait_for_state(RobotState.MANUAL, timeout=3.0):
            print(f"[Session] Warning: state is {self.controller.state_machine.state.value}, expected MANUAL")

        # Create recorder.
        self.recorder = PerimeterRecorder(
            control_loop=self.controller,
            sample_distance_m=self.sample_distance,
            closure_threshold_m=self.closure_threshold,
        )

        self._print_instructions()
        self._setup_terminal()

        try:
            while True:
                # Keyboard input.
                if self._kbhit():
                    key = self._getch()
                    should_quit = self._handle_key(key)
                    if should_quit:
                        break

                # Tick the recorder.
                if self.recorder.is_recording():
                    wp = self.recorder.update()
                    if wp is not None:
                        count = self.recorder.get_waypoint_count()
                        print(
                            f"\r  #{count}: ({wp.x:.2f}, {wp.y:.2f}) "
                            f"dist={wp.distance:.1f}m   ",
                            end="", flush=True,
                        )

                    # Check for auto loop closure.
                    if self.recorder.get_state() == RecorderState.DONE:
                        print("\n\nLoop closure detected! Recording complete.")
                        self._on_recording_done(auto=True)

                await asyncio.sleep(0.05)  # ~20 Hz

        except (KeyboardInterrupt, asyncio.CancelledError):
            print("\n[Session] Interrupted")
            if self.recorder.is_recording():
                self.recorder.stop_recording()
                self._on_recording_done(auto=False)
            if self.set_mode:
                self._finish_set()
        finally:
            self._restore_terminal()
            await self.controller.stop()

    # ------------------------------------------------------------------
    # Key handling
    # ------------------------------------------------------------------

    def _handle_key(self, key: str) -> bool:
        """Handle keyboard input. Returns True if the session should quit."""
        state = self.recorder.get_state()

        if key in ("\r", "\n"):
            if state == RecorderState.IDLE:
                if self.set_mode and self._stage == "holes":
                    print("\nStarting HOLE recording... Drive around the hole boundary.")
                    print("Press ENTER to stop. Loop closure auto-stops.\n")
                    if not self.recorder.start_recording(reset_odometry=False):
                        print("Failed to start recording (odometry not ready).")
                        return False
                else:
                    print("\nStarting recording... Drive around the boundary.")
                    print("Press ENTER to stop. Loop closure auto-stops.\n")
                    if not self.recorder.start_recording(reset_odometry=True):
                        print("Failed to start recording.")
                        return False
            elif state == RecorderState.RECORDING:
                print("\n\nStopping recording...")
                self.recorder.stop_recording()
                self._on_recording_done(auto=False)
            elif state == RecorderState.DONE:
                print("\nResetting for a new recording...")
                self.recorder.reset()

        elif key == "v":
            self._print_status()

        elif key == "n":
            if self.set_mode and state != RecorderState.RECORDING:
                return self._finish_set()

        elif key == "q":
            if state == RecorderState.RECORDING:
                print("\n\nStopping and saving before exit...")
                self.recorder.stop_recording()
                self._on_recording_done(auto=False)
            if self.set_mode:
                return self._finish_set()
            return True

        return False

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------

    def _save(self):
        perimeter = self.recorder.get_perimeter()
        if perimeter is None:
            print("No perimeter data to save.")
            return

        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"perimeter_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)

        perimeter.save(filepath)

        print(f"\nSaved: {filepath}")
        print(f"  Waypoints:      {perimeter.vertex_count}")
        print(f"  Total distance: {perimeter.total_distance_m:.1f} m")
        print(f"  Area:           {perimeter.area():.1f} m\u00b2")
        print(f"  Loop closed:    {perimeter.loop_closed}")
        print(f"  Closure error:  {perimeter.closure_error_m:.3f} m")

    def _save_set(self):
        if self._outer is None:
            print("No outer perimeter recorded. Nothing to save.")
            return False

        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = self.set_name or f"perimeter_set_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)

        perimeter_set = PerimeterSet(
            outer=self._outer,
            holes=list(self._holes),
            metadata={"hole_count": len(self._holes)},
        )
        perimeter_set.save(filepath)

        print(f"\nSaved perimeter set: {filepath}")
        print(f"  Outer vertices: {self._outer.vertex_count}")
        print(f"  Holes:          {len(self._holes)}")
        return True

    def _finish_set(self) -> bool:
        return self._save_set()

    def _on_recording_done(self, auto: bool):
        if not self.set_mode:
            self._save()
            return

        perimeter = self.recorder.get_perimeter()
        if perimeter is None:
            print("No perimeter data to capture.")
            return

        if self._stage == "outer":
            self._outer = perimeter
            print("\nOuter perimeter captured.")
            print("Drive to a hole boundary and press ENTER to record a hole.")
            print("Press 'n' to finish and save.\n")
            self._stage = "holes"
        else:
            self._holes.append(perimeter)
            print(f"\nHole captured. Total holes: {len(self._holes)}")
            print("Drive to another hole and press ENTER, or press 'n' to save.\n")

        self.recorder.reset()

    # ------------------------------------------------------------------
    # Display helpers
    # ------------------------------------------------------------------

    def _print_instructions(self):
        print()
        print("=" * 50)
        print("  SnoBot Perimeter Recording Tool")
        print("=" * 50)
        print()
        print("  ENTER   Start / stop recording")
        print("  v       View current status (pose, waypoints)")
        if self.set_mode:
            print("  n       Save perimeter set")
        print("  q       Quit")
        print()
        print(f"  Sample distance:    {self.sample_distance} m")
        print(f"  Closure threshold:  {self.closure_threshold} m")
        print(f"  Output directory:   {self.output_dir}")
        print()
        if self.set_mode:
            print("  Recording flow:")
            print("  1) Record OUTER perimeter (odometry resets once).")
            print("  2) Record HOLES without odometry reset (same frame).")
            print("  3) Press 'n' to save the set.")
        else:
            print("  Drive the robot around the boundary and press ENTER to begin.")
        print()

    def _print_status(self):
        pose = self.controller.get_pose()
        stats = self.recorder.get_stats()
        state = self.controller.state_machine.state.value
        print(f"\n  Robot state: {state}")
        if pose:
            print(f"  Pose: x={pose.x:.3f}m  y={pose.y:.3f}m  theta={pose.theta:.2f}rad")
        else:
            print("  Pose: unavailable (odometry not running)")
        print(f"  Recorder: {stats['state']}  |  waypoints={stats['waypoints']}  |  "
              f"distance={stats['cumulative_distance_m']:.1f}m  |  "
              f"elapsed={stats['elapsed_s']}s")
        print()


async def main():
    parser = argparse.ArgumentParser(description="SnoBot Perimeter Recording Tool")
    parser.add_argument("--dummy", action="store_true", help="Use simulated transport (no Arduino)")
    parser.add_argument("--port", default=SERIAL_PORT, help=f"Serial port (default: {SERIAL_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"Baud rate (default: {BAUD_RATE})")
    parser.add_argument("--output", default="data/sban/perimeters", help="Output directory")
    parser.add_argument("--sample-distance", type=float, default=0.3, help="Meters between samples (default: 0.3)")
    parser.add_argument("--closure-threshold", type=float, default=0.5, help="Loop closure distance in meters (default: 0.5)")
    parser.add_argument("--set", action="store_true", help="Record outer + holes into a single set file")
    parser.add_argument("--set-name", default=None, help="Filename for perimeter set (default: perimeter_set_YYYYMMDD_HHMMSS.json)")
    args = parser.parse_args()

    session = PerimeterRecordingSession(
        use_dummy=args.dummy,
        port=args.port,
        baud=args.baud,
        output_dir=args.output,
        sample_distance=args.sample_distance,
        closure_threshold=args.closure_threshold,
        set_mode=args.set,
        set_name=args.set_name,
    )

    try:
        await session.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nGoodbye!")


if __name__ == "__main__":
    asyncio.run(main())
