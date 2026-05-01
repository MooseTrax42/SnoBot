"""
SBCP (SnoBot Communication Protocol) v0.3.0
Interactive test using the ControlLoop and unified envelope system.
Supports both real serial and dummy transport for testing.
"""

import asyncio
import os
import sys
import time
import platform
from typing import Optional

from pathlib import Path

# Add project paths.
sys.path.insert(0, str(Path(__file__).parent.parent))

if platform.system() == "Windows":
    import msvcrt
else:
    import tty
    import termios
    import select

from sbcp.control_loop import ControlLoop
from sbcp.transport import DummyTransport, AsyncSerialTransport
from sbcp.commands import Hello, Ping, Status, Mode, Stop, Resume, ResetFaults, Shutdown, RobotMode
from sbcp.types import PROTOCOL_VERSION, robot_state_from_id

SERIAL_PORT = "COM12" if platform.system() == "Windows" else "/dev/ttyACM0"
BAUD_RATE = 115200

class SBCPTester:
    """
    Interactive test using ControlLoop.
    Commands (CMD) wait for ACK when required.
    Intents are sent continuously by the control loop.
    """
    def __init__(
        self,
        port: str,
        baud: int,
        use_dummy: bool = False,
        diagnostic: bool = False,
        auto_reconnect: bool = True,
        read_mode: str = "auto",
        use_rx_thread: Optional[bool] = None,
        diagnostic_reconnect: bool = True,
        log_path: Optional[str] = None,
        log_interval_s: float = 10.0,
    ):
        self.port = port
        self.baud = baud
        self.use_dummy = use_dummy
        self.diagnostic = diagnostic
        self.auto_reconnect = auto_reconnect
        self.read_mode = read_mode
        self.use_rx_thread = use_rx_thread
        self.diagnostic_reconnect = diagnostic_reconnect
        self.log_path = log_path
        self.log_interval_s = log_interval_s
        self._last_log_time = 0.0
        self._log_file = None
        self.is_windows = platform.system() == "Windows"
        self.old_settings = None

        self.transport = None
        self.controller: Optional[ControlLoop] = None

        # Current intent state (sent continuously)
        self.current_v = 0.0
        self.current_w = 0.0
        self.auger_enabled = False
        self.salt_enabled = False
        self.chute_angle = 90.0
        self.headlight_on = False

        # Robot state from telemetry
        self.robot_state = "UNKNOWN"
        self.battery_v = 0.0
        self.temp_c = 0.0
        self.estop_active = False
        self.mcu_runtime_ms = 0
        self.telemetry_faults = []
        self.active_faults = []
        self.imu_roll_deg = None
        self.imu_pitch_deg = None
        self.imu_yaw_deg = None
        self.imu_accel_x = None
        self.imu_accel_y = None
        self.imu_accel_z = None
        self.imu_gyro_x = None
        self.imu_gyro_y = None
        self.imu_gyro_z = None
        self.imu_valid = False
        self.imu_status = None
        self.imu_self_test = None
        self.imu_error = None
        self.imu_cal_sys = None
        self.imu_cal_gyro = None
        self.imu_cal_accel = None
        self.imu_cal_mag = None
        self.odom_pose = None
        self.odom_heading_deg = None
        self.odom_distance_m = None
        self.odom_update_count = None
        self.odom_m_per_tick = None
        self._last_host_rx_bytes_total = None
        self._last_host_msgs_received = None
        self._last_serial_open = None
        self._last_telem_stalled = False

    def _ensure_log_file(self):
        if not self.log_path or self._log_file:
            return
        log_dir = os.path.dirname(self.log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self._log_file = open(self.log_path, "a", encoding="utf-8")

    async def connect(self) -> bool:
        """Connect to Arduino (real or dummy) and start control loop."""
        try:
            if self.use_dummy:
                print("Using DUMMY transport (simulated Arduino)...")
                self.transport = DummyTransport(enable_telemetry=True)
            else:
                print(f"Using SERIAL transport on {self.port} at {self.baud} baud...")
                self.transport = AsyncSerialTransport(
                    port=self.port,
                    baud=self.baud,
                    timeout=0.1,
                    debug=self.diagnostic,
                    read_mode=self.read_mode,
                    use_rx_thread=(self.use_rx_thread if self.use_rx_thread is not None else (platform.system() != "Windows"))
                )

            self.controller = ControlLoop(
                self.transport,
                auto_reconnect=self.auto_reconnect,
                diagnostic_reconnect=self.diagnostic_reconnect,
                enable_odometry=True
            )
            await self.controller.start()

            self._ensure_log_file()

            print("Initializing...")
            ok = await self.controller.initialize()
            if not ok:
                print("ERROR Initialization failed")
                return False

            print("OK Controller initialized")
            return True

        except Exception as e:
            print(f"ERROR Connection failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    async def send_shutdown(self):
        if not self.controller:
            return
        try:
            await self.controller.send_command(Shutdown())
        except Exception:
            pass

    async def disconnect(self):
        """Stop controller and disconnect transport."""
        if self.controller:
            await self.send_shutdown()
            await self.controller.stop()
        print("Disconnected.")
        if self._log_file:
            self._log_file.close()
            self._log_file = None

    async def send_command(self, cmd):
        """Send a command using the control loop."""
        if not self.controller:
            return None
        try:
            await self.controller.send_command(cmd)
            return {"ok": True}
        except Exception as e:
            print(f"ERROR command failed: {e}")
            return None

    def update_state_from_telemetry(self):
        """Pull latest telemetry from controller and update cached fields."""
        if not self.controller:
            return
        latest = self.controller.subscriber.get_latest()
        if not latest:
            return
        data = latest.get("data")
        if not data:
            return
        msg_type = latest.get("type")
        if msg_type in ("STAT", "S"):
            if "ts" in latest:
                self.mcu_runtime_ms = latest.get("ts", self.mcu_runtime_ms)
            if msg_type == "S":
                state_id = data.get("st")
                state_enum = robot_state_from_id(state_id)
                if state_enum is not None:
                    self.robot_state = state_enum.value
                self.telemetry_faults = data.get("f", self.telemetry_faults) or []
                if "b" in data:
                    self.battery_v = data.get("b", 0) / 100.0
                if "tp" in data:
                    self.temp_c = data.get("tp", 0) / 10.0
                self.estop_active = bool(data.get("e", 0))
                imu = data.get("imu") or []
                if isinstance(imu, list) and len(imu) >= 9:
                    self.imu_roll_deg = imu[0] / 100.0
                    self.imu_pitch_deg = imu[1] / 100.0
                    self.imu_yaw_deg = imu[2] / 100.0
                    self.imu_accel_x = imu[3] / 1000.0
                    self.imu_accel_y = imu[4] / 1000.0
                    self.imu_accel_z = imu[5] / 1000.0
                    self.imu_gyro_x = imu[6] / 100.0
                    self.imu_gyro_y = imu[7] / 100.0
                    self.imu_gyro_z = imu[8] / 100.0
                    self.imu_valid = bool(data.get("v", 0))
                imu_stat = data.get("is") or []
                if isinstance(imu_stat, list) and len(imu_stat) >= 3:
                    self.imu_status = int(imu_stat[0])
                    self.imu_self_test = int(imu_stat[1])
                    self.imu_error = int(imu_stat[2])
                imu_cal = data.get("ic") or []
                if isinstance(imu_cal, list) and len(imu_cal) >= 4:
                    self.imu_cal_sys = int(imu_cal[0])
                    self.imu_cal_gyro = int(imu_cal[1])
                    self.imu_cal_accel = int(imu_cal[2])
                    self.imu_cal_mag = int(imu_cal[3])
            else:
                self.robot_state = data.get("state", "UNKNOWN")
                self.telemetry_faults = data.get("faults", self.telemetry_faults) or []
                if "battery_v" in data:
                    self.battery_v = data.get("battery_v", self.battery_v)
                if "temp" in data:
                    self.temp_c = data.get("temp", self.temp_c)
                self.estop_active = bool(data.get("estop", False))
                imu_yaw = data.get("imu_yaw")
                if imu_yaw is not None:
                    self.imu_roll_deg = None
                    self.imu_pitch_deg = None
                    self.imu_accel_x = None
                    self.imu_accel_y = None
                    self.imu_accel_z = None
                    self.imu_gyro_x = None
                    self.imu_gyro_y = None
                    self.imu_gyro_z = None
                    self.imu_yaw_deg = float(imu_yaw)
                    self.imu_valid = bool(data.get("imu_valid", self.imu_valid))
            if self.robot_state in ("STOPPED", "FAULT", "ESTOPPED"):
                self.current_v = 0.0
                self.current_w = 0.0
                self.auger_enabled = False
                self.salt_enabled = False
                self.headlight_on = False
        if self.controller:
            self.active_faults = sorted(self.controller.state_machine.active_faults)
            if self.controller.odometry is not None:
                pose = self.controller.get_pose()
                stats = self.controller.get_odometry_stats() or {}
                self.odom_pose = pose
                self.odom_heading_deg = self.controller.odometry.get_heading_degrees()
                self.odom_distance_m = stats.get("total_distance_m")
                self.odom_update_count = stats.get("update_count")
                self.odom_m_per_tick = stats.get("meters_per_tick")
            else:
                self.odom_pose = None
                self.odom_heading_deg = None
                self.odom_distance_m = None
                self.odom_update_count = None
                self.odom_m_per_tick = None

    def _format_stats_lines(self):
        lines = []
        if not self.controller:
            return lines
        sub_stats = self.controller.subscriber.get_stats()
        lines.append("Subscriber Stats:")
        lines.append(f"  Messages received: {sub_stats['messages_received']}")
        lines.append(f"  Latest age: {sub_stats['latest_age_s']:.3f}s")
        lines.append(f"  MCU runtime: {self.mcu_runtime_ms} ms")
        lines.append(f"  Telemetry faults: {self.telemetry_faults}")
        lines.append(f"  Active faults: {self.active_faults}")
        if not self.use_dummy:
            try:
                tstats = self.transport.get_stats()
            except Exception as e:
                lines.append(f"Transport Stats: ERROR ({e})")
                return lines
            serial_open = tstats.get("serial_open")
            if self._last_serial_open is None:
                self._last_serial_open = serial_open
            elif serial_open != self._last_serial_open:
                print(f"[Transport] serial_open changed: {self._last_serial_open} -> {serial_open}")
                self._last_serial_open = serial_open
            lines.append("Transport Stats:")
            lines.append(f"  Messages received: {tstats['messages_received']}")
            if "messages_sent" in tstats:
                lines.append(f"  Messages sent: {tstats['messages_sent']}")
            if self._last_host_msgs_received is None:
                msg_delta = 0
            else:
                msg_delta = tstats['messages_received'] - self._last_host_msgs_received
            self._last_host_msgs_received = tstats['messages_received']
            lines.append(f"  Messages delta: {msg_delta}")
            if "last_tx_age_s" in tstats:
                lines.append(f"  Last TX age: {tstats['last_tx_age_s']:.3f}s")
            if "tx_bytes_total" in tstats:
                lines.append(f"  TX bytes total: {tstats['tx_bytes_total']}")
            if "write_failures" in tstats:
                lines.append(f"  Write failures: {tstats['write_failures']}")
            lines.append(f"  Parse errors: {tstats['parse_errors']}")
            lines.append(f"  Last error time: {tstats['last_error_time']}")
            lines.append(f"  RX queue len: {tstats['rx_queue_len']}")
            lines.append(f"  Serial open: {tstats['serial_open']}")
            lines.append(f"  RX thread alive: {tstats['rx_thread_alive']}")
            lines.append(f"  Last RX age: {tstats['last_rx_age_s']:.3f}s")
            lines.append(f"  Last byte age: {tstats['last_byte_age_s']:.3f}s")
            if "last_in_waiting_nonzero_age_s" in tstats:
                lines.append(f"  Last in_waiting>0 age: {tstats['last_in_waiting_nonzero_age_s']:.3f}s")
            if "rx_bytes_total" in tstats:
                rx_bytes_total = tstats["rx_bytes_total"]
                if self._last_host_rx_bytes_total is None:
                    rx_bytes_delta = 0
                else:
                    rx_bytes_delta = rx_bytes_total - self._last_host_rx_bytes_total
                self._last_host_rx_bytes_total = rx_bytes_total
                lines.append(f"  RX bytes total: {rx_bytes_total}")
                lines.append(f"  RX bytes delta: {rx_bytes_delta}")
            if "rx_buffer_len" in tstats:
                lines.append(f"  RX buffer len: {tstats['rx_buffer_len']}")
            if "read_mode" in tstats:
                lines.append(f"  Read mode: {tstats['read_mode']}")
            if "in_waiting" in tstats:
                lines.append(f"  In waiting: {tstats['in_waiting']}")
            if "out_waiting" in tstats:
                lines.append(f"  Out waiting: {tstats['out_waiting']}")
            if "empty_reads" in tstats:
                lines.append(f"  Empty reads: {tstats['empty_reads']}")
            if "last_serial_error" in tstats:
                lines.append(f"  Last serial error: {tstats['last_serial_error']}")
            if "last_serial_error_time" in tstats:
                lines.append(f"  Last serial error time: {tstats['last_serial_error_time']}")
            if "last_decode_error" in tstats and tstats["last_decode_error"]:
                lines.append(f"  Last decode error: {tstats['last_decode_error']}")
            if "last_decode_error_time" in tstats and tstats["last_decode_error_time"]:
                lines.append(f"  Last decode error time: {tstats['last_decode_error_time']:.3f}")
            if "port_present" in tstats:
                lines.append(f"  Port present: {tstats['port_present']}")
            if "use_rx_thread" in tstats:
                lines.append(f"  Use RX thread: {tstats['use_rx_thread']}")
            if "cts" in tstats or "dsr" in tstats:
                lines.append(
                    f"  Signals: CTS={tstats.get('cts')} DSR={tstats.get('dsr')} "
                    f"RI={tstats.get('ri')} CD={tstats.get('cd')} "
                    f"DTR={tstats.get('dtr')} RTS={tstats.get('rts')}"
                )
            if "consecutive_errors" in tstats:
                lines.append(f"  Consecutive errors: {tstats['consecutive_errors']}")
            if "rx_thread_restarts" in tstats:
                lines.append(f"  RX thread restarts: {tstats['rx_thread_restarts']}")
            if "last_thread_death_reason" in tstats and tstats['last_thread_death_reason']:
                lines.append(f"  Last death reason: {tstats['last_thread_death_reason']}")
        return lines

    def _print_stats(self):
        for line in self._format_stats_lines():
            print(line)

    def _write_stats_log(self):
        if not self._log_file:
            return
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        self._log_file.write(f"[{ts}]\n")
        for line in self._format_stats_lines():
            self._log_file.write(line + "\n")
        self._log_file.flush()

    def print_menu(self):
        """Print command menu."""
        print("" + "=" * 60)
        print("SBCP v0.3.0 - CONTROL LOOP TEST")
        print(f"Transport: {'DUMMY (Simulated)' if self.use_dummy else 'SERIAL (Real Arduino)'}")
        print("=" * 60)
        print("COMMANDS (wait for ACK when required):")
        print("  h - HELLO handshake")
        print("  p - PING")
        print("  s - STATUS request")
        print("  i - Set IDLE mode")
        print("  m - Set MANUAL mode")
        print("  n - Set AUTO mode")
        print("  o - STOP")
        print("  r - RESUME")
        print("  e - EMERGENCY STOP (intent)")
        print("  f - RESET_FAULTS")
        print("  g - SHUTDOWN")

        print("INTENTS (sent continuously by ControlLoop):")
        print("  w - Drive FORWARD (v=0.5)")
        print("  x - Drive BACKWARD (v=-0.5)")
        print("  a - Turn LEFT (w=1.0)")
        print("  z - Turn RIGHT (w=-1.0)")
        print("  SPACE - STOP (v=0, w=0)")

        print("ACTUATORS (part of intent):")
        print("  1 - Auger ON")
        print("  2 - Auger OFF")
        print("  3 - Salt ON")
        print("  4 - Salt OFF")
        print("  5 - Chute LEFT (45 deg)")
        print("  6 - Chute CENTER (90 deg)")
        print("  7 - Chute RIGHT (135 deg)")
        print("  [ - Headlight ON")
        print("  ] - Headlight OFF")

        print("INFO:")
        print("  v - View current state (incl IMU)")
        print("  b - Battery info")
        print("  t - Subscriber stats")
        print("  y - Odometry info")
        if self.use_dummy:
            print("  d - Dummy transport stats")
        print("  ? - Show this menu")
        print("  q - Quit")
        print("=" * 60)
        print("KEY CONCEPT:")
        print("  Commands (HELLO, MODE, etc) -> Wait for ACK when required")
        print("  Intents (motion, actuators) -> Sent at 20Hz by ControlLoop")
        print("=" * 60)

    def print_state_info(self):
        """Print current state information."""
        print("" + "-" * 40)
        print("CURRENT STATE:")
        print(f"  Robot state: {self.robot_state}")
        print(f"  E-stop active: {self.estop_active}")
        print(f"  Battery: {self.battery_v:.1f} V")
        print(f"  Temperature: {self.temp_c:.1f} C")
        print(f"  MCU runtime: {self.mcu_runtime_ms} ms")
        print(f"  Telemetry faults: {self.telemetry_faults}")
        print(f"  Active faults: {self.active_faults}")
        print("IMU:")
        if self.imu_roll_deg is not None and self.imu_pitch_deg is not None and self.imu_yaw_deg is not None:
            print(
                "  Orientation: "
                f"roll={self.imu_roll_deg:.2f} deg, "
                f"pitch={self.imu_pitch_deg:.2f} deg, "
                f"yaw={self.imu_yaw_deg:.2f} deg"
            )
        elif self.imu_yaw_deg is not None:
            print(f"  Orientation: yaw={self.imu_yaw_deg:.2f} deg")
        else:
            print("  Orientation: n/a")
        if self.imu_accel_x is not None and self.imu_accel_y is not None and self.imu_accel_z is not None:
            print(
                "  Accel: "
                f"x={self.imu_accel_x:.3f} m/s^2, "
                f"y={self.imu_accel_y:.3f} m/s^2, "
                f"z={self.imu_accel_z:.3f} m/s^2"
            )
        if self.imu_gyro_x is not None and self.imu_gyro_y is not None and self.imu_gyro_z is not None:
            print(
                "  Gyro: "
                f"x={self.imu_gyro_x:.2f} deg/s, "
                f"y={self.imu_gyro_y:.2f} deg/s, "
                f"z={self.imu_gyro_z:.2f} deg/s"
            )
        print(f"  Valid: {self.imu_valid}")
        if self.imu_status is not None or self.imu_cal_sys is not None:
            status_str = "n/a"
            if self.imu_status is not None:
                status_str = str(self.imu_status)
            self_test_str = "n/a" if self.imu_self_test is None else f"0x{self.imu_self_test:02X}"
            error_str = "n/a" if self.imu_error is None else str(self.imu_error)
            print(f"  Status: sys={status_str} self_test={self_test_str} err={error_str}")
            if self.imu_cal_sys is not None:
                print(
                    "  Calib: "
                    f"sys={self.imu_cal_sys} gyro={self.imu_cal_gyro} "
                    f"accel={self.imu_cal_accel} mag={self.imu_cal_mag}"
                )
        print("CURRENT INTENT:")
        print(f"  Motion: v={self.current_v:.2f} m/s, w={self.current_w:.2f} rad/s")
        print(f"  Auger: {'ON' if self.auger_enabled else 'OFF'}")
        print(f"  Salt: {'ON' if self.salt_enabled else 'OFF'}")
        print(f"  Chute: {self.chute_angle:.1f} deg")
        print(f"  Headlight: {'ON' if self.headlight_on else 'OFF'}")
        if self.odom_pose is not None:
            print("ODOMETRY:")
            print(
                f"  Pose: x={self.odom_pose.x:.3f} m, "
                f"y={self.odom_pose.y:.3f} m, "
                f"heading={self.odom_heading_deg:.1f} deg"
            )
            if self.odom_distance_m is not None:
                print(f"  Distance: {self.odom_distance_m:.3f} m")
            if self.odom_update_count is not None:
                print(f"  Updates: {self.odom_update_count}")
            if self.odom_m_per_tick is not None:
                print(f"  Meters/tick: {self.odom_m_per_tick:.6f}")
        print("-" * 40)

    def print_odometry_info(self):
        """Print odometry summary."""
        print("" + "-" * 40)
        if self.odom_pose is None:
            print("ODOMETRY: DISABLED or NO DATA")
        else:
            print("ODOMETRY:")
            print(
                f"  Pose: x={self.odom_pose.x:.3f} m, "
                f"y={self.odom_pose.y:.3f} m, "
                f"heading={self.odom_heading_deg:.1f} deg"
            )
            if self.odom_distance_m is not None:
                print(f"  Distance: {self.odom_distance_m:.3f} m")
            if self.odom_update_count is not None:
                print(f"  Updates: {self.odom_update_count}")
            if self.odom_m_per_tick is not None:
                print(f"  Meters/tick: {self.odom_m_per_tick:.6f}")
        print("-" * 40)

    def _kbhit(self) -> bool:
        """Check if keyboard input is available."""
        if self.is_windows:
            return msvcrt.kbhit()
        return select.select([sys.stdin], [], [], 0)[0] != []

    def _getch(self) -> str:
        """Get a character from keyboard."""
        if self.is_windows:
            return msvcrt.getch().decode("utf-8").lower()
        return sys.stdin.read(1).lower()

    def _setup_terminal(self):
        """Setup terminal for non-blocking input (Unix only)."""
        if not self.is_windows and sys.stdin.isatty():
            try:
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
            except termios.error:
                pass

    def _restore_terminal(self):
        """Restore terminal settings (Unix only)."""
        if not self.is_windows and self.old_settings and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except termios.error:
                pass

    async def run(self):
        """Run interactive test loop."""
        if not await self.connect():
            return

        self.print_menu()
        print("Ready! Press keys to send commands/intents...")
        print("(Intents are sent automatically at 20Hz)")

        self._setup_terminal()

        try:
            while True:
                self.update_state_from_telemetry()

                if self._kbhit():
                    key = self._getch()

                    try:
                        if key == "q":
                            print("Quitting...")
                            break

                        if key == "?":
                            self.print_menu()

                        elif key == "v":
                            self.print_state_info()

                        elif key == "b":
                            print(f"Battery: {self.battery_v:.1f} V, Temp: {self.temp_c:.1f} C")

                        elif key == "t":
                            self._print_stats()

                        elif key == "y":
                            self.print_odometry_info()

                        elif key == "d" and self.use_dummy:
                            stats = self.transport.get_stats()
                            print("Dummy Transport Stats:")
                            print(f"  Commands sent: {stats['commands_sent']}")
                            print(f"  Intents sent: {stats['intents_sent']}")
                            print(f"  ACKs received: {stats['acks_received']}")
                            print(f"  Telemetry received: {stats['telemetry_received']}")
                            print(f"  Robot state: {stats['simulated_state']['state']}")
                            print(f"  Motion: v={stats['simulated_state']['v']:.2f}, w={stats['simulated_state']['w']:.2f}")

                        # COMMANDS (wait for ACK)
                        elif key == "h":
                            print("Sending HELLO...")
                            await self.send_command(Hello(version=PROTOCOL_VERSION))

                        elif key == "p":
                            print("Sending PING...")
                            await self.send_command(Ping())

                        elif key == "s":
                            print("Sending STATUS...")
                            await self.send_command(Status())

                        elif key == "i":
                            print("Sending MODE:IDLE...")
                            await self.send_command(Mode(mode=RobotMode.IDLE))

                        elif key == "m":
                            print("Sending MODE:MANUAL...")
                            await self.send_command(Mode(mode=RobotMode.MANUAL))

                        elif key == "n":
                            print("Sending MODE:AUTO...")
                            await self.send_command(Mode(mode=RobotMode.AUTO))

                        elif key == "o":
                            print("Sending STOP...")
                            await self.send_command(Stop())
                            self.current_v = 0.0
                            self.current_w = 0.0
                            self.auger_enabled = False
                            self.salt_enabled = False
                            self.headlight_on = False
                            self.controller.stop_motion()
                            self.controller.set_auger(False)
                            self.controller.set_salt(False)
                            self.controller.set_headlight(False)

                        elif key == "r":
                            print("Sending RESUME...")
                            await self.send_command(Resume())

                        elif key == "e":
                            print("EMERGENCY STOP...")
                            self.controller.estop()
                            self.current_v = 0.0
                            self.current_w = 0.0
                            self.auger_enabled = False
                            self.salt_enabled = False
                            self.headlight_on = False
                            self.controller.stop_motion()
                            self.controller.set_auger(False)
                            self.controller.set_salt(False)
                            self.controller.set_headlight(False)

                        elif key == "f":
                            print("Sending RESET_FAULTS...")
                            await self.send_command(ResetFaults())

                        elif key == "g":
                            print("Sending SHUTDOWN...")
                            await self.send_shutdown()

                        # INTENTS (modify state, sent automatically at 20Hz)
                        elif key == "w":
                            self.current_v = 1.5
                            self.current_w = 0.0
                            self.controller.set_velocity(self.current_v, self.current_w)
                            print("-> Intent: FORWARD (v=1.0)")

                        elif key == "x":
                            self.current_v = -1.5
                            self.current_w = 0.0
                            self.controller.set_velocity(self.current_v, self.current_w)
                            print("-> Intent: BACKWARD (v=-1.0)")

                        elif key == "a":
                            self.current_v = 0.0
                            self.current_w = 3.0
                            self.controller.set_velocity(self.current_v, self.current_w)
                            print("-> Intent: LEFT (w=2.0)")

                        elif key == "z":
                            self.current_v = 0.0
                            self.current_w = -3.0
                            self.controller.set_velocity(self.current_v, self.current_w)
                            print("-> Intent: RIGHT (w=-2.0)")

                        elif key == " ":
                            self.current_v = 0.0
                            self.current_w = 0.0
                            self.controller.stop_motion()
                            print("-> Intent: STOP")

                        elif key == "1":
                            self.auger_enabled = True
                            self.controller.set_auger(True)
                            print("-> Intent: Auger ON")

                        elif key == "2":
                            self.auger_enabled = False
                            self.controller.set_auger(False)
                            print("-> Intent: Auger OFF")

                        elif key == "3":
                            self.salt_enabled = True
                            self.controller.set_salt(True)
                            print("-> Intent: Salt ON")

                        elif key == "4":
                            self.salt_enabled = False
                            self.controller.set_salt(False)
                            print("-> Intent: Salt OFF")

                        elif key == "5":
                            self.chute_angle = 45.0
                            self.controller.set_chute(45.0)
                            print("-> Intent: Chute LEFT (45 deg)")

                        elif key == "6":
                            self.chute_angle = 90.0
                            self.controller.set_chute(90.0)
                            print("-> Intent: Chute CENTER (90 deg)")

                        elif key == "7":
                            self.chute_angle = 135.0
                            self.controller.set_chute(135.0)
                            print("-> Intent: Chute RIGHT (135 deg)")

                        elif key == "[":
                            self.headlight_on = True
                            self.controller.set_headlight(True)
                            print("-> Intent: Headlight ON")

                        elif key == "]":
                            self.headlight_on = False
                            self.controller.set_headlight(False)
                            print("-> Intent: Headlight OFF")

                        else:
                            print(f"Unknown key: '{key}' (press '?' for menu)")

                    except Exception as e:
                        print(f"ERROR: {e}")

                if self._log_file:
                    now = time.time()
                    if now - self._last_log_time >= self.log_interval_s:
                        self._write_stats_log()
                        self._last_log_time = now

                # Emit an immediate snapshot on first telemetry stall, and on resume.
                if self.controller:
                    telem_age = self.controller.subscriber.get_age()
                    if telem_age > 3.0 and not self._last_telem_stalled:
                        self._last_telem_stalled = True
                        print(f"[Diag] Telemetry stall detected (age {telem_age:.2f}s)")
                        self._print_stats()
                        if self._log_file:
                            ts = time.strftime("%Y-%m-%d %H:%M:%S")
                            self._log_file.write(f"[STALL SNAPSHOT {ts}]\n")
                            for line in self._format_stats_lines():
                                self._log_file.write(line + "\n")
                            self._log_file.flush()
                    elif telem_age <= 1.0 and self._last_telem_stalled:
                        self._last_telem_stalled = False
                        print(f"[Diag] Telemetry resumed (age {telem_age:.2f}s)")

                await asyncio.sleep(0.01)

        finally:
            self._restore_terminal()
            await self.disconnect()

async def main():
    def _has_flag(name: str) -> bool:
        return name in sys.argv

    def _get_arg_value(name: str, default: str) -> str:
        if name in sys.argv:
            idx = sys.argv.index(name)
            if idx + 1 < len(sys.argv):
                return sys.argv[idx + 1]
        return default

    use_dummy = _has_flag("--dummy")
    diagnostic = _has_flag("--diag")
    no_reconnect = _has_flag("--no-reconnect")
    no_diag_reconnect = _has_flag("--no-diag-reconnect")
    log_stats = _has_flag("--log-stats")
    read_mode = _get_arg_value("--read-mode", "auto").lower()
    force_rx_thread = _has_flag("--rx-thread")
    force_no_rx_thread = _has_flag("--no-rx-thread")
    log_dir = _get_arg_value("--log-dir", "logs")
    log_interval_s = float(_get_arg_value("--log-interval", "10"))
    log_path = None
    if log_stats:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(log_dir, f"test_run_{timestamp}.txt")

    print("=" * 60)
    print("SBCP v0.3.0 - Interactive Test (ControlLoop)")
    print("=" * 60)
    print(f"Platform: {platform.system()}")
    if use_dummy:
        print("Mode: DUMMY TRANSPORT (simulated Arduino)")
    else:
        print(f"Port: {SERIAL_PORT}")
        print(f"Baud: {BAUD_RATE}")
        print("Mode: REAL SERIAL")
    if diagnostic:
        print("Diagnostics: ENABLED")
    if no_reconnect:
        print("Auto-reconnect: DISABLED")
    if log_stats:
        print(f"Auto-log: {log_path} (interval {log_interval_s:.1f}s)")
    if not use_dummy:
        print(f"Read mode: {read_mode}")
        if force_rx_thread:
            print("RX thread: FORCED ON")
        elif force_no_rx_thread:
            print("RX thread: FORCED OFF")
        if no_diag_reconnect:
            print("Diagnostic reconnect: DISABLED")
    print()
    print("Usage: python test.py [--dummy] [--diag] [--no-reconnect] [--no-diag-reconnect] [--log-stats] [--log-interval SEC] [--log-dir PATH] [--read-mode auto|line|chunk] [--rx-thread|--no-rx-thread]")
    print()

    use_rx_thread = None
    if force_rx_thread:
        use_rx_thread = True
    elif force_no_rx_thread:
        use_rx_thread = False

    tester = SBCPTester(
        port=SERIAL_PORT,
        baud=BAUD_RATE,
        use_dummy=use_dummy,
        diagnostic=diagnostic,
        auto_reconnect=not no_reconnect,
        diagnostic_reconnect=not no_diag_reconnect,
        read_mode=read_mode,
        use_rx_thread=use_rx_thread,
        log_path=log_path,
        log_interval_s=log_interval_s,
    )

    try:
        await tester.run()
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Goodbye!")

if __name__ == "__main__":
    asyncio.run(main())
