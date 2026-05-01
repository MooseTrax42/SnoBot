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
from sbcp.types import PROTOCOL_VERSION, RobotState

from collections import deque

SERIAL_PORT = "COM3" if platform.system() == "Windows" else "/dev/ttyACM0"
BAUD_RATE = 115200

# Breadcrumb decode map (from Arduino system_test.ino)
BREADCRUMB_MAP = {
    100: "LOOP_START",
    110: "LOOP_SERIAL_CHECK",
    190: "LOOP_SENDSTAT_START",
    191: "LOOP_SENDSTAT_END",
    210: "LOOP_END",
    300: "SENDSTAT_ENTER",
    310: "SENDSTAT_BUILD_JSON",
    320: "SENDSTAT_CHECK_SIZE",
    330: "SENDSTAT_SERIAL_WRITE_START",
    331: "SENDSTAT_SERIAL_WRITE_END",
    340: "SENDSTAT_PRINTLN_START",
    341: "SENDSTAT_PRINTLN_END",
    350: "SENDSTAT_EXIT"
}


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
        self.uptime_ms = 0
        self.telemetry_faults = []
        self.active_faults = []
        self.mcu_rx_msgs = 0
        self.mcu_rx_overflows = 0
        self.mcu_rx_bytes = 0
        self.mcu_last_rx_ms = 0
        self.mcu_tx_msgs = 0
        self.mcu_tx_bytes = 0
        self.mcu_tx_avail = None
        self.mcu_hello_done = None
        self.mcu_loop_max_us = 0
        self.mcu_loop_count = 0
        self.mcu_stream_period_ms = 0
        self.mcu_sendstat_calls = 0
        self.mcu_sendstat_success = 0
        self.mcu_sendstat_blocked = 0
        self.mcu_stat_attempts = 0
        self.mcu_stat_failures = 0
        self.mcu_stat_consecutive_failures = 0
        self.mcu_stat_last_attempt_ms = 0
        self.mcu_stat_last_success_ms = 0
        self.mcu_serial_write_entered = 0
        self.mcu_serial_write_exited = 0
        self.mcu_max_serial_write_time = 0
        self.mcu_max_println_time = 0
        self.mcu_stat_len_last = 0
        self.mcu_stat_tx_planned = 0
        self.mcu_stat_tx_written = 0
        self.mcu_stat_tx_write_calls = 0
        self._last_host_rx_bytes_total = None
        self._last_host_msgs_received = None
        self._last_serial_open = None
        self._last_telem_stalled = False

        # Breadcrumb tracking for freeze detection
        self.last_breadcrumb = None
        self.last_breadcrumb_time = None
        self.breadcrumb_history = deque(maxlen=50)
        self.last_loop_iterations = None
        self.last_loop_time = None
        self.freeze_detected = False
        self.freeze_start_time = None

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

    def _format_breadcrumb(self, bc):
        """Format breadcrumb for display."""
        if bc is None:
            return "N/A"
        name = BREADCRUMB_MAP.get(bc, f"UNKNOWN({bc})")
        return f"{bc:3d} ({name})"

    def _check_for_freeze(self):
        """Check if Arduino has frozen based on breadcrumb age and telemetry."""
        if self.last_breadcrumb_time is None:
            return
        
        breadcrumb_age = time.time() - self.last_breadcrumb_time
        telemetry_age = self.controller.subscriber.get_age() if self.controller else float('inf')
        
        # Detect freeze start: breadcrumb stale >3s AND telemetry stale >2s
        # (If telemetry is fresh but breadcrumbs stuck, that's JSON overflow not full freeze)
        if breadcrumb_age > 3.0 and telemetry_age > 2.0 and not self.freeze_detected:
            self.freeze_detected = True
            self.freeze_start_time = time.time()
            self._report_freeze()
        
        # Clear freeze if EITHER breadcrumb updates OR telemetry resumes
        elif self.freeze_detected:
            breadcrumb_fresh = breadcrumb_age < 1.0
            telemetry_fresh = telemetry_age < 1.0
            
            if breadcrumb_fresh or telemetry_fresh:
                duration = time.time() - self.freeze_start_time
                
                if breadcrumb_fresh and telemetry_fresh:
                    status = "Breadcrumb and telemetry resumed"
                elif telemetry_fresh:
                    status = "Telemetry resumed (breadcrumbs stuck - likely JSON overflow)"
                else:
                    status = "Breadcrumb resumed"
                
                print(f"\n✅ Freeze cleared! {status} after {duration:.1f}s\n")
                if self._log_file:
                    self._log_file.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] Freeze cleared: {status} after {duration:.1f}s\n\n")
                    self._log_file.flush()
                self.freeze_detected = False
                self.freeze_start_time = None
            self.freeze_start_time = None

    def _report_freeze(self):
        """Report freeze detection with breadcrumb history."""
        msg = f"""
{'='*60}
🚨 FREEZE DETECTED at {time.strftime('%H:%M:%S')} 🚨
{'='*60}

LAST KNOWN LOCATION:
  Breadcrumb: {self._format_breadcrumb(self.last_breadcrumb)}
  Age: {time.time() - self.last_breadcrumb_time:.1f}s

RECENT BREADCRUMB HISTORY (last 10):
"""
        print(msg)
        if self._log_file:
            self._log_file.write(f"\n{msg}")
        
        # Show last 10 breadcrumbs
        for i, bc_data in enumerate(list(self.breadcrumb_history)[-10:]):
            line = f"  {i+1:2d}. {self._format_breadcrumb(bc_data['value'])} @ {time.strftime('%H:%M:%S', time.localtime(bc_data['time']))}"
            print(line)
            if self._log_file:
                self._log_file.write(line + "\n")
        
        # Interpretation
        bc = self.last_breadcrumb
        if bc == 330:
            interp = "Froze during serializeJson() - JSON serialization blocking"
        elif bc == 340:
            interp = "Froze during Serial.println() - serial write blocking"
        elif bc == 331:
            interp = "Froze after Serial.write() completed - println() issue"
        elif bc in [110, 120]:
            interp = "Froze during serial read/processing"
        elif bc >= 300 and bc < 350:
            interp = "Froze inside sendStat() - telemetry transmission issue"
        else:
            interp = "Froze outside sendStat() - may not be serial-related"
        
        footer = f"\nINTERPRETATION: {interp}\n{'='*60}\n"
        print(footer)
        if self._log_file:
            self._log_file.write(footer)
            self._log_file.flush()

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
        if latest.get("type") == "STAT":
            self.robot_state = data.get("state", "UNKNOWN")
            self.telemetry_faults = data.get("faults", self.telemetry_faults) or []
            if "battery_v" in data:
                self.battery_v = data.get("battery_v", self.battery_v)
            if "temp" in data:
                self.temp_c = data.get("temp", self.temp_c)
            self.estop_active = bool(data.get("estop", False))
            if "uptime_ms" in data:
                self.uptime_ms = data.get("uptime_ms", self.uptime_ms)
            if "rx_msgs" in data:
                self.mcu_rx_msgs = data.get("rx_msgs", self.mcu_rx_msgs)
            if "rx_bytes" in data:
                self.mcu_rx_bytes = data.get("rx_bytes", self.mcu_rx_bytes)
            if "rx_last_ms" in data:
                self.mcu_last_rx_ms = data.get("rx_last_ms", self.mcu_last_rx_ms)
            if "hello_done" in data:
                self.mcu_hello_done = data.get("hello_done", self.mcu_hello_done)
            if "rx_overflows" in data:
                self.mcu_rx_overflows = data.get("rx_overflows", self.mcu_rx_overflows)
            if "tx_msgs" in data:
                self.mcu_tx_msgs = data.get("tx_msgs", self.mcu_tx_msgs)
            if "tx_bytes" in data:
                self.mcu_tx_bytes = data.get("tx_bytes", self.mcu_tx_bytes)
            if "tx_avail" in data:
                self.mcu_tx_avail = data.get("tx_avail", self.mcu_tx_avail)
            if "sendstat_calls" in data:
                self.mcu_sendstat_calls = data.get("sendstat_calls", self.mcu_sendstat_calls)
            if "sendstat_success" in data:
                self.mcu_sendstat_success = data.get("sendstat_success", self.mcu_sendstat_success)
            if "sendstat_blocked" in data:
                self.mcu_sendstat_blocked = data.get("sendstat_blocked", self.mcu_sendstat_blocked)
            if "stat_attempts" in data:
                self.mcu_stat_attempts = data.get("stat_attempts", self.mcu_stat_attempts)
            if "stat_failures" in data:
                self.mcu_stat_failures = data.get("stat_failures", self.mcu_stat_failures)
            if "stat_consecutive_failures" in data:
                self.mcu_stat_consecutive_failures = data.get("stat_consecutive_failures", self.mcu_stat_consecutive_failures)
            if "stat_last_attempt_ms" in data:
                self.mcu_stat_last_attempt_ms = data.get("stat_last_attempt_ms", self.mcu_stat_last_attempt_ms)
            if "stat_last_success_ms" in data:
                self.mcu_stat_last_success_ms = data.get("stat_last_success_ms", self.mcu_stat_last_success_ms)
            if "serial_write_entered" in data:
                self.mcu_serial_write_entered = data.get("serial_write_entered", self.mcu_serial_write_entered)
            if "serial_write_exited" in data:
                self.mcu_serial_write_exited = data.get("serial_write_exited", self.mcu_serial_write_exited)
            if "max_serial_write_time" in data:
                self.mcu_max_serial_write_time = data.get("max_serial_write_time", self.mcu_max_serial_write_time)
            if "max_println_time" in data:
                self.mcu_max_println_time = data.get("max_println_time", self.mcu_max_println_time)
            if "stat_len_last" in data:
                self.mcu_stat_len_last = data.get("stat_len_last", self.mcu_stat_len_last)
            if "stat_tx_planned" in data:
                self.mcu_stat_tx_planned = data.get("stat_tx_planned", self.mcu_stat_tx_planned)
            if "stat_tx_written" in data:
                self.mcu_stat_tx_written = data.get("stat_tx_written", self.mcu_stat_tx_written)
            if "stat_tx_write_calls" in data:
                self.mcu_stat_tx_write_calls = data.get("stat_tx_write_calls", self.mcu_stat_tx_write_calls)
            if "loop_max_us" in data:
                self.mcu_loop_max_us = data.get("loop_max_us", self.mcu_loop_max_us)
            if "loop_count" in data:
                self.mcu_loop_count = data.get("loop_count", self.mcu_loop_count)
            if "stream_period_ms" in data:
                self.mcu_stream_period_ms = data.get("stream_period_ms", self.mcu_stream_period_ms)
            
            # Track breadcrumbs for freeze detection
            if "breadcrumb" in data:
                bc = data["breadcrumb"]
                if bc != self.last_breadcrumb:
                    now = time.time()
                    self.breadcrumb_history.append({
                        "value": bc,
                        "time": now
                    })
                    self.last_breadcrumb = bc
                    self.last_breadcrumb_time = now
            
            # Track loop progress
            if "loop_iterations" in data:
                loop_iter = data["loop_iterations"]
                now = time.time()
                if self.last_loop_iterations is not None and self.last_loop_time is not None:
                    delta = loop_iter - self.last_loop_iterations
                    time_delta = now - self.last_loop_time
                    # Only check if enough time and iterations have passed
                    if time_delta > 0.1 and delta > 0:  # At least 100ms and some iterations
                        loops_per_sec = delta / time_delta
                        # Warn if loop rate drops below 100 Hz (should be ~1000 Hz)
                        # But only if we have a meaningful sample
                        if loops_per_sec < 100 and delta > 10:  # Need at least 10 iterations
                            print(f"⚠️  Loop slowdown: {loops_per_sec:.1f} loops/sec")
                            if self._log_file:
                                self._log_file.write(f"[{time.strftime('%H:%M:%S')}] Loop slowdown: {loops_per_sec:.1f} loops/sec\n")
                                self._log_file.flush()
                self.last_loop_iterations = loop_iter
                self.last_loop_time = now
            
            if self.robot_state in ("STOPPED", "FAULT", "ESTOPPED"):
                self.current_v = 0.0
                self.current_w = 0.0
                self.auger_enabled = False
                self.salt_enabled = False
                self.headlight_on = False
        if self.controller:
            self.active_faults = sorted(self.controller.state_machine.active_faults)

    def _format_stats_lines(self):
        lines = []
        if not self.controller:
            return lines
        sub_stats = self.controller.subscriber.get_stats()
        lines.append("Subscriber Stats:")
        lines.append(f"  Messages received: {sub_stats['messages_received']}")
        lines.append(f"  Latest age: {sub_stats['latest_age_s']:.3f}s")
        lines.append(f"  Uptime: {self.uptime_ms} ms")
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
            if self.mcu_rx_msgs or self.mcu_rx_bytes:
                lines.append("MCU RX Stats:")
                lines.append(f"  rx_msgs: {self.mcu_rx_msgs}")
                lines.append(f"  rx_bytes: {self.mcu_rx_bytes}")
                lines.append(f"  rx_last_ms: {self.mcu_last_rx_ms}")
                lines.append(f"  rx_overflows: {self.mcu_rx_overflows}")
            if self.mcu_tx_msgs or self.mcu_tx_bytes or self.mcu_tx_avail is not None:
                lines.append("MCU TX Stats:")
                lines.append(f"  tx_msgs: {self.mcu_tx_msgs}")
                lines.append(f"  tx_bytes: {self.mcu_tx_bytes}")
                lines.append(f"  tx_avail: {self.mcu_tx_avail}")
            if self.mcu_sendstat_calls or self.mcu_sendstat_success or self.mcu_sendstat_blocked:
                lines.append("MCU STAT Stats:")
                lines.append(f"  sendstat_calls: {self.mcu_sendstat_calls}")
                lines.append(f"  sendstat_success: {self.mcu_sendstat_success}")
                lines.append(f"  sendstat_blocked: {self.mcu_sendstat_blocked}")
                lines.append(f"  stat_attempts: {self.mcu_stat_attempts}")
                lines.append(f"  stat_failures: {self.mcu_stat_failures}")
                lines.append(f"  stat_consecutive_failures: {self.mcu_stat_consecutive_failures}")
                lines.append(f"  stat_last_attempt_ms: {self.mcu_stat_last_attempt_ms}")
                lines.append(f"  stat_last_success_ms: {self.mcu_stat_last_success_ms}")
                lines.append(f"  serial_write_entered: {self.mcu_serial_write_entered}")
                lines.append(f"  serial_write_exited: {self.mcu_serial_write_exited}")
                lines.append(f"  max_serial_write_time: {self.mcu_max_serial_write_time}")
                lines.append(f"  max_println_time: {self.mcu_max_println_time}")
                lines.append(f"  stat_len_last: {self.mcu_stat_len_last}")
                lines.append(f"  stat_tx_planned: {self.mcu_stat_tx_planned}")
                lines.append(f"  stat_tx_written: {self.mcu_stat_tx_written}")
                lines.append(f"  stat_tx_write_calls: {self.mcu_stat_tx_write_calls}")
            if self.mcu_loop_max_us > 0:
                lines.append("MCU Loop Stats:")
                lines.append(f"  loop_max_us: {self.mcu_loop_max_us}")
                lines.append(f"  loop_count: {self.mcu_loop_count}")
                if self.mcu_stream_period_ms > 0:
                    lines.append(f"  stream_period_ms: {self.mcu_stream_period_ms}")
            if self.mcu_hello_done is not None:
                lines.append(f"  hello_done: {self.mcu_hello_done}")
            
            # Add breadcrumb tracking info
            if self.last_breadcrumb is not None:
                lines.append("Breadcrumb Tracking:")
                lines.append(f"  Current: {self._format_breadcrumb(self.last_breadcrumb)}")
                if self.last_breadcrumb_time:
                    age = time.time() - self.last_breadcrumb_time
                    lines.append(f"  Age: {age:.2f}s")
                    if age > 2.0:
                        lines.append(f"  ⚠️  WARNING: Breadcrumb stale!")
                lines.append(f"  History size: {len(self.breadcrumb_history)}")
                if self.freeze_detected and self.freeze_start_time is not None:
                    duration = time.time() - self.freeze_start_time
                    lines.append(f"  🚨 FREEZE ACTIVE: {duration:.1f}s")
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
        print("  v - View current state")
        print("  b - Battery info")
        print("  t - Subscriber stats")
        if self.use_dummy:
            print("  d - Dummy transport stats")
        print("  ? - Show this menu")
        print("  q - Quit")
        print("=" * 60)
        print("FREEZE MONITORING:")
        print("  - Breadcrumb tracking enabled")
        print("  - Auto-detects Arduino freezes (>3s stall)")
        print("  - Shows last known location when frozen")
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
        print(f"  Uptime: {self.uptime_ms} ms")
        print(f"  Telemetry faults: {self.telemetry_faults}")
        print(f"  Active faults: {self.active_faults}")
        print("CURRENT INTENT:")
        print(f"  Motion: v={self.current_v:.2f} m/s, w={self.current_w:.2f} rad/s")
        print(f"  Auger: {'ON' if self.auger_enabled else 'OFF'}")
        print(f"  Salt: {'ON' if self.salt_enabled else 'OFF'}")
        print(f"  Chute: {self.chute_angle:.1f} deg")
        print(f"  Headlight: {'ON' if self.headlight_on else 'OFF'}")
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
                
                # Check for breadcrumb freeze
                self._check_for_freeze()

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
