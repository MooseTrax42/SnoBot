"""
SBCP (SnoBot Communication Protocol) v0.3.0
Simplfied transport system.
"""

import asyncio
import time
import serial
import serial.tools.list_ports
import json
import threading
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from threading import Thread, Lock, Event
from queue import Queue, Empty
from collections import deque
from sbcp.types import PROTOCOL_VERSION, ARDUINO_BUFFER_BYTES, robot_state_id_from_name
from sbcp.schema import make_envelope, parse_envelope
from common.common_errors import CommunicationError, ParseError
from common.common_events import EventBus, get_event_bus

class TransportBase(ABC):
    """
    Abstract base class for SBCP transport implementations.

    Defines the interface that all transports must implement.
    This ensures consistency and enables polymorphic use of different transport types.
    """
    
    @abstractmethod
    def connect(self) -> bool:
        """
        Open/establish connection.
        
        Returns:
            True if connection successful
            
        Raises:
            CommunicationError: If connection fails.
        """
        pass
        
    @abstractmethod
    def disconnect(self):
        """
        Close connection and clean up resources.
        """
        pass
    
    @abstractmethod
    def send(self, message: Dict[str, Any]) -> bool:
        """
        Send a message.
        
        Args:
            message: Dictionary containing command or data to send.
            
        Returns: 
            True if sent successfully
            
        Raises: 
            CommunicationError: If send fails.    
        """
        pass
    
    @abstractmethod
    def recv(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Receive a message with optional timeout.
        
        Args:
            timeout: Timeout in seconds (None = use default)
            
        Returns:
            Parsed message dict or None if timeout
            
        Raises:
            ParseError: If message parsing fails
        """
        pass
    
    @abstractmethod
    def recv_nowait(self) -> Optional[Dict[str, Any]]:
        """
        Receive a message without blocking.
        
        Returns:
            Parsed message dict or None if no data available
            
        Raises:
            ParseError: If message parsing fails
        """
        pass
    
    @abstractmethod
    def flush_input(self):
        """
        Flush/clear input buffer.
        """
        pass
    
    @abstractmethod
    def flush_output(self):
        """
        Flush/clear output buffer.
        """
        pass
    
    @abstractmethod
    def bytes_available(self) -> int:
        """
        Get number of bytes/messages waiting in input buffer.
        
        Returns:
            Number of bytes or messages available.
        """
        pass
    
    @property
    @abstractmethod
    def connected(self) -> bool:
        """
        Check if transport is currently connected.
        
        Returns:
            True if connected
        """
        pass
        
    # Context manager support.
    def __enter__(self):
        """
        Context manager entry - connect.
        """
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit - disconnect.
        """
        self.disconnect()
        return False    # Don't suppress exceptions.
    
class DummyTransport(TransportBase):
    """
    Dummy transport that simulates Arduino behavior.
    
    - Commands (with "cmd" field) get ACK responses
    - Intents (no "cmd" field) are applied silently
    - Periodic STAT telemetry sent at 12Hz
    """
    
    def __init__(self, response_delay_ms: float = 0, enable_telemetry: bool = True, drop_acks: bool = False):
        """
        Initialize dummy transport.
        
        Args:
            response_delay_ms: Artificial delay for ACK responses
            enable_telemetry: Send periodic STAT telemetry at 12Hz
        """
        self.response_delay_ms = response_delay_ms
        self.enable_telemetry = enable_telemetry
        self._connected = False
        self.drop_acks = drop_acks
        
        # Message queues.
        self.tx_queue: Queue = Queue()
        self.rx_queue: Queue = Queue()
        
        # Statistics.
        self.stats = {
            "commands_sent": 0,
            "intents_sent": 0,
            "acks_received": 0,
            "telemetry_received": 0
        }
        
        self.start_time = time.time()
        self._stat_seq = 0
        
        # Simulated robot state (updated from intents).
        self.sim_state = {
            "state": "BOOT",
            "v": 0.0,
            "w": 0.0,
            "auger_en": False,
            "salt_en": False,
            "chute_angle": 90.0,
            "battery_v": 46.5,
            "temp": 42.0,
            "estop": False,
            "faults": []
        }
        
        # Telemetry thread.
        self._telemetry_thread = None
        self._telemetry_stop = threading.Event()
        
    def connect(self) -> bool:
        """
        Simulate connection.
        """
        if self._connected:
            return True
        self._connected = True
        
        self._telemetry_stop.clear()

        # Start telemetry thread if enabled.
        if self.enable_telemetry and not self._telemetry_thread:
            self._start_telemetry_thread()
        
        return True
        
    def disconnect(self):
        """
        Simulate disconnection.
        """
        if not self._connected:
            return False
        self._connected = False
        
        # Stop telemetry thread.
        if self._telemetry_thread:
            self._telemetry_stop.set()
            self._telemetry_thread.join(timeout=1.0)
        
    @property
    def connected(self) -> bool:
        """
        Check if transport is connected.
        """
        return self._connected
    
    def send(self, message: Dict[str, Any]) -> bool:
        """
        Send message - route to command or intent handler.
        
        Args:
            message: Message dict (command or intent)
            
        Returns:
            True always
        """
        if not self._connected:
            return False
        
        self.tx_queue.put(message)
        
        msg_type, payload, meta = parse_envelope(message)

        # Route based on message type.
        if msg_type == "CMD":
            # COMMAND - generate ACK response
            self.stats["commands_sent"] += 1
            if "seq" not in payload and "seq" in meta:
                payload["seq"] = meta["seq"]
            self._handle_command(payload)
        else:
            # INTENT - apply silently, no response
            self.stats["intents_sent"] += 1
            self._handle_intent(payload)
        
        return True
    
    def _handle_command(self, message: Dict[str, Any]):
        """
        Handle command - generate ACK response.
        
        Args:
            message: Command message
        """
        cmd = message.get("cmd", "")
        seq = message.get("seq", 0)
        
        print(f"[DummyTransport] Handling command: {cmd}")

        # Add artificial delay if configured.
        if self.response_delay_ms > 0:
            time.sleep(self.response_delay_ms / 1000.0)
        
        # Generate ACK response (and side-effects).
        response = self._generate_ack(cmd, seq, message.get("args", {}))
        if self.drop_acks:
            print(f"[DummyTransport] Dropping ACK for {cmd} (seq={seq})")
            return
        self.rx_queue.put(response)
        print(f"[DummyTransport] Sent ACK for {cmd} (seq={seq})")
    
    def _handle_intent(self, message: Dict[str, Any]):
        """
        Handle intent - update simulated state, no response.
        
        Args:
            message: Intent message
        """
        # Update simulated robot state from intent fields.
        if "v" in message:
            self.sim_state["v"] = message["v"]
        if "w" in message:
            self.sim_state["w"] = message["w"]
        if "auger_en" in message:
            self.sim_state["auger_en"] = message["auger_en"]
        if "salt_en" in message:
            self.sim_state["salt_en"] = message["salt_en"]
        if "chute_angle" in message:
            self.sim_state["chute_angle"] = message["chute_angle"]
        if "mode" in message:
            self.sim_state["state"] = message["mode"]
            print(f"[DummyTransport] Mode changed via intent: {message['mode']}")
        if "stop" in message and message["stop"]:
            self.sim_state["state"] = "STOPPED"
            print(f"[DummyTransport] Emergency stop via intent")
        
        if "resume" in message and message["resume"]:
            # Resume to previous state (simplified - just go to MANUAL)
            self.sim_state["state"] = "MANUAL"
            print(f"[DummyTransport] Resume via intent")
        
        if "reset_faults" in message:
            self.sim_state["faults"] = []
            print(f"[DummyTransport] Faults reset via intent")

        if "estop" in message and message["estop"]:
            self.sim_state["state"] = "ESTOPPED"
            print(f"[DummyTransport] E-stop via intent")
        
        # No response for intents!
    
    def _generate_ack(self, cmd: str, seq: int, args: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate ACK response for a command.
        
        Args:
            cmd: Command name
            seq: Sequence number
            args: Command arguments
            
        Returns:
            ACK response dict
        """
        now_ms = int(time.time() * 1000)
        payload = {
            "ok": True,
            "seq": seq,
            "ts": now_ms
        }
        
        # Add command-specific data.
        if cmd == "HELLO":
            payload["data"] = {
                "name": "SnoBot-Dummy",
                "version": PROTOCOL_VERSION,
                "compatible": True,
                "min_version": PROTOCOL_VERSION,
                "max_version": PROTOCOL_VERSION
            }
        
        elif cmd == "MODE":
            # Update simulated state.
            mode = args.get("mode", "IDLE")
            self.sim_state["state"] = mode
            print(f"[DummyTransport] State changed to: {mode}")
        
        elif cmd == "PING" or cmd == "UPTIME":
            uptime_ms = int((time.time() - self.start_time) * 1000)
            payload["data"] = {"uptime_ms": uptime_ms}
        
        elif cmd == "STATUS":
            # Trigger immediate STAT telemetry.
            self._send_stat_telemetry()
        
        elif cmd == "RESET_FAULTS":
            payload["data"] = {
                "cleared": [],
                "remaining": []
            }
            self.sim_state["faults"] = []
        
        elif cmd == "GET_CONFIG":
            param = args.get("param_name", "")
            payload["data"] = {
                "param": param,
                "value": 1.5,
                "units": "unitless"
            }
        
        # All other commands just return OK.
        return make_envelope("ACK", payload, seq=seq, ts_ms=now_ms)
    
    def _start_telemetry_thread(self):
        """
        Start background telemetry thread.
        """
        self._telemetry_stop.clear()
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()
        print("[DummyTransport] Telemetry thread started")
    
    def _telemetry_loop(self):
        """
        Background loop that sends STAT telemetry at 12Hz.
        """
        while not self._telemetry_stop.is_set() and self._connected:
            self._send_stat_telemetry()
            time.sleep(1.0 / 12.0)  # 12Hz = ~83ms
        print("[DummyTransport] Telemetry thread stopped")

    def pause_telemetry(self):
        """Stop sending telemetry for testing timeouts."""
        self._telemetry_stop.set()
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=1.0)
            self._telemetry_thread = None
            
    def resume_telemetry(self):
        """Resume sending telemetry."""
        if not self._telemetry_thread:
            self._start_telemetry_thread()
    
    def _send_stat_telemetry(self):
        """
        Send STAT telemetry message.
        """
        now_ms = int(time.time() * 1000)
        
        state_id = robot_state_id_from_name(self.sim_state["state"])
        stat = {
            "s": self._stat_seq,
            "st": state_id if state_id is not None else 1,
            "e": 1 if self.sim_state["estop"] else 0,
            "enc": [0, 0],
            "imu": [0, 0, 0, 0, 0, 0, 0, 0, 0],
            "v": 0,
            "b": int(self.sim_state["battery_v"] * 100.0),
            "tp": int(self.sim_state["temp"] * 10.0),
            "f": self.sim_state["faults"]
        }
        self._stat_seq = (self._stat_seq + 1) % 65536
        msg = {
            "type": "S",
            "ts": now_ms,
            "data": stat
        }
        self.rx_queue.put(msg)
        # print(f"[DummyTransport] Sent STAT: {self.sim_state['state']}")
    
    def recv(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Receive message with timeout.
        
        Args:
            timeout: Timeout in seconds (default 0.1)
            
        Returns:
            Message dict or None if timeout
        """
        if not self._connected:
            return None
        
        try:
            msg = self.rx_queue.get(timeout=timeout or 0.1)
            
            # Update stats.
            if msg.get("type") == "ACK":
                self.stats["acks_received"] += 1
            elif msg.get("type") in ("STAT", "S"):
                self.stats["telemetry_received"] += 1
            
            return msg
        except Empty:
            return None
        
    def recv_nowait(self) -> Optional[Dict[str, Any]]:
        """
        Receive message without blocking.
        
        Returns:
            Message dict or None if no data.
        """
        if not self._connected:
            return None
        
        try:
            msg = self.rx_queue.get_nowait()
            
            # Update stats.
            if msg.get("type") == "ACK":
                self.stats["acks_received"] += 1
            elif msg.get("type") in ("STAT", "S"):
                self.stats["telemetry_received"] += 1
            
            return msg
        except Empty:
            return None
        
    def flush_input(self):
        """
        Clear input queue.
        """
        while not self.rx_queue.empty():
            try:
                self.rx_queue.get_nowait()
            except Empty:
                break
            
    def flush_output(self):
        """
        Clear output queue.
        """
        while not self.tx_queue.empty():
            try:
                self.tx_queue.get_nowait()
            except Empty:
                break
            
    def bytes_available(self) -> int:
        """
        Get number of messages in receive queue.
        """
        return self.rx_queue.qsize()
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get transport statistics.
        """
        return {
            **self.stats,
            "simulated_state": self.sim_state.copy()
        }
    
    def reset_stats(self):
        """
        Reset statistics.
        """
        self.stats = {
            "commands_sent": 0,
            "intents_sent": 0,
            "acks_received": 0,
            "telemetry_received": 0
        }
        
    def __enter__(self):
        """
        Context manager entry.
        """
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit.
        """
        self.disconnect()
        
    def __repr__(self) -> str:
        """
        String representation.
        """
        status = "connected" if self._connected else "disconnected"
        return f"<DummyTransport status={status} state={self.sim_state['state']} v={self.sim_state['v']:.2f}>"
    
class AsyncSerialTransport(TransportBase):
    """
    Asynchronous serial transport for Arduino communication.

    Provides non-blocking read/write with automatic JSON parsing.
    Maintains separate TX and RX queues for reate-independent operation.
    """
    def __init__(
        self, 
        port: str = "/dev/ttyACM0",
        baud: int = 115200,
        timeout: float = 0.1, # Short timeout for non-blocking, but not too short to avoid dropping issues.
        rx_buffer_size: int = 100,
        write_timeout: Optional[float] = 0.5,
        debug: bool = False,
        read_mode: str = "auto",
        use_rx_thread: bool = True,
        len_prefix: bool = False
    ):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.read_mode = read_mode
        self.use_rx_thread = use_rx_thread
        self.len_prefix = len_prefix
        self._debug = debug

        # Serial connection.
        self._serial: Optional[serial.Serial] = None
        self._connected = False

        # RX queue to store latest messages.
        self._rx_queue: deque = deque(maxlen=rx_buffer_size)
        self._rx_lock = Lock()
        self._tx_lock = Lock()
        self._rx_thread: Optional[Thread] = None
        self._stop_event = Event()
        self._rx_buffer = b""
        self._rx_expected_len: Optional[int] = None

        # Statistics.
        self._messages_sent = 0
        self._messages_received = 0
        self._parse_errors = 0
        self._tx_bytes_total = 0
        self._last_tx_time: float = 0.0
        self._write_failures: int = 0

        # Error detection.
        self._last_error_time: float = 0
        self._error_present = False
        self._last_error_log_time: float = 0
        self._error_log_interval_s: float = 1.0
        self._last_rx_time: float = 0.0
        self._last_byte_time: float = 0.0
        self._rx_thread_alive: bool = False
        self._empty_reads: int = 0
        self._last_in_waiting: int = 0
        self._last_out_waiting: int = 0
        self._last_serial_error: Optional[str] = None
        self._last_serial_error_time: float = 0.0
        self._last_decode_error: Optional[str] = None
        self._last_decode_error_time: float = 0.0
        self._last_debug_log_time: float = 0.0
        self._last_port_present: Optional[bool] = None
        self._bytes_read_total: int = 0
        self._last_in_waiting_nonzero_time: float = 0.0

        # Thread restart tracking.
        self._rx_thread_restarts: int = 0
        self._consecutive_errors: int = 0
        self._last_thread_death_reason: Optional[str] = None
        self._last_successful_read_time: float = 0.0
        self._rx_buffer_last_change_time: float = 0.0  # Track buffer age for stuck messages

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            # Wait for Arduino reset.
            time.sleep(1)

            # Flush buffers.
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            # Initialize time tracking.
            self._last_byte_time = time.time()
            self._last_rx_time = time.time()

            self._connected = True

            # Start RX thread if enabled.
            if self.use_rx_thread:
                self._stop_event.clear()
                self._rx_thread = Thread(target=self._rx_loop, daemon=True)
                self._rx_thread.start()
                self._rx_thread_alive = True
            else:
                self._rx_thread_alive = False

            return True
        except serial.SerialException as e:
            self._connected = False
            raise CommunicationError(f"Failed to open serial port {self.port}: {e}")
        
    def disconnect(self):
        self._last_thread_death_reason = "disconnect() called"
        self._stop_event.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._rx_thread_alive = False

        if self._serial and self._serial.is_open:
            self._serial.close()
        self._connected = False

    def _restart_rx_thread(self) -> bool:
        """Attempt to restart dead RX thread if port still valid.

        Returns:
            True if thread was successfully restarted, False otherwise.
        """
        if self._rx_thread and not self._rx_thread.is_alive():
            if self._serial and self._serial.is_open:
                print("[TRANSPORT] Restarting dead RX thread")
                self._stop_event.clear()
                self._rx_thread = Thread(target=self._rx_loop, daemon=True)
                self._rx_thread.start()
                self._rx_thread_alive = True
                self._rx_thread_restarts += 1
                self._consecutive_errors = 0  # Reset error counter on restart
                self._last_thread_death_reason = None  # Clear death reason
                return True
        return False

    def send(self, message: Dict[str, Any]) -> bool:
        if not self._connected or not self._serial:
            raise CommunicationError(f"Send failed, not connected to {self.port}")
        if "type" not in message or "data" not in message:
            raise CommunicationError("Invalid message: expected envelope with 'type' and 'data'")
        
        try:
            line = json.dumps(message, separators=(",", ":")) + "\n"
            data = line.encode("utf-8")

            if len(data) > 256:
                raise CommunicationError(f"Message too long ({len(data)} bytes)")

            with self._tx_lock:
                bytes_written = self._serial.write(data)
                try:
                    self._last_out_waiting = self._serial.out_waiting
                except Exception:
                    pass
                # Verify write completed (detect partial writes)
                if bytes_written != len(data):
                    raise CommunicationError(f"Partial write: {bytes_written}/{len(data)} bytes")
                # Warn if TX buffer is filling up (potential flow control issue)
                if self._last_out_waiting > 128:
                    print(f"[TRANSPORT] Warning: TX buffer filling ({self._last_out_waiting} bytes waiting)")
            self._messages_sent += 1
            self._tx_bytes_total += len(data)
            self._last_tx_time = time.time()
            return True
        except (serial.SerialTimeoutException, serial.SerialException, OSError) as e:
            self._last_serial_error = f"write: {e}"
            self._last_serial_error_time = time.time()
            self._write_failures += 1
            now = time.time()
            if now - self._last_error_log_time >= self._error_log_interval_s:
                print(f"[TRANSPORT] Serial write error: {e}")
                self._last_error_log_time = now
            raise CommunicationError(f"Serial write failed: {e}")

    def _parse_json_envelope_from_line(self, line: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse a newline-delimited message into an SBCP envelope.

        Some boards can emit occasional noisy bytes on serial resets/brownouts.
        This method attempts strict parsing first, then a conservative recovery
        path that keeps only printable ASCII JSON content between braces.
        """
        line = line.strip()
        if not line:
            return None

        # Ignore non-JSON/plain-text debug lines early.
        start = line.find(b"{")
        end = line.rfind(b"}")
        if start == -1 or end <= start:
            return None

        candidate = line[start : end + 1]

        def _decode_and_parse(candidate: bytes) -> Dict[str, Any]:
            text = candidate.decode("utf-8").strip()
            if not text:
                raise ValueError("empty JSON payload")
            msg = json.loads(text)
            if "type" not in msg or "data" not in msg:
                raise ValueError("invalid envelope (missing type/data)")
            return msg

        # Fast path: strict UTF-8 + JSON.
        try:
            return _decode_and_parse(candidate)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            # Recovery path: drop non-printable noise.
            ascii_candidate = bytes(
                b for b in candidate if b in (9, 10, 13) or 32 <= b <= 126
            ).strip()

            if not ascii_candidate:
                return None

            try:
                return _decode_and_parse(ascii_candidate)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
                # Drop malformed/noisy lines after recovery attempt.
                return None
        
    def recv_nowait(self) -> Optional[Dict[str, Any]]:
        """
        If using RX thread: returns from queue.

        If not using RX thread: polls serial directly with buffering.
        """
        if not self.use_rx_thread:
            # Poll serial directly and use lock for async safety.
            with self._rx_lock:
                if not self._serial or not self._serial.is_open:
                    return None
                
                try:
                    now = time.time()

                    # Check for stale buffer before reading new data.
                    if len(self._rx_buffer) > 0 and self._last_byte_time > 0:
                        buffer_age = now - self._last_byte_time

                        # Clear if no new bytes in 2 seconds.
                        if buffer_age > 2.0:
                            buffer_preview = self._rx_buffer[:50].decode('utf-8', errors='replace')
                            self._log_decode_error(
                                f"[RECV_NOWAIT] Clearing stale buffer (age={buffer_age:.1f}s, "
                                f"len={len(self._rx_buffer)}, preview={repr(buffer_preview)})"
                            )
                            self._rx_buffer = b""
                            self._parse_errors += 1

                            # Flush to sync.
                            try:
                                self._serial.reset_input_buffer()
                            except Exception:
                                pass
                            return None
                        
                    # Hard buffer limit.
                    if len(self._rx_buffer) > ARDUINO_BUFFER_BYTES:
                        self._log_decode_error(f"[RECV_NOWAIT] Buffer overflow ({len(self._rx_buffer)} bytes)")
                        self._rx_buffer = b""
                        self._parse_errors += 1
                        try:
                            self._serial.reset_input_buffer()
                        except Exception:
                            pass
                        return None
                    
                    # Determine read mode.
                    use_readline = False
                    if self.read_mode == "line":
                        use_readline = True
                    elif self.read_mode == "auto":
                        try:
                            import platform as _platform
                            use_readline = _platform.system() == "Windows"
                        except Exception:
                            use_readline = False
                    if self.len_prefix:
                        use_readline = False

                    # Update stats.
                    try:
                        self._last_in_waiting = self._serial.in_waiting
                        self._last_out_waiting = self._serial.out_waiting
                        if self._last_in_waiting > 0:
                            self._last_in_waiting_nonzero_time = time.time()
                    except Exception:
                        pass
                    
                    # First, drain any complete length-prefixed frames already in the buffer.
                    if self.len_prefix:
                        msg = self._try_parse_len_prefixed()
                        if msg:
                            self._messages_received += 1
                            self._last_rx_time = time.time()
                            self._last_byte_time = time.time()
                            return msg

                    # Then, drain any complete lines already in the buffer.
                    while b"\n" in self._rx_buffer:
                        line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        
                        try:
                            msg = self._parse_json_envelope_from_line(line)
                            if not msg:
                                continue

                            # Successfully parsed.
                            self._messages_received += 1
                            self._last_rx_time = time.time()
                            self._last_byte_time = time.time()
                            return msg
                            
                        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as e:
                            self._parse_errors += 1
                            self._last_error_time = time.time()
                            self._log_decode_error(f"decode/parse error: {e}")
                            continue
                    
                    # Read new data from serial.
                    if use_readline:
                        chunk = self._serial.readline()
                    else:
                        chunk = self._serial.read(self._serial.in_waiting or 1)
                    
                    if not chunk:
                        self._empty_reads += 1
                        return None
                    
                    self._bytes_read_total += len(chunk)
                    self._last_byte_time = time.time()
                    
                    # Append to buffer.
                    self._rx_buffer += chunk

                    # Attempt to parse length-prefixed frame from updated buffer.
                    if self.len_prefix:
                        msg = self._try_parse_len_prefixed()
                        if msg:
                            self._messages_received += 1
                            self._last_rx_time = time.time()
                            self._last_byte_time = time.time()
                            return msg
                    
                    # Normalize line endings.
                    if b"\r" in self._rx_buffer:
                        self._rx_buffer = self._rx_buffer.replace(b"\r", b"\n")
                    
                    # Handle complete JSON without readline if it happens.
                    if b"\n" not in self._rx_buffer and len(self._rx_buffer) > 0:

                        # Check if buffer hasn't grown in last 100ms.
                        if not hasattr(self, '_last_buffer_len_nowait'):
                            self._last_buffer_len_nowait = len(self._rx_buffer)
                            self._last_buffer_check_nowait = time.time()
                        elif len(self._rx_buffer) == self._last_buffer_len_nowait:
                            if (time.time() - self._last_buffer_check_nowait) > 0.1:

                                # Buffer stable for 100ms, try to parse data.
                                buf = self._rx_buffer.strip()
                                if buf.startswith(b"{") and buf.endswith(b"}"):
                                    try:
                                        msg = self._parse_json_envelope_from_line(buf)
                                        if msg:
                                            self._messages_received += 1
                                            self._last_rx_time = time.time()
                                            self._last_byte_time = time.time()
                                            self._rx_buffer = b""
                                            self._last_buffer_len_nowait = 0
                                            return msg
                                    except Exception:
                                        pass
                        else:
                            # Buffer changed, reset tracking.
                            self._last_buffer_len_nowait = len(self._rx_buffer)
                            self._last_buffer_check_nowait = time.time()
                    
                    # Process any complete lines that arrived in this chunk.
                    while b"\n" in self._rx_buffer:
                        line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        
                        try:
                            msg = self._parse_json_envelope_from_line(line)
                            if not msg:
                                continue

                            # Successfully parsed.
                            self._messages_received += 1
                            self._last_rx_time = time.time()
                            return msg
                            
                        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as e:
                            self._parse_errors += 1
                            self._last_error_time = time.time()
                            self._log_decode_error(f"decode/parse error: {e}")
                            continue
                    
                    # No complete message yet.
                    return None
                    
                except (serial.SerialException, OSError) as e:
                    raise CommunicationError(f"Serial read failed: {e}")
        else:
            # Using RX thread, just pop from the queue.
            with self._rx_lock:
                if self._rx_queue:
                    return self._rx_queue.popleft()
        
        return None
    
    def recv(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Async transports use recv_nowait, this is for compatibility only."""
        return self.recv_nowait()

    def _try_parse_len_prefixed(self) -> Optional[Dict[str, Any]]:
        """Parse length-prefixed JSON frames from buffer if enabled."""
        if not self.len_prefix:
            return None

        if self._rx_buffer.startswith(b"\n"):
            self._rx_buffer = self._rx_buffer.lstrip(b"\n")

        if self._rx_expected_len is None:
            if not self._rx_buffer.startswith(b"@"):
                return None

            newline_idx = self._rx_buffer.find(b"\n")
            if newline_idx == -1:
                return None

            header = self._rx_buffer[1:newline_idx].strip()
            if not header.isdigit():
                self._parse_errors += 1
                self._last_error_time = time.time()
                self._log_decode_error("invalid length prefix header")
                self._rx_buffer = self._rx_buffer[newline_idx + 1:]
                return None

            self._rx_expected_len = int(header)
            self._rx_buffer = self._rx_buffer[newline_idx + 1:]

        if self._rx_expected_len is None:
            return None

        if len(self._rx_buffer) < self._rx_expected_len:
            return None

        payload = self._rx_buffer[:self._rx_expected_len]
        self._rx_buffer = self._rx_buffer[self._rx_expected_len:]
        self._rx_expected_len = None

        if self._rx_buffer.startswith(b"\n"):
            self._rx_buffer = self._rx_buffer[1:]

        try:
            text = payload.decode("utf-8").strip()
            if not text:
                return None
            msg = json.loads(text)
            if "type" not in msg or "data" not in msg:
                self._parse_errors += 1
                self._last_error_time = time.time()
                self._log_decode_error("invalid envelope (missing type/data)")
                return None
            return msg
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            self._parse_errors += 1
            self._last_error_time = time.time()
            self._log_decode_error(f"len-pref decode/parse error: {e}")
            return None

    def _rx_loop(self):
        """Background RX thread: reads lines, parses JSON, pushes to queue."""
        consecutive_errors = 0
        max_consecutive_errors = 20
        error_backoff = 0.1  # Start with 100ms backoff.
        max_backoff = 2.0    # Cap at 2 seconds.
        error_reset_window_s = 30.0  # Reset consecutive errors if this much time passes without errors.
        use_readline = False
        if self.read_mode == "line":
            use_readline = True
        elif self.read_mode == "auto":
            # Prefer readline on Windows for stability.
            try:
                import platform as _platform
                use_readline = _platform.system() == "Windows"
            except Exception:
                use_readline = False
        if self.len_prefix:
            use_readline = False
        
        while not self._stop_event.is_set():
            try:
                now = time.time()

                # Check for stale buffer.
                if len(self._rx_buffer) > 0 and self._last_byte_time > 0:
                    buffer_age = now - self._last_byte_time

                    if buffer_age > 2.0:
                        buffer_preview = self._rx_buffer[:50].decode('utf-8', errors='replace')
                        self._log_decode_error(
                            f"[RX_LOOP] Clearing stale buffer (age={buffer_age:.1f}s, "
                            f"len={len(self._rx_buffer)}, preview={repr(buffer_preview)})"
                        )
                        self._rx_buffer = b""
                        self._parse_errors += 1

                        # Flush to sync.
                        try:
                            self._serial.reset_input_buffer()
                        except Exception:
                            pass

                        time.sleep(0.01)
                        continue

                # Set a hard buffer limit.
                if len(self._rx_buffer) > ARDUINO_BUFFER_BYTES:
                    self._log_decode_error(f"[RX_LOOP] Buffer overflow ({len(self._rx_buffer)} bytes)")
                    self._rx_buffer = b""
                    self._parse_errors += 1
                    try:
                        self._serial.reset_input_buffer()
                    except Exception:
                        pass
                    time.sleep(0.01)
                    continue

            except Exception as pre_check_error:
                self._log_decode_error(f"[RX_LOOP] Buffer pre-check error: {pre_check_error}")
                time.sleep(0.01)
                continue

            # Check if serial port is still valid.
            if not self._serial or not self._serial.is_open:
                print("[TRANSPORT] Serial port closed, exiting RX thread...")
                self._connected = False
                break

            try:
                try:
                    self._last_in_waiting = self._serial.in_waiting
                    self._last_out_waiting = self._serial.out_waiting
                except Exception:
                    pass

                # Non-blocking read with a small timeout.
                if use_readline:
                    chunk = self._serial.readline()
                else:
                    chunk = self._serial.read(self._serial.in_waiting or 1)

                if not chunk:
                    self._empty_reads += 1
                    if self._debug:
                        now = time.time()
                    if self._last_byte_time and (now - self._last_byte_time) > 1.0:
                        if now - self._last_debug_log_time >= 1.0:
                            self._last_debug_log_time = now
                            print(
                                "[TRANSPORT] idle "
                                f"{now - self._last_byte_time:.2f}s "
                                f"in_waiting={self._last_in_waiting} "
                                f"out_waiting={self._last_out_waiting} "
                                f"port_present={self._port_present()} "
                                f"cts={self._get_signal('cts')} "
                                f"dsr={self._get_signal('dsr')} "
                                f"ri={self._get_signal('ri')} "
                                f"cd={self._get_signal('cd')} "
                                f"dtr={self._get_signal('dtr')} "
                                f"rts={self._get_signal('rts')}"
                            )

                    # No data available, brief sleep.
                    time.sleep(0.001)
                    continue
                self._empty_reads = 0

                # Successful read, reset error tracking.
                consecutive_errors = 0
                error_backoff = 0.1
                self._last_successful_read_time = time.time()
                self._last_in_waiting_nonzero_time = self._last_successful_read_time

                self._last_byte_time = time.time()
                self._rx_buffer += chunk
                self._bytes_read_total += len(chunk)

                # Parse any length-prefixed frames first.
                if self.len_prefix:
                    while True:
                        msg = self._try_parse_len_prefixed()
                        if not msg:
                            break
                        with self._rx_lock:
                            self._rx_queue.append(msg)
                        self._messages_received += 1
                        self._last_rx_time = time.time()

                # Treat \r as a line delimiter in the event that it occurs.
                if b"\r" in self._rx_buffer:
                    self._rx_buffer = self._rx_buffer.replace(b"\r", b"\n")

                # Handle if no newline and buffer is stable.
                if b"\n" not in self._rx_buffer and len(self._rx_buffer) > 0:

                    # Check if buffer hasn't grown recently.
                    if not hasattr(self, '_last_buffer_len'):
                        self._last_buffer_len = len(self._rx_buffer)
                        self._last_buffer_check = time.time()
                    elif len(self._rx_buffer) == self._last_buffer_len:
                        if (time.time() - self._last_buffer_check) > 0.1:

                            # Consider stable.
                            buf = self._rx_buffer.strip()
                            if buf.startswith(b"{") and buf.endswith(b"}"):
                                try:
                                    msg = self._parse_json_envelope_from_line(buf)
                                    if msg:
                                        with self._rx_lock:
                                            self._rx_queue.append(msg)
                                        self._messages_received += 1
                                        self._last_rx_time = time.time()
                                        self._rx_buffer = b""
                                        self._last_buffer_len = 0
                                        continue
                                except Exception:
                                    pass
                    else:
                        # Buffer changed, reset tracking.
                        self._last_buffer_len = len(self._rx_buffer)
                        self._last_buffer_check = time.time()
                    
                # Buffer overflow protection.
                if len(self._rx_buffer) > ARDUINO_BUFFER_BYTES * 2:
                    self._rx_buffer = b""
                    self._parse_errors += 1
                    self._last_error_time = time.time()
                    self._log_decode_error("rx buffer overflow")
                    continue

                # Process complete lines.
                while b"\n" in self._rx_buffer:
                    line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                    line = line.strip()
                    
                    if not line:
                        continue
                            
                    try:
                        msg = self._parse_json_envelope_from_line(line)
                        if not msg:
                            continue

                        # Successfully parsed, add to queue.
                        with self._rx_lock:
                            self._rx_queue.append(msg)
                        self._messages_received += 1
                        self._last_rx_time = time.time()
                    
                    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as e:
                        self._parse_errors += 1
                        self._last_error_time = time.time()
                        self._log_decode_error(f"decode/parse error: {e}")
                        # Then continue processing since it's not a fatal error.

            except (serial.SerialException, OSError, PermissionError) as e:
                # Critical serial communication error.
                consecutive_errors += 1
                self._consecutive_errors = consecutive_errors
                self._last_serial_error = f"read: {e}"
                self._last_serial_error_time = time.time()
                if not self._last_thread_death_reason:
                    self._last_thread_death_reason = f"serial read error: {e}"

                # Time-based error reset. 
                now = time.time()
                if self._last_successful_read_time > 0 and (now - self._last_successful_read_time) >= error_reset_window_s:
                    if consecutive_errors > 0:
                        print(f"[TRANSPORT] Resetting error count after {error_reset_window_s}s recovery window")
                        consecutive_errors = 0
                        self._consecutive_errors = 0
                        error_backoff = 0.1

                # Log error with rate limiting.
                if now - self._last_error_log_time >= self._error_log_interval_s:
                    print(f"[TRANSPORT] Serial error #{consecutive_errors}: {e}")
                    self._last_error_log_time = now

                if consecutive_errors >= max_consecutive_errors:
                    self._last_thread_death_reason = f"Too many consecutive errors ({consecutive_errors}): {e}"
                    print(f"[TRANSPORT] {self._last_thread_death_reason}")

                    # Attempt port reset.
                    try:
                        print("[TRANSPORT] Attempting serial port reset...")
                        if self._serial and self._serial.is_open:
                            self._serial.close()
                            time.sleep(1.0)

                             # Reopen with the same settings.
                            self._serial = serial.Serial(
                                port=self.port,
                                baudrate=self.baud,
                                timeout=self.timeout,
                                write_timeout=self.write_timeout,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE
                            )
                            self._serial.reset_input_buffer()
                            self._serial.reset_output_buffer()

                            # Clear internal state.
                            self._rx_buffer = b""
                            consecutive_errors = 0
                            error_backoff = 0.1
                            self._rx_thread_restarts += 1

                            print("[TRANSPORT] Serial port reset successful")
                            continue # Keep thread alive.
                        
                    except Exception as e:
                        print(f"[TRANSPORT] Serial reset failed: {e}")
                        self._connected = False

                        # Signal failure.
                        with self._rx_lock:
                            self._rx_queue.clear()
                        break

                    # Exponential backoff before retry.
                    time.sleep(error_backoff)
                    error_backoff = min(error_backoff * 2, max_backoff)

            except Exception as e:
                # Unexpected error, log but continue.
                self._parse_errors += 1
                self._last_error_time = time.time()
                self._log_decode_error(f"unexpected rx loop error: {e}")

                # Brief pause to prevent tight error loop.
                time.sleep(0.01)

        # Thread exit cleanup.
        print("[TRANSPORT] RX thread exiting...")
        self._rx_thread_alive = False

    def flush_input(self):
        if self._serial and self._serial.is_open:
            with self._rx_lock:
                self._serial.reset_input_buffer()
                self._rx_queue.clear()

    def flush_output(self):
        if self._serial and self._serial.is_open:
            with self._tx_lock:
                self._serial.reset_output_buffer()

    def bytes_available(self) -> int:
        with self._rx_lock:
            return len(self._rx_queue)

    def _log_decode_error(self, detail: str):
        now = time.time()
        self._last_decode_error = detail
        self._last_decode_error_time = now
        if now - self._last_error_log_time >= self._error_log_interval_s:
            self._last_error_log_time = now
            print(f"[TRANSPORT] Error decoding message: {detail}")

    def _get_signal(self, name: str):
        if not self._serial:
            return None
        try:
            return getattr(self._serial, name, None)
        except Exception:
            return None

    def _port_present(self) -> Optional[bool]:
        """Return True/False if the OS reports the port, None if unavailable."""
        try:
            ports = serial.tools.list_ports.comports()
        except Exception:
            return None
        target = str(self.port).lower()
        for p in ports:
            device = str(getattr(p, "device", "")).lower()
            if device == target:
                return True
        return False

    def get_stats(self) -> Dict[str, Any]:
        """Return transport statistics and error info."""
        serial_open = bool(self._serial and self._serial.is_open)
        now = time.time()
        last_rx_age = (now - self._last_rx_time) if self._last_rx_time else 0.0
        last_byte_age = (now - self._last_byte_time) if self._last_byte_time else 0.0
        last_in_waiting_nonzero_age = (now - self._last_in_waiting_nonzero_time) if self._last_in_waiting_nonzero_time else 0.0
        last_tx_age = (now - self._last_tx_time) if self._last_tx_time else 0.0
        return {
            "messages_sent": self._messages_sent,
            "messages_received": self._messages_received,
            "parse_errors": self._parse_errors,
            "last_error_time": self._last_error_time,
            "rx_queue_len": len(self._rx_queue),
            "rx_buffer_len": len(self._rx_buffer),
            "connected": self._connected,
            "serial_open": serial_open,
            "rx_thread_alive": bool(self._rx_thread and self._rx_thread.is_alive()),
            "last_rx_age_s": last_rx_age,
            "last_byte_age_s": last_byte_age,
            "last_tx_age_s": last_tx_age,
            "tx_bytes_total": self._tx_bytes_total,
            "write_failures": self._write_failures,
            "last_in_waiting_nonzero_age_s": last_in_waiting_nonzero_age,
            "rx_bytes_total": self._bytes_read_total,
            "port": self.port,
            "baud": self.baud,
            "timeout": self.timeout,
            "write_timeout": self.write_timeout,
            "read_mode": self.read_mode,
            "in_waiting": self._last_in_waiting,
            "out_waiting": self._last_out_waiting,
            "empty_reads": self._empty_reads,
            "last_serial_error": self._last_serial_error,
            "last_serial_error_time": self._last_serial_error_time,
            "last_decode_error": self._last_decode_error,
            "last_decode_error_time": self._last_decode_error_time,
            "port_present": self._port_present(),
            "use_rx_thread": self.use_rx_thread,
            "cts": self._get_signal("cts"),
            "dsr": self._get_signal("dsr"),
            "ri": self._get_signal("ri"),
            "cd": self._get_signal("cd"),
            "dtr": self._get_signal("dtr"),
            "rts": self._get_signal("rts"),
            # Thread restart diagnostics
            "consecutive_errors": self._consecutive_errors,
            "rx_thread_restarts": self._rx_thread_restarts,
            "last_thread_death_reason": self._last_thread_death_reason,
        }
        
    def __repr__(self) -> str:
        status = "connected" if self._connected else "disconnected"
        return f"<AsyncSerialTransport port={self.port} baud={self.baud} status={status}>"
