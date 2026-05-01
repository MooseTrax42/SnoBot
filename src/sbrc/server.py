"""
Bluetooth RFCOMM server for ESP32 controller connections.

Manages the RFCOMM socket, connection lifecycle, RX/TX threads,
and telemetry push.  Only one controller connection at a time.

Threading model:
  - Accept thread:     blocks on socket.accept(), loops after disconnect
  - RX thread:         reads JSON lines from the connected socket,
                       dispatches via ControllerAPI, enqueues responses
  - TX thread:         drains the outbound queue (responses + telemetry)
  - Telemetry task:    asyncio task, builds telemetry at 5 Hz and enqueues
  - Watchdog task:     asyncio task, 1 Hz, triggers disconnect on RX silence

Safety:  if the connection drops, stop_motion() + auger/salt off immediately.
"""

import asyncio
import json
import platform
import queue
import socket
import threading
import time
from typing import Optional, Dict, Any

from sbrc.api import ControllerAPI


class BluetoothServer:
    """
    Bluetooth RFCOMM server for an ESP32-based handheld controller.
    """

    def __init__(
        self,
        api: ControllerAPI,
        channel: int = 1,
        telemetry_hz: float = 5.0,
        disconnect_watchdog_s: float = 6.0,
        log_controller_inputs: bool = False,
        log_vel_every_s: float = 0.25,
    ):
        self._api = api
        self._channel = channel
        self._telemetry_hz = telemetry_hz
        self._watchdog_timeout = disconnect_watchdog_s
        self._log_controller_inputs = log_controller_inputs
        self._log_vel_every_s = max(0.0, float(log_vel_every_s))
        self._last_vel_log_s: float = 0.0

        # Sockets.
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._client_addr: Optional[str] = None
        self._connected = False

        # Threads.
        self._accept_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._disconnect_event = threading.Event()

        # TX queues.
        # Keep command responses/ACKs separate from telemetry so control-path
        # traffic cannot be starved by periodic telemetry.
        self._tx_queue_ack: queue.Queue = queue.Queue(maxsize=64)
        self._tx_queue_telem: queue.Queue = queue.Queue(maxsize=64)

        # Watchdog.
        self._last_rx_time: float = 0.0

        # Asyncio tasks.
        self._telemetry_task: Optional[asyncio.Task] = None
        self._watchdog_task: Optional[asyncio.Task] = None

        # Stats.
        self._stats: Dict[str, Any] = {
            "connections": 0,
            "messages_rx": 0,
            "messages_tx": 0,
            "errors": 0,
        }

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Bind RFCOMM server and start accept thread + asyncio tasks."""
        self._stop_event.clear()
        self._bt_available = False

        try:
            self._server_sock = socket.socket(
                socket.AF_BLUETOOTH,
                socket.SOCK_STREAM,
                socket.BTPROTO_RFCOMM,
            )
            self._server_sock.settimeout(1.0)

            # Windows typically expects an explicit BDADDR_ANY string for RFCOMM bind.
            if platform.system() == "Windows":
                any_addr = getattr(socket, "BDADDR_ANY", "00:00:00:00:00:00")
                bind_candidates = [(any_addr, self._channel)]
            else:
                bind_candidates = [("", self._channel)]

            bind_err = None
            for bind_addr in bind_candidates:
                try:
                    self._server_sock.bind(bind_addr)
                    bind_err = None
                    break
                except OSError as e:
                    # Keep the first error so a later fallback does not hide
                    # the real root cause (common on Windows RFCOMM bind).
                    if bind_err is None:
                        bind_err = e

            if bind_err is not None:
                raise bind_err

            self._server_sock.listen(1)
            self._bt_available = True
            print(f"[BluetoothServer] Listening on RFCOMM channel {self._channel}")
        except (AttributeError, OSError) as e:
            print(f"[BluetoothServer] Bluetooth unavailable: {e}")
            print("[BluetoothServer] Running without BT (API still usable programmatically)")
            self._server_sock = None

        if self._bt_available:
            self._accept_thread = threading.Thread(
                target=self._accept_loop, daemon=True, name="bt-accept",
            )
            self._accept_thread.start()

        self._telemetry_task = asyncio.create_task(self._telemetry_loop())
        self._watchdog_task = asyncio.create_task(self._watchdog_loop())

    async def stop(self) -> None:
        """Shut down everything cleanly."""
        self._stop_event.set()

        # Disconnect current client.
        if self._connected:
            self._on_disconnect("server stopping")

        # Cancel asyncio tasks.
        for task in (self._telemetry_task, self._watchdog_task):
            if task and not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        # Close server socket (unblocks accept thread).
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass

        # Join accept thread.
        if self._accept_thread and self._accept_thread.is_alive():
            self._accept_thread.join(timeout=3.0)

        print("[BluetoothServer] Stopped")

    # ------------------------------------------------------------------
    # Connection management (accept thread)
    # ------------------------------------------------------------------

    def _accept_loop(self):
        while not self._stop_event.is_set():
            try:
                client_sock, addr = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                # Server socket was closed.
                break

            addr_str = addr[0] if isinstance(addr, tuple) else str(addr)
            self._on_connect(client_sock, addr_str)

            # Block until this connection ends, then loop back to accept.
            self._disconnect_event.wait()

    def _on_connect(self, client_sock: socket.socket, addr: str):
        self._client_sock = client_sock
        self._client_sock.settimeout(0.5)
        self._client_addr = addr
        self._connected = True
        self._disconnect_event.clear()
        self._last_rx_time = time.time()
        self._stats["connections"] += 1
        print(f"[BluetoothServer] Connected: {addr}")

        # Drain any stale items from the TX queue.
        while not self._tx_queue_ack.empty():
            try:
                self._tx_queue_ack.get_nowait()
            except queue.Empty:
                break
        while not self._tx_queue_telem.empty():
            try:
                self._tx_queue_telem.get_nowait()
            except queue.Empty:
                break

        # Start RX and TX threads.
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="bt-rx",
        )
        self._tx_thread = threading.Thread(
            target=self._tx_loop, daemon=True, name="bt-tx",
        )
        self._rx_thread.start()
        self._tx_thread.start()

    def _on_disconnect(self, reason: str):
        if not self._connected:
            return
        self._connected = False
        self._disconnect_event.set()

        # Safety: stop motion, disable actuators.
        try:
            self._api._ctrl.stop_motion()
            self._api._ctrl.set_auger(False)
            self._api._ctrl.set_salt(False)
        except Exception as e:
            print(f"[BluetoothServer] Safety stop error: {e}")

        # Close client socket.
        if self._client_sock:
            try:
                self._client_sock.close()
            except OSError:
                pass
            self._client_sock = None

        print(f"[BluetoothServer] Disconnected: {reason}")

        try:
            from common.common_events import get_event_bus
            get_event_bus().emit(
                "controller.disconnected",
                source="bluetooth_server",
                reason=reason,
            )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # RX thread
    # ------------------------------------------------------------------

    def _rx_loop(self):
        rx_buf = bytearray()
        try:
            while not self._disconnect_event.is_set():
                sock = self._client_sock
                if sock is None:
                    break
                try:
                    chunk = sock.recv(512)
                except socket.timeout:
                    continue
                except (OSError, AttributeError):
                    break

                if not chunk:
                    # EOF — peer closed.
                    break

                rx_buf.extend(chunk)
                # Basic guard against unbounded buffer growth if a peer never
                # sends newline-terminated JSON frames.
                if len(rx_buf) > 8192:
                    rx_buf.clear()
                    self._stats["errors"] += 1
                    continue

                while True:
                    nl = rx_buf.find(b"\n")
                    if nl < 0:
                        break

                    raw_line = bytes(rx_buf[:nl])
                    del rx_buf[: nl + 1]
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue

                    self._last_rx_time = time.time()
                    self._stats["messages_rx"] += 1

                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        self._stats["errors"] += 1
                        continue

                    self._log_controller_input(msg)
                    response = self._api.dispatch(msg)
                    if response is not None:
                        try:
                            self._tx_queue_ack.put(response, timeout=0.1)
                        except queue.Full:
                            self._stats["errors"] += 1
        finally:
            if self._connected:
                self._on_disconnect("RX ended")

    # ------------------------------------------------------------------
    # TX thread
    # ------------------------------------------------------------------

    def _tx_loop(self):
        while not self._disconnect_event.is_set():
            msg = None
            try:
                msg = self._tx_queue_ack.get_nowait()
            except queue.Empty:
                try:
                    msg = self._tx_queue_telem.get(timeout=0.1)
                except queue.Empty:
                    continue

            line = json.dumps(msg, separators=(",", ":")) + "\n"
            sock = self._client_sock
            if sock is None:
                break
            try:
                sock.sendall(line.encode("utf-8"))
                self._stats["messages_tx"] += 1
            except (OSError, BrokenPipeError, AttributeError):
                if self._connected:
                    self._on_disconnect("TX socket error")
                break

    def _log_controller_input(self, msg: Dict[str, Any]) -> None:
        """Print controller command activity to the server terminal."""
        if not self._log_controller_inputs:
            return

        cmd = str(msg.get("cmd", ""))
        if not cmd:
            return

        seq = msg.get("seq")
        seq_text = f" seq={seq}" if seq is not None else ""
        now = time.time()

        if cmd == "hb":
            return
        if cmd == "status":
            return
        if cmd == "vel":
            if self._log_vel_every_s > 0.0 and (now - self._last_vel_log_s) < self._log_vel_every_s:
                return
            self._last_vel_log_s = now
            v = float(msg.get("v", 0.0))
            w = float(msg.get("w", 0.0))
            print(f"[ControllerInput] vel v={v:+.2f} w={w:+.2f}{seq_text}")
            return
        if cmd in ("auger", "salt"):
            on = bool(msg.get("on", False))
            print(f"[ControllerInput] {cmd} on={on}{seq_text}")
            return
        if cmd == "mode":
            mode = msg.get("mode", "?")
            print(f"[ControllerInput] mode={mode}{seq_text}")
            return
        if cmd == "chute":
            angle = float(msg.get("angle", 0.0))
            print(f"[ControllerInput] chute angle={angle:.1f}{seq_text}")
            return

        print(f"[ControllerInput] {cmd}{seq_text}")

    # ------------------------------------------------------------------
    # Telemetry push (asyncio)
    # ------------------------------------------------------------------

    async def _telemetry_loop(self):
        period = 1.0 / self._telemetry_hz
        while not self._stop_event.is_set():
            if self._connected:
                telem = self._api.build_telemetry()
                try:
                    self._tx_queue_telem.put_nowait(telem)
                except queue.Full:
                    pass  # Drop telemetry if queue is backed up.
            await asyncio.sleep(period)

    # ------------------------------------------------------------------
    # Watchdog (asyncio)
    # ------------------------------------------------------------------

    async def _watchdog_loop(self):
        while not self._stop_event.is_set():
            if self._connected and self._last_rx_time > 0:
                age = time.time() - self._last_rx_time
                if age > self._watchdog_timeout:
                    self._on_disconnect(f"watchdog timeout ({age:.1f}s)")
            await asyncio.sleep(1.0)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def client_address(self) -> Optional[str]:
        return self._client_addr if self._connected else None

    def get_stats(self) -> Dict[str, Any]:
        return {
            **self._stats,
            "connected": self._connected,
            "client": self._client_addr,
        }
