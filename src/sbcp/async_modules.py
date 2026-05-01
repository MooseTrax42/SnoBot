"""
SBCP (SnoBot Communication Protocol) v0.3.0
Async modules for rate-independent communication.
"""

import asyncio
import json
import time
from collections import deque
from typing import Optional, Callable, Any, Dict, List
from sbcp.transport import TransportBase
from sbcp.schema import parse_envelope

class AsyncPublisher:
    """
    Publishes data at a configurable rate.

    Maintains latest data and updates periodically.
    """
    def __init__(self, transport: TransportBase, rate_hz: float = 20.0):
        self.transport = transport
        self.rate_hz = rate_hz
        self.period = 1 / rate_hz

        self._latest_data: Optional[Dict[str, Any]] = None
        self._pending_send: bool = False
        self._paused: bool = False
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._callbacks: List[Callable[[Dict[str, Any]], Any]] = []

    def update_data(self, data: Dict[str, Any]):
        """Update data to publish."""
        self._latest_data = data
        self._pending_send = True

    def register_callback(self, callback: Callable[[Dict[str, Any]], Any]):
        """Register a callback triggered after sending data."""
        self._callbacks.append(callback)
    
    def pause(self):
        """Pause sending without stopping the run loop."""
        self._paused = True
    
    def resume(self):
        """Resume sending after a pause."""
        self._paused = False

    async def run(self):
        """Async publishing loop."""
        self._running = True
        loop = asyncio.get_event_loop()

        while self._running:
            tasks = []
            for cb in self._callbacks:
                try:
                    if asyncio.iscoroutinefunction(cb):
                        tasks.append(asyncio.create_task(cb(self._latest_data)))
                    else:
                        cb(self._latest_data)
                except Exception as e:
                    print(f"AsyncPublisher callback error: {e}")
            
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)
            
            # Then send once if data was updated.
            if self._pending_send and self._latest_data is not None and not self._paused:
                try:
                    self.transport.send(self._latest_data)
                    self._pending_send = False
                except Exception as e:
                    print(f"AsyncPublisher send error: {e}")
            
            await asyncio.sleep(self.period)

    async def stop(self):
        """Stop the async loop."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    def get_latest(self) -> Optional[Dict[str, Any]]:
        """Return the latest data being sent."""
        return self._latest_data

class AsyncSubscriber:
    """
    Subscribes to data stream and maintains latest.

    Can be polled at any rate, always returns freshest data.
    """
    def __init__(self, transport: TransportBase, rate_hz: float = 50.0):
        self.transport = transport
        self.rate_hz = rate_hz
        self.period = 1 / rate_hz

        self._latest_data: Optional[Dict[str, Any]] = None
        self._data_timestamp: float = 0
        self._callbacks: List[Callable[[Dict[str, Any]], None]] = []

        self._running = False
        self._task: Optional[asyncio.Task] = None

        # Error detection.
        self._error_present = False
        self._messages_received = 0
        self._last_rx_error_time = 0.0
        self._rx_error_log_interval_s = 1.0

    async def run(self):
        self._running = True
        loop = asyncio.get_event_loop()

        while self._running:
            tasks = []
            while True:
                try:
                    msg = self.transport.recv_nowait()
                except Exception as e:
                    now = time.time()
                    if now - self._last_rx_error_time >= self._rx_error_log_interval_s:
                        print(f"AsyncSubscriber recv error: {e}")
                        self._last_rx_error_time = now
                    break
                if not msg:
                    break

                self._latest_data = msg
                # Use monotonic wall-clock so age checks are safe from non-async
                # worker threads (e.g., Bluetooth RX thread serving STATUS).
                self._data_timestamp = time.monotonic()
                self._error_present = self.has_error()
                self._messages_received += 1

                # Trigger callbacks concurrently.
                for cb in self._callbacks:
                    try:
                        if asyncio.iscoroutinefunction(cb):
                            tasks.append(asyncio.create_task(cb(msg)))
                        else:
                            cb(msg)
                    except Exception as e:
                        print(f"AsyncSubscriber callback error: {e}")

            if tasks:
                # Run async callbacks concurrently but don't block loop.
                await asyncio.gather(*tasks, return_exceptions=True)

            # Apply rate limiting.
            await asyncio.sleep(self.period)

    async def stop(self):
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    def get_latest(self) -> Optional[Dict[str, Any]]:
        """Get latest recieved data (non-blocking)."""
        return self._latest_data
    
    def get_age(self) -> float:
        """Return age in seconds since last recieved message."""
        if self._data_timestamp == 0:
            return float('inf')
        return time.monotonic() - self._data_timestamp
    
    def has_error(self) -> bool:
        """Check if error is set in latest data."""
        if self._latest_data:
            try:
                msg_type, payload, _meta = parse_envelope(self._latest_data)
            except ValueError:
                return False
            if msg_type == "FAULT":
                return True
            return payload.get("error", False) or payload.get("fault", False)
        return False
    
    def register_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """Register callback for new data."""
        self._callbacks.append(callback)

    def get_stats(self) -> Dict[str, Any]:
        """Return subscriber stats."""
        return {
            "messages_received": self._messages_received,
            "latest_age_s": self.get_age(),
        }

class AsyncCommManager:
    """
    Helper class to manage multiple async publishers/subscribers.

    Provides start/stop for all loops and ensures tasks are properly cancelled.
    """
    def __init__(self, min_send_interval_ms: float = 20.0):
        self._tasks: List[asyncio.Task] = []
        self._modules: List[Any] = []
        self.min_send_interval = min_send_interval_ms / 1000.0
        self._last_send_time = 0.0

    def add_module(self, module: Any):
        """Add a publisher or subscriber."""
        self._modules.append(module)

    async def start_all(self):
        """Start all modules' run loops."""
        for mod in self._modules:
            task = asyncio.create_task(mod.run())
            self._tasks.append(task)

    async def stop_all(self):
        """Stop all modules and cancel their tasks."""
        for mod in self._modules:
            await mod.stop()

        # Cancel lingering tasks just in case.
        for task in self._tasks:
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._tasks.clear()

    def wrap_transport(self, transport):
        """Wrap transport.send() to enforce spacing."""
        original_send = transport.send

        def coordinated_send(message):
            now = time.time()
            wait = self.min_send_interval - (now - self._last_send_time)
            if wait > 0:
                time.sleep(wait)

            result = original_send(message)
            self._last_send_time = time.time()
            return result
        
        transport.send = coordinated_send

