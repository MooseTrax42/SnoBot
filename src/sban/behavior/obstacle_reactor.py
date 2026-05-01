"""
Obstacle Reactor — reactive behavior layer.

Subscribes to OBJECT_DETECTED / OBJECT_LOST events from the vision pipeline
and drives the ControlLoop accordingly.

Decision tick runs at 10 Hz in the asyncio loop.  Event callbacks are called
from the vision thread, so the tracked-objects dict is protected by a Lock.
"""

import asyncio
import time
import threading
from typing import Optional

from common.common_events import (
    get_event_bus, EventType, Event, EventPriority,
)
from sbcp.control_loop import ControlLoop

# Behavior constants.
CRUISE_VEL = 0.4        # m/s forward when path is clear
SLOW_VEL = 0.15         # m/s when obstacle at medium range
STOP_DISTANCE = 0.6     # meters — hard stop threshold
SLOW_DISTANCE = 2.0     # meters — begin slowing
STALE_TIMEOUT = 1.0     # seconds before a track is considered gone
PERSON_SAFETY_MULT = 2.0  # distance multiplier for "person" class
TICK_HZ = 10.0          # decision rate


class ObstacleReactor:
    """
    Reactive obstacle-avoidance layer.

    Maintains a lightweight world model of tracked objects (updated via
    EventBus) and periodically decides what velocity / actuator state
    to command through the ControlLoop.
    """

    def __init__(self, ctrl: ControlLoop):
        self._ctrl = ctrl
        self._lock = threading.Lock()

        # World model: {track_id: {class_name, distance, last_seen}}
        self._tracked: dict = {}

        # EventBus subscriptions.
        bus = get_event_bus()
        self._sub_detected = bus.subscribe(
            self._on_detection,
            event_types={EventType.OBJECT_DETECTED},
        )
        self._sub_lost = bus.subscribe(
            self._on_lost,
            event_types={EventType.OBJECT_LOST},
        )

        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._last_action = ""

    # ------------------------------------------------------------------
    # EventBus callbacks (called from vision thread)
    # ------------------------------------------------------------------

    def _on_detection(self, event: Event):
        track_id = event.get("track_id")
        if track_id is None:
            return
        with self._lock:
            self._tracked[track_id] = {
                "class_name": event.get("class_name", "unknown"),
                "distance": event.get("distance", 999.0),
                "last_seen": event.get("timestamp", time.time()),
            }

    def _on_lost(self, event: Event):
        track_id = event.get("track_id")
        if track_id is None:
            return
        with self._lock:
            self._tracked.pop(track_id, None)

    # ------------------------------------------------------------------
    # Async tick loop
    # ------------------------------------------------------------------

    async def run(self):
        """Start the periodic decision loop. Call as an asyncio task."""
        self._running = True
        period = 1.0 / TICK_HZ
        print(f"[ObstacleReactor] Started ({TICK_HZ} Hz)")
        while self._running:
            self._expire_stale()
            self._decide()
            await asyncio.sleep(period)

    def stop(self):
        """Stop the reactor and unsubscribe from events."""
        self._running = False
        if self._task and not self._task.done():
            self._task.cancel()
        bus = get_event_bus()
        bus.unsubscribe(self._sub_detected)
        bus.unsubscribe(self._sub_lost)
        print("[ObstacleReactor] Stopped")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _expire_stale(self):
        now = time.time()
        with self._lock:
            stale = [
                tid for tid, t in self._tracked.items()
                if now - t["last_seen"] > STALE_TIMEOUT
            ]
            for tid in stale:
                del self._tracked[tid]

    def _decide(self):
        # Let the ControlLoop safety layer handle non-operational states.
        if not self._ctrl.is_motion_enabled():
            return

        with self._lock:
            snapshot = dict(self._tracked)

        if not snapshot:
            self._apply_cruise()
            return

        # Find closest object and its effective safety distance.
        min_effective = float("inf")
        closest_name = ""
        for t in snapshot.values():
            dist = t["distance"]
            mult = PERSON_SAFETY_MULT if t["class_name"] == "person" else 1.0
            effective = dist / mult  # shrink distance → react sooner
            if effective < min_effective:
                min_effective = effective
                closest_name = t["class_name"]

        if min_effective < STOP_DISTANCE:
            self._apply_stop(closest_name, min_effective)
        elif min_effective < SLOW_DISTANCE:
            self._apply_slow(closest_name, min_effective)
        else:
            self._apply_cruise()

    # ------------------------------------------------------------------
    # Action helpers (thin wrappers for logging + dedup)
    # ------------------------------------------------------------------

    def _apply_stop(self, name: str, dist: float):
        action = "stop"
        if self._last_action != action:
            print(f"[ObstacleReactor] STOP — {name} at {dist:.2f}m (effective)")
            self._last_action = action
        self._ctrl.stop_motion()
        self._ctrl.set_auger(False)

    def _apply_slow(self, name: str, dist: float):
        action = "slow"
        if self._last_action != action:
            print(f"[ObstacleReactor] SLOW — {name} at {dist:.2f}m (effective)")
            self._last_action = action
        self._ctrl.set_velocity(SLOW_VEL, 0.0)

    def _apply_cruise(self):
        action = "cruise"
        if self._last_action != action:
            print(f"[ObstacleReactor] CRUISE — path clear")
            self._last_action = action
        self._ctrl.set_velocity(CRUISE_VEL, 0.0)
        self._ctrl.set_auger(True)
