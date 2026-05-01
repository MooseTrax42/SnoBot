"""
Perimeter Recorder.

Records odometry waypoints while the user drives the robot in MANUAL mode.
Uses distance-based sampling and detects approximate loop closure.
"""

import math
import time
from enum import Enum
from datetime import datetime
from typing import Optional, Dict, Any

from common.common_state_machine import StateMachine
from common.common_events import get_event_bus
from sbcp.control_loop import ControlLoop
from sban.localization.odometry import Pose
from sban.perimeter.perimeter import Perimeter, Waypoint


class RecorderState(Enum):
    """States for the perimeter recorder."""
    IDLE = "idle"
    RECORDING = "recording"
    DONE = "done"


class PerimeterRecorder:
    """
    Records perimeter waypoints from odometry during manual driving.

    Uses distance-based sampling to avoid redundant points when the
    robot is stationary, and detects loop closure when the robot
    returns near its starting position.
    """

    def __init__(
        self,
        control_loop: ControlLoop,
        sample_distance_m: float = 0.3,
        closure_threshold_m: float = 0.5,
        min_waypoints_before_closure: int = 10,
    ):
        self._ctrl = control_loop
        self._sample_distance_m = sample_distance_m
        self._closure_threshold_m = closure_threshold_m
        self._min_wp_closure = min_waypoints_before_closure
        self._event_bus = get_event_bus()

        # State machine.
        self._sm: StateMachine[RecorderState] = StateMachine(
            initial_state=RecorderState.IDLE,
            allowed_transitions={
                RecorderState.IDLE: {RecorderState.RECORDING},
                RecorderState.RECORDING: {RecorderState.DONE},
                RecorderState.DONE: {RecorderState.IDLE},
            },
            name="perimeter_recorder",
            event_bus=self._event_bus,
        )

        # Recording data.
        self._waypoints: list[Waypoint] = []
        self._last_sample_pose: Optional[Pose] = None
        self._cumulative_distance: float = 0.0
        self._start_time: float = 0.0
        self._perimeter: Optional[Perimeter] = None
        self._origin_pose: Optional[Pose] = None
        self._last_reset: bool = True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start_recording(self, reset_odometry: bool = True) -> bool:
        """
        Begin recording.

        By default, resets odometry to (0, 0, 0) and records an initial
        waypoint at the origin.  If reset_odometry is False, recording
        starts from the current odometry pose (shared frame).
        """
        if self._sm.state != RecorderState.IDLE:
            return False

        origin_pose: Optional[Pose] = None
        if reset_odometry:
            # Reset odometry so the perimeter is relative to here.
            self._ctrl.reset_odometry(0.0, 0.0, 0.0)
            origin_pose = Pose(x=0.0, y=0.0, theta=0.0)
        else:
            origin_pose = self._ctrl.get_pose()
            if origin_pose is None:
                return False

        # Clear previous data.
        self._waypoints.clear()
        self._cumulative_distance = 0.0
        self._perimeter = None
        self._start_time = time.time()
        self._origin_pose = origin_pose
        self._last_reset = reset_odometry

        # Record origin waypoint.
        self._waypoints.append(Waypoint(
            x=origin_pose.x, y=origin_pose.y, heading=origin_pose.theta,
            distance=0.0, timestamp=time.time(),
        ))
        self._last_sample_pose = Pose(
            x=origin_pose.x,
            y=origin_pose.y,
            theta=origin_pose.theta,
        )

        self._sm.transition_to(RecorderState.RECORDING, trigger="start_recording")
        print("[PerimeterRecorder] Recording started — drive around the boundary")
        return True

    def stop_recording(self) -> bool:
        """Stop recording and finalize the perimeter."""
        if self._sm.state != RecorderState.RECORDING:
            return False

        self._finalize_perimeter()
        self._sm.transition_to(RecorderState.DONE, trigger="stop_recording")
        print(f"[PerimeterRecorder] Recording stopped — {len(self._waypoints)} waypoints")
        return True

    def reset(self) -> bool:
        """Return to IDLE, discarding the completed perimeter."""
        if self._sm.state != RecorderState.DONE:
            return False
        self._waypoints.clear()
        self._perimeter = None
        self._sm.transition_to(RecorderState.IDLE, trigger="reset")
        return True

    def update(self) -> Optional[Waypoint]:
        """
        Sample current pose and record a waypoint if the robot has
        traveled far enough since the last sample.

        Should be called at the control rate (~10-20 Hz).
        Returns a new Waypoint if one was recorded, None otherwise.
        """
        if self._sm.state != RecorderState.RECORDING:
            return None

        pose = self._ctrl.get_pose()
        if pose is None:
            return None

        # Distance from last sample.
        if self._last_sample_pose is not None:
            dx = pose.x - self._last_sample_pose.x
            dy = pose.y - self._last_sample_pose.y
            dist = math.sqrt(dx * dx + dy * dy)
        else:
            dist = 0.0

        if dist < self._sample_distance_m:
            return None

        # Record waypoint.
        self._cumulative_distance += dist
        wp = Waypoint(
            x=pose.x,
            y=pose.y,
            heading=pose.theta,
            distance=self._cumulative_distance,
            timestamp=time.time(),
        )
        self._waypoints.append(wp)
        self._last_sample_pose = Pose(x=pose.x, y=pose.y, theta=pose.theta)

        self._event_bus.emit(
            "perimeter.waypoint_recorded",
            source="perimeter_recorder",
            waypoint_index=len(self._waypoints) - 1,
            x=wp.x, y=wp.y,
            cumulative_distance=self._cumulative_distance,
        )

        # Check loop closure.
        if len(self._waypoints) >= self._min_wp_closure:
            if self._origin_pose is None:
                dist_to_origin = math.sqrt(pose.x ** 2 + pose.y ** 2)
            else:
                dx = pose.x - self._origin_pose.x
                dy = pose.y - self._origin_pose.y
                dist_to_origin = math.sqrt(dx * dx + dy * dy)
            if dist_to_origin <= self._closure_threshold_m:
                print(
                    f"[PerimeterRecorder] Loop closure detected — "
                    f"{dist_to_origin:.3f}m from origin"
                )
                self._event_bus.emit(
                    "perimeter.loop_closed",
                    source="perimeter_recorder",
                    closure_error_m=dist_to_origin,
                    waypoint_count=len(self._waypoints),
                )
                self._finalize_perimeter()
                self._sm.transition_to(RecorderState.DONE, trigger="loop_closure")
                return wp

        return wp

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_perimeter(self) -> Optional[Perimeter]:
        """Returns the completed Perimeter, or None if not yet done."""
        return self._perimeter

    def get_waypoint_count(self) -> int:
        return len(self._waypoints)

    def get_waypoints(self) -> list[Waypoint]:
        return list(self._waypoints)

    def get_state(self) -> RecorderState:
        return self._sm.state

    def is_recording(self) -> bool:
        return self._sm.state == RecorderState.RECORDING

    def get_stats(self) -> Dict[str, Any]:
        return {
            "state": self._sm.state.value,
            "waypoints": len(self._waypoints),
            "cumulative_distance_m": round(self._cumulative_distance, 3),
            "elapsed_s": round(time.time() - self._start_time, 1) if self._start_time else 0,
        }

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _finalize_perimeter(self):
        if not self._waypoints:
            return

        last = self._waypoints[-1]
        if self._origin_pose is None:
            closure_error = math.sqrt(last.x ** 2 + last.y ** 2)
        else:
            dx = last.x - self._origin_pose.x
            dy = last.y - self._origin_pose.y
            closure_error = math.sqrt(dx * dx + dy * dy)

        self._perimeter = Perimeter(
            waypoints=list(self._waypoints),
            created_at=datetime.now().isoformat(),
            sample_distance_m=self._sample_distance_m,
            total_distance_m=self._cumulative_distance,
            loop_closed=(closure_error <= self._closure_threshold_m),
            closure_error_m=closure_error,
            metadata={
                "robot_width_m": 0.61,
                "odometry_source": "encoder+imu",
                "recording_duration_s": round(time.time() - self._start_time, 1),
                "waypoint_count": len(self._waypoints),
                "origin": None if self._origin_pose is None else {
                    "x": round(self._origin_pose.x, 4),
                    "y": round(self._origin_pose.y, 4),
                    "theta": round(self._origin_pose.theta, 4),
                },
                "odometry_reset": self._last_reset,
            },
        )
