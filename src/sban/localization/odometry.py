"""
SBAN (SnoBot Autonomous Navigation) v0.1.0

Odometry estimator for differential drive robot.
Uses wheel encoders and IMU for pose estimation (x, y, theta).
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple
import yaml

@dataclass
class Pose:
    """Robot pose in 2D."""
    x: float = 0.0      # meters.
    y: float = 0.0      # meters.
    theta: float = 0.0  # radians.

    def __repr__(self) -> str:
        return f"Pose(x={self.x:.3f}m, y={self.y:.3f}m, theta={math.degrees(self.theta):.1f}°)"

class OdometryEstimator:
    """
    Estimates robot pose using differential drive kinematics.

    Integrates wheel encoder data with optional IMU heading correction.
    Uses simple dead reckoning, no Kalman filtering in initial implementation.
    """

    def __init__(self, config_path: str = "data/sban/robot_params.yaml"):
        """
        Initialize odometry estimator.

        Args:
            config_path: Path to robot parameters YAML file
        """
        # Load robot parameters.
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        robot_params = config['robot']
        odom_params = config['odometry']

        # Robot geometry.
        self.wheelbase_m = robot_params['wheelbase_m']
        self.wheel_radius_m = robot_params['wheel_radius_m']
        self.encoder_ticks_per_rev = robot_params['encoder_ticks_per_rev']
        self.gear_ratio = robot_params['gear_ratio']

        # Calculate meters per tick.
        # Distance per wheel revolution = 2 * pi * radius.
        # Ticks per wheel revolution = encoder_ticks_per_rev * gear_ratio (depending on encoder placement).
        ticks_per_wheel_rev = self.encoder_ticks_per_rev#  * self.gear_ratio
        meters_per_wheel_rev = 2.0 * math.pi * self.wheel_radius_m
        self.meters_per_tick = meters_per_wheel_rev / ticks_per_wheel_rev

        # Odometry calibration.
        self.left_wheel_scale = odom_params.get('left_wheel_scale', 1.0)
        self.right_wheel_scale = odom_params.get('right_wheel_scale', 1.0)

        # IMU fusion parameters.
        self.use_imu_heading = odom_params.get('use_imu_heading', True)
        self.imu_weight = odom_params.get('imu_weight', 0.3)

        # Current pose.
        self._pose = Pose()

        # Previous encoder counts.
        self._last_left_ticks: Optional[int] = None
        self._last_right_ticks: Optional[int] = None

        # Previous IMU yaw (in radians).
        self._last_imu_yaw_rad: Optional[float] = None

        # Timestamps for velocity estimation (future use).
        self._last_update_time: Optional[float] = None

        # Statistics.
        self.total_distance_m = 0.0
        self.update_count = 0

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """
        Reset pose to specified position.

        Args:
            x: X position in meters
            y: Y position in meters
            theta: Heading in radians
        """
        self._pose.x = x
        self._pose.y = y
        self._pose.theta = theta

        # Clear previous encoder counts to force re-initialization.
        self._last_left_ticks = None
        self._last_right_ticks = None
        self._last_imu_yaw_rad = None
        self._last_update_time = None

        # Reset statistics.
        self.total_distance_m = 0.0
        self.update_count = 0

    def update(self, left_ticks: int, right_ticks: int, dt: Optional[float] = None) -> Pose:
        """
        Update pose estimate from encoder readings.

        Args:
            left_ticks: Current left encoder count
            right_ticks: Current right encoder count
            dt: Time step in seconds (optional, for velocity estimation)

        Returns:
            Updated pose
        """
        # Initialize on first update.
        if self._last_left_ticks is None or self._last_right_ticks is None:
            self._last_left_ticks = left_ticks
            self._last_right_ticks = right_ticks
            self._last_update_time = time.time()
            return self._pose

        # Calculate tick deltas.
        delta_left_ticks = self._handle_encoder_wraparound(left_ticks, self._last_left_ticks)
        delta_right_ticks = self._handle_encoder_wraparound(right_ticks, self._last_right_ticks)

        # Convert to distances.
        delta_left_m = delta_left_ticks * self.meters_per_tick * self.left_wheel_scale
        delta_right_m = delta_right_ticks * self.meters_per_tick * self.right_wheel_scale

        # Differential drive kinematics.
        # Average distance traveled by robot center.
        delta_s = (delta_left_m + delta_right_m) / 2.0

        # Change in heading (from encoder differential).
        delta_theta_encoder = (delta_right_m - delta_left_m) / self.wheelbase_m

        # Update pose using encoder-based heading change.
        # Use midpoint method for better accuracy on curves.
        theta_mid = self._pose.theta + delta_theta_encoder / 2.0

        self._pose.x += delta_s * math.cos(theta_mid)
        self._pose.y += delta_s * math.sin(theta_mid)
        self._pose.theta += delta_theta_encoder

        # Normalize theta to [-pi, pi].
        self._pose.theta = self._normalize_angle(self._pose.theta)

        # Update tracking.
        self.total_distance_m += abs(delta_s)
        self.update_count += 1

        # Store current values for next update.
        self._last_left_ticks = left_ticks
        self._last_right_ticks = right_ticks
        self._last_update_time = time.time()

        return self._pose

    def update_imu(self, yaw_deg: float, valid: bool = True) -> Pose:
        """
        Update heading estimate from IMU.

        Fuses IMU yaw with encoder-based heading for improved accuracy.

        Args:
            yaw_deg: IMU yaw angle in degrees
            valid: Whether IMU reading is valid

        Returns:
            Updated pose
        """
        if not self.use_imu_heading or not valid:
            return self._pose

        # Convert to radians.
        yaw_rad = math.radians(yaw_deg)

        # Initialize on first IMU update.
        if self._last_imu_yaw_rad is None:
            self._last_imu_yaw_rad = yaw_rad
            # Set initial heading from IMU.
            self._pose.theta = yaw_rad
            return self._pose

        # Calculate IMU delta (handle wraparound).
        delta_yaw_imu = self._normalize_angle(yaw_rad - self._last_imu_yaw_rad)

        # Fuse IMU delta with current heading. Weighted correction. 
        encoder_heading = self._pose.theta
        imu_heading = yaw_rad

        # Compute shortest angualr distance between headings.
        heading_error = self._normalize_angle(imu_heading - encoder_heading)

        # Apply weighted correction to pull encoder heading to IMU heading.
        self._pose.theta = encoder_heading + heading_error * self.imu_weight

        # Normalize to [-pi, pi].
        self._pose.theta = self._normalize_angle(self._pose.theta)

        # Store for next update.
        self._last_imu_yaw_rad = yaw_rad

        return self._pose

    def get_pose(self) -> Pose:
        """Get current pose estimate."""
        return self._pose

    def get_position(self) -> Tuple[float, float]:
        """Get current position (x, y) in meters."""
        return (self._pose.x, self._pose.y)

    def get_heading(self) -> float:
        """Get current heading in radians."""
        return self._pose.theta

    def get_heading_degrees(self) -> float:
        """Get current heading in degrees."""
        return math.degrees(self._pose.theta)

    def get_stats(self) -> dict:
        """Get odometry statistics."""
        return {
            "total_distance_m": self.total_distance_m,
            "update_count": self.update_count,
            "current_pose": str(self._pose),
            "meters_per_tick": self.meters_per_tick,
            "last_update_time": self._last_update_time
        }

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    @staticmethod
    def _handle_encoder_wraparound(current: int, previous: int, max_val: int = 214783647) -> int:
        """
        Handle encoder wraparound for 32-bit integers. Most definitely not needed but included anyways.

        Args:
            current: Current encoder count
            previous: Previous encoder count
            max_val: Maximum value before wraparound

        Returns:
            Corrected delta that accounts for wraparound
        """
        delta = current - previous

        if delta > max_val / 2:
            # Negative wrap.
            delta -= (max_val + 1)
        elif delta < -max_val / 2:
            # Positive wrap.
            delta += (max_val + 1)

        return delta
