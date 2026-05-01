"""
SnoBot Common Types
Shared type definitions, enums, and constants for modules.
"""

from enum import Enum, IntEnum
from dataclasses import dataclass
from typing import Optional, Tuple, Any, Dict, TypedDict

# Enumerations.
class ErrorSeverity(Enum):
    """
    Error severity levels.
    
    Used across all SnoBot sytems for consistent error classification.
    """
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"
    
    def is_blocking(self) -> bool:
        """
        Check if this severity blocks operations.
        
        Returns:
            True if ERROR or CRITICAL
        """
        return self in (ErrorSeverity.ERROR, ErrorSeverity.CRITICAL)
    
    def is_critical(self) -> bool:
        """
        Check if this is a critical severity.
        
        Returns:
            True if CRITICAL
        """
        return self == ErrorSeverity.CRITICAL
    
class LogLevel(Enum):
    """
    Logging levels.
    
    Used across all SnoBot systems.
    """
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"
    
    @classmethod
    def from_string(cls, level_str: str) -> 'LogLevel':
        """
        Create LogLevel from string.
        
        Args:
            level_str: Level string (case insensitive)
            
        Returns:
            LogLevel enum
            
        Raises: ValueError: If invalid level string
        """
        try:
            return cls[level_str.upper()]
        except KeyError:
            raise ValueError(f"Invalid log level: {level_str}")
        
class LogFormat(Enum):
    """
    Log output formats.
    
    Used across all SnoBot systems.
    """
    TEXT = "text"
    JSON = "json"
    CSV = "csv"
    
    @classmethod
    def from_string(cls, format_str: str) -> 'LogFormat':
        """
        Create LogFormat from string.
        
        Args:
            format_str: Format string (case insensitive)
            
        Returns:
            LogFormat enu,
            
        Raises:
            ValueError: If invalid format string
        """
        try:
            return cls[format_str.upper()]
        except KeyError:
            raise ValueError(f"Invalid log format: {format_str}")
        
class SystemState(Enum):
    """
    Generic system state.
    
    Can be used as a base for specific system states (RobotState, CameraState, etc.)
    """
    STOPPED = "STOPPED"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    FAULT = "FAULT"
    
    def is_operational(self) -> bool:
        """
        Check if state represents operational status.
        
        Returns:
            True if STARTING or RUNNING
        """
        return self in (SystemState.STARTING, SystemState.RUNNING)
    
    def is_stopped(self) -> bool:
        """
        Check if state is stopped.
        
        Returns:
            True if STOPPED or STOPPING
        """
        return self in (SystemState.STOPPED, SystemState.STOPPING)
    
# Data structures.
class Timestamp:
    """
    Timestamp with both system time and frame time.
    
    Useful for tracking latency and synchronization.
    """
    system_time_s: float
    frame_time_s: Optional[float] = None
    
    def age_ms(self, current_time_s: float) -> float:
        """
        Calculate age of timestamp.
        
        Args:
            current_time_s: Current system time.
            
        Returns:
            Age in milliseconds
        """
        return (current_time_s - self.system_time_s) * 1000.0
    
    def latency_ms(self) -> Optional[float]:
        """
        Calculate latency if frame time is available.
        
        Returns:
            Latency in milliseconds, or None if no frame time.
        """
        if self.frame_time_s is None:
            return None
        return (self.system_time_s - self.frame_time_s) * 1000.0

class VersionInfo:
    """
    Version information.
    
    Common to protocol versions and system versions.
    """
    major: int
    minor: int
    patch: int
    
    @classmethod
    def from_string(cls, version_str: str) -> 'VersionInfo':
        """
        Parse version from string.
        
        Args:
            version_str: Version string (e.g., "0.2.0")
            
        Returns:
            VersionInfo instance
            
        Raises:
            ValueError: If invalid version string
        """
        try:
            parts = version_str.split('.')
            if len(parts) != 3:
                raise ValueError("Version musst be in format X.Y.Z")
            return cls(int(parts[0]), int(parts[1]), int(parts[2]))
        except (ValueError, IndexError) as e:
            raise ValueError(f"Invalid version string '{version_str}': {e}")
        
    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.major}.{self.minor}.{self.patch}"
    
    def __lt__(self, other: 'VersionInfo') -> bool:
        """
        Less-than comparison.
        """
        return (self.major, self.minor, self.patch) < (other.major, other.minor, other.patch)
    
    def __le__(self, other: 'VersionInfo') -> bool:
        """
        Less-than-or-equal comparison.
        """
        return (self.major, self.minor, self.patch) <= (other.major, other.minor, other.patch)
    
    def __gt__(self, other: 'VersionInfo') -> bool:
        """
        Greater-than comparison.
        """
        return (self.major, self.minor, self.patch) > (other.major, other.minor, other.patch)
    
    def __ge__(self, other: 'VersionInfo') -> bool:
        """
        Greater-than-or-equal comparison.
        """
        return (self.major, self.minor, self.patch) >= (other.major, other.minor, other.patch)
    
    def __eq__(self, other: 'VersionInfo') -> bool:
        """
        Equality comparison.
        """
        if not isinstance(other, VersionInfo):
            return False
        return (self.major, self.minor, self.patch) == (other.major, other.minor, other.patch)
    
    def is_compatible(self, other: 'VersionInfo') -> bool:
        """
        Check if versions are compatible (same major version)
        
        Args:
            other: Other version to check
            
        Returns:
            True if major versions match
        """
        return self.major == other.major
    
@dataclass
class Statistics:
    """
    Base statistics structure.
    
    Common fields for various statistics tracking.
    """
    count: int = 0
    errors: int = 0
    warnings: int = 0
    strat_time_s: Optional[float] = None
    
    def increment(self):
        """
        Increment count.
        """
        self.count += 1
        
    def record_error(self):
        """
        Record an error.
        """
        self.errors += 1
        
    def record_warning(self):
        """
        Record a warning.
        """
        self.warnings += 1
        
    def get_error_rate(self) -> float:
        """
        Calculate error rate.
        
        Returns:
            errors / count, or 0.0 if no counts
        """
        return self.errors / self.count if self.count > 0 else 0.0
    
    def get_success_rate(self) -> float:
        """
        Calculate success rate.
        
        Returns:
            (count - errors) / count, or 1.0 if no counts
        """
        return (self.count - self.errors) / self.count if self.count > 0 else 1.0
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert to dictionary.
        
        Returns:
            Dictionary representation
        """
        return {
            "count": self.count,
            "errors": self.errors,
            "warnings": self.warnings,
            "error_rate": round(self.get_error_rate(), 4),
            "success_rate": round(self.get_success_rate(), 4)
        }
        
# Utility functions.
def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp a value between min and max.
    
    Args:
        value: Value to clamp
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Clamped value
    """
    return max(min_val, min(max_val, value))

def clamp_int(value: int, min_val: int, max_val: int) -> int:
    """
    Clamp an integer value between min and max.
    
    Args:
        value: Value to clamp
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Clamped value
    """
    return max(min_val, min(max_val, value))

def normalize(value: float, min_val: float, max_val: float) -> float:
    """
    Normalize a value to [0, 1] range.
    
    Args:
        value: Value to normalize
        min_val: Minimum value of range
        max_val: Maximum value of range
        
    Returns:
        Normalized value in [0, 1]
    """
    if max_val == min_val:
        return 0.0
    return clamp((value - min_val) / (max_val - min_val), 0.0, 1.0)

def lerp(a: float, b: float, t: float) -> float:
    """
    Linear interpolation between two values.
    
    Args:
        a: Start value
        b: End value
        t: Interpolation factor [0, 1]

    Returns:
        Interpolated value
    """
    return a + (b - a) * clamp(t, 0.0, 1.0)

def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    """
    Map value from one range to another.
    
    Args:
        value: Input value
        in_min: Input range minimum
        in_max: Input range maximum
        out_min: Output range minimum
        out_max: Output range maximum
        
    Returns:
        Mapped value
    """
    # Normalize to [0, 1]
    normalized = normalize(value, in_min, in_max)
    # Map to output range.
    return lerp(out_min, out_max, normalized)

# Common timeout values (seconds).
DEFAULT_TIMEOUT_S = 1.0
SHORT_TIMEOUT_S = 0.1
LONG_TIMEOUT_S = 5.0

# Common buffer sizes.
DEFAULT_QUEUE_SIZE = 10
SMALL_QUEUE_SIZE = 2
LARGE_QUEUE_SIZE = 100

# Common retry values.
DEFAULT_MAX_RETRIES = 3
MAX_CONSECUTIVE_FAILURES = 10