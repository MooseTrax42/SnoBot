"""
SBCP (SnoBot Communication Protocol) v0.3.0
Simplified type definitions. Fuck.
"""

from enum import Enum, IntEnum
from dataclasses import dataclass
from typing import Optional
from common.common_types import ErrorSeverity

# Protocol information.
PROTOCOL_VERSION = "0.3.0"
PROTOCOL_MIN_VERSION = "0.3.0"
PROTOCOL_MAX_VERSION = "0.3.x"

# Watchdog timeouts.
WATCHDOG_GRACE_PERIOD_MS = 2000
WATCHDOG_TIMEOUT_MS = 500

# Motion control timing.
RAMP_DOWN_TIME_MS = 500

# Machine limits.
MAX_LINEAR_VELOCITY = 1.5   # m/s.
MAX_ANGULAR_VELOCITY = 3.14 # rad/s.

# Battery thresholds (volts).
BATTERY_CRITICAL_V = 43.0
BATTERY_WARNING_V = 44.0

# Temperature thresholds (Celsius).
TEMP_WARNING_C = 60.0
TEMP_CRITICAL_C = 75.0

# Communication thresholds.
PACKET_LOSS_DEGRADED_PERCENT = 10.0
DEGRADED_COMM_DURATION_S = 3.0
COMM_FAULT_DURATION_S = 30.0

# Sequence number handling.
SEQUENCE_NUMBER_MIN = 0
SEQUENCE_NUMBER_MAX = 65535  # uint16_t
SEQUENCE_NUMBER_WRAP = SEQUENCE_NUMBER_MAX + 1

# Useful buffer things.
ARDUINO_BUFFER_BYTES = 1024

# Timestamp handling.
TIMESTAMP_MAX = 4294967295  # uint32_t max
TIMESTAMP_WRAP = TIMESTAMP_MAX + 1

def normalize_sequence_number(seq: int) -> int:
    """
    Normalize sequence number to valid range with wraparound.
    
    Args:
        seq: Sequence number to normalize
        
    Returns:
        Normalized sequence number in [0, 65535]
    """
    return seq % SEQUENCE_NUMBER_WRAP


def sequence_number_delta(current: int, previous: int) -> int:
    """
    Calculate delta between sequence numbers accounting for wraparound.
    
    Args:
        current: Current sequence number
        previous: Previous sequence number
        
    Returns:
        Signed delta (-32768 to 32767)
    """
    delta = (current - previous) % SEQUENCE_NUMBER_WRAP
    if delta > 32768:
        delta -= SEQUENCE_NUMBER_WRAP
    return delta

class RobotState(Enum):
    """
    Robot machine states. 
    """
    BOOT = "BOOT"
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    STOPPED = "STOPPED"
    DEGRADED_COMM = "DEGRADED_COMM"
    FAULT = "FAULT"
    ESTOPPED = "ESTOPPED"

    def allows_motion(self) -> bool:
        """Returns True if motion commands are allowed in this state."""
        return self in (RobotState.MANUAL, RobotState.AUTO, RobotState.DEGRADED_COMM)
    
    def is_operational(self) -> bool:
        """Returns True if robot is in an operational mode."""
        return self in (RobotState.MANUAL, RobotState.AUTO, RobotState.DEGRADED_COMM)

# Compact telemetry state mapping (MCU enum order).
ROBOT_STATE_BY_ID = {
    0: RobotState.BOOT,
    1: RobotState.IDLE,
    2: RobotState.MANUAL,
    3: RobotState.AUTO,
    4: RobotState.STOPPED,
    5: RobotState.DEGRADED_COMM,
    6: RobotState.FAULT,
    7: RobotState.ESTOPPED,
}

ROBOT_STATE_ID_BY_NAME = {state.value: state_id for state_id, state in ROBOT_STATE_BY_ID.items()}

def robot_state_from_id(state_id: int) -> Optional[RobotState]:
    """Convert compact numeric state to RobotState."""
    try:
        return ROBOT_STATE_BY_ID.get(int(state_id))
    except (TypeError, ValueError):
        return None

def robot_state_id_from_name(name: str) -> Optional[int]:
    """Convert state name to compact numeric state."""
    if not name:
        return None
    return ROBOT_STATE_ID_BY_NAME.get(str(name).upper())
