"""
SBCP (SnoBot Communication Protocol) v0.3.0
Lightweight error implementation. 
"""

from enum import IntEnum
from typing import Dict, NamedTuple
from dataclasses import dataclass

class FaultSeverity(IntEnum):
    INFO = 0      # Just a log entry.
    WARNING = 1   # Alert but keep operating.
    CRITICAL = 2  # Transition to FAULT state and stop motion.

@dataclass
class FaultDef:
    code: int
    name: str
    desc: str
    severity: FaultSeverity
    latched: bool = True # If true, requires manual RESET_FAULTS.

# Centralized, lightweight registry.
FAULT_CATALOG = {
    # 1-19: Safety & Power (Always Critical).
    1: FaultDef(1, "ESTOP", "Hardware E-Stop active", FaultSeverity.CRITICAL, True),
    2: FaultDef(2, "INTERLOCK", "Access panel open", FaultSeverity.CRITICAL, True),
    5: FaultDef(5, "BATT_CRITICAL", "Battery voltage below 42V", FaultSeverity.CRITICAL, True),
    
    # 20-49: Motor & Hardware (Warning or Critical).
    20: FaultDef(20, "MOTOR_TEMP", "Motor temperature high", FaultSeverity.WARNING, False),
    21: FaultDef(21, "MOTOR_OVERLOAD", "Motor current limit reached", FaultSeverity.WARNING, False),
    
    # 70-99: Communication (Software-level).
    70: FaultDef(70, "COMM_TIMEOUT", "No telemetry from MCU", FaultSeverity.CRITICAL, False),
    71: FaultDef(71, "ACK_TIMEOUT", "Command not acknowledged", FaultSeverity.WARNING, False),
}

# Faults raised/cleared by the Jetson side (not MCU telemetry).
COMM_TIMEOUT_CODE = 70
ACK_TIMEOUT_CODE = 71
JETSON_MANAGED_FAULTS = {COMM_TIMEOUT_CODE, ACK_TIMEOUT_CODE}

def get_fault(code: int) -> FaultDef:
    """Safely fetch a fault definition."""
    return FAULT_CATALOG.get(code, FaultDef(code, "UNKNOWN", "unknown fault", FaultSeverity.INFO, False))
