"""
SBCP (SnoBot Communication Protocol) v0.3.0
Robot state machine and associated enums.
"""
import time
from enum import Enum
from typing import Optional, Dict, Any, List, Set
from common.common_errors import StateError
from sbcp.commands import RobotMode, Command
from sbcp.types import RobotState
from sbcp.errors import FAULT_CATALOG, FaultSeverity, JETSON_MANAGED_FAULTS
    
class StateMachine:
    """
    Simplified state machine implementation with new command interfacing.
    """
    # Define allowed transitions.
    TRANSITIONS: Dict[RobotState, Set[RobotState]] = {
        RobotState.BOOT: {RobotState.IDLE, RobotState.FAULT},
        RobotState.IDLE: {RobotState.IDLE, RobotState.MANUAL, RobotState.AUTO, RobotState.STOPPED, RobotState.DEGRADED_COMM, RobotState.FAULT, RobotState.ESTOPPED},
        RobotState.MANUAL: {RobotState.MANUAL, RobotState.IDLE, RobotState.AUTO, RobotState.STOPPED, RobotState.DEGRADED_COMM, RobotState.FAULT, RobotState.ESTOPPED},
        RobotState.AUTO: {RobotState.AUTO, RobotState.IDLE, RobotState.MANUAL, RobotState.STOPPED, RobotState.DEGRADED_COMM, RobotState.FAULT, RobotState.ESTOPPED},
        RobotState.STOPPED: {RobotState.STOPPED, RobotState.MANUAL, RobotState.AUTO, RobotState.IDLE, RobotState.DEGRADED_COMM, RobotState.FAULT, RobotState.ESTOPPED},
        RobotState.DEGRADED_COMM: {RobotState.DEGRADED_COMM, RobotState.MANUAL, RobotState.AUTO, RobotState.STOPPED, RobotState.FAULT, RobotState.ESTOPPED},
        RobotState.FAULT: {RobotState.FAULT, RobotState.IDLE, RobotState.ESTOPPED},
        RobotState.ESTOPPED: {RobotState.ESTOPPED, RobotState.IDLE},
    }
    
    
    def __init__(self, auto_clear_non_latched: bool):
        """
        Initialize state machine.

        Args:
            auto_clear_non_latched: Clear non-latched faults when no longer in telemetry.
        """
        # Current state.
        self.state: RobotState = RobotState.BOOT

        # Fault tracking
        self._active_faults: Set[int] = set()
        self._estop_active: bool = False
        self._auto_clear_non_latched = auto_clear_non_latched

        # Recovery contexts.
        self._state_before_stopped: Optional[RobotState] = None
        self._state_before_degraded: Optional[RobotState] = None

        # Degraded communication tracking.
        self._degraded_comm_entry_time: Optional[float] = None
        
    @property
    def active_faults(self) -> Set[int]:
        """Get active fault codes."""
        return self._active_faults.copy()
    
    @property
    def estop_active(self) -> bool:
        """Check if E-stop is active."""
        return self._estop_active
    
    def is_motion_enabled(self) -> bool:
        """Check if motion is currently allowed."""
        return (self.state.allows_motion() and not self._estop_active and not self._has_blocking_fault())

    def transition_to(self, target: RobotState, trigger: str = "unknown") -> bool:
        """
        Transition to target state with logging, metrics, and events.

        Args:
            target: Target state
            trigger: What triggered this transition

        Returns:
            True if transition succeeded

        Raises:
            StateError if transition is not allowed
        """
        if target not in self.TRANSITIONS[self.state]:
            raise StateError(f"Invalid transition {self.state.value} -> {target.value}")
        
        # E-stop blocks all transitions except to ESTOPPED.
        if self._estop_active and target != RobotState.ESTOPPED:
            return False
        
        # Blocking faults prevent operational transitions.
        if self._has_blocking_fault() and target.is_operational():
            return False
        
        # Save context for recovery.
        if target == RobotState.STOPPED:
            self._state_before_stopped = self.state
        elif target == RobotState.DEGRADED_COMM:
            self._state_before_degraded = self.state
            self._degraded_comm_entry_time = time.time()

        # Perform transition.
        self.state = target

        # Clear recovery contexts when leaving.
        if self.state != RobotState.STOPPED:
            self._state_before_stopped = None
        if self.state != RobotState.DEGRADED_COMM:
            self._state_before_degraded = None
            self._degraded_comm_entry_time = None
        
        return True
    
    def update_from_telemetry(self, state: str, faults:List[int], estop: bool):
        """
        Update state machine from Arduino telemetry.

        Syncs Jetson intent with Arduino state.
    
        Args:
            state: Robot state string from telemetry
            faults: List of active fault codes
            estop: E-stop status
        """
        new_faults = set(faults)

        # Auto-clear non-latched faults if enabled.
        if self._auto_clear_non_latched:
            non_latched_faults = {
                f for f in self._active_faults
                if not self._is_latched_fault(f)
            }
            # Don't auto-clear software/Jetson-managed comm faults.
            non_latched_faults -= JETSON_MANAGED_FAULTS
            cleared_faults = non_latched_faults - new_faults
            self._active_faults -= cleared_faults

        # Add new faults from telemetry.
        for fault_code in new_faults - self._active_faults:
            self.add_fault(fault_code, auto_transition=False)

        # Update E-stop status.
        if estop != self._estop_active:
            if estop:
                self.activate_estop()
            else:
                self.deactivate_estop()

        # Sync with telemetry.
        try:
            telemetry_state = RobotState[state.upper()]
            if telemetry_state != self.state:
                # Arduino commands different state.

                if self.state == RobotState.FAULT and telemetry_state != RobotState.FAULT:
                    # Jetson detected a fault but Arduino doesn't know yet, keep FAULT state.
                    if len(self._active_faults) > 0:
                        return

                if telemetry_state != self.state:
                    if telemetry_state in self.TRANSITIONS[self.state]:
                        self.transition_to(telemetry_state, trigger="telemetry")
        except (KeyError, ValueError):
            # Invalid state from telemetry, ignore.
            pass

    def can_accept_command(self, cmd: Command) -> bool:
        """Verify that a command is allowed in the current state."""
        # If no allowed states are specified, allow in any state (including fault/estop).
        if not cmd.ALLOWED_STATES:
            return True
        if cmd.ALLOWED_STATES and self.state not in cmd.ALLOWED_STATES:
            return False
        if self.estop_active and cmd.NAME != "RESET_FAULTS":
            return False
        if self._has_blocking_fault() and cmd.NAME != "RESET_FAULTS":
            return False
        return True

    def handle_mode_command(self, mode: RobotMode) -> bool:
        """
        Handle MODE command.

        Args:
            mode: Target mode (RobotMode.IDLE/MANUAL/AUTO)

        Returns:
            True if transition succeeded
        """
        mode_to_state = {
            RobotMode.IDLE: RobotState.IDLE,
            RobotMode.MANUAL: RobotState.MANUAL,
            RobotMode.AUTO: RobotState.AUTO
        }

        target = mode_to_state.get(mode)
        if not target:
            raise ValueError(f"Invalid mode: {mode}")
        
        return self.transition_to(target, trigger=f"MODE:{mode}")
    
    def handle_stop_command(self) -> bool:
        """Returns true if transition was successful."""
        if self.state.is_operational():
            return self.transition_to(RobotState.STOPPED, trigger="STOP")
        return False
    
    def handle_resume_command(self) -> bool:
        """Returns true if transition was successful."""
        if self.state != RobotState.STOPPED:
            raise StateError("RESUME only allowed in STOPPED state", current_state=self.state)
        
        target = self._state_before_stopped or RobotState.IDLE
        return self.transition_to(target, trigger="RESUME")
    
    def add_fault(self, fault_code: int, auto_transition: bool = True) -> bool:
        """
        Add a fault to the active set.

        Args:
            fault code: Error code to add
            auto_transition: Automatically transition to FAULT if blocking

        Returns:
            True if fault was newly added
        """
        if fault_code in self._active_faults:
            return False
        
        self._active_faults.add(fault_code)

        # Auto-transition to FAULT if blocking.
        if auto_transition and self._is_blocking_fault(fault_code):
            if self.state not in (RobotState.FAULT, RobotState.ESTOPPED):
                self.transition_to(RobotState.FAULT, trigger=f"fault={fault_code}")

        return True
    
    def remove_fault(self, fault_code: int) -> bool:
        """
        Remove fault from the active set.

        Args:
            fault_code: Error code to remove

        Returns:
            True if fault was present
        """
        if fault_code not in self._active_faults:
            return False
        
        self._active_faults.remove(fault_code)
        return True
    
    def clear_faults(self, specified: Optional[List[int]] = None) -> Set[int]:
        """
        Clear faults.

        Args:
            specified: Specific codes to clear (None = clear all)

        Returns:
            Set of codes that were cleared
        """
        to_clear = specified if specified else list(self._active_faults)
        cleared = set()

        # RESET_FAULTS should clear latched faults as well.
        for code in to_clear:
            if code in self._active_faults:
                self._active_faults.remove(code)
                cleared.add(code)

        return cleared
    
    def activate_estop(self) -> bool:
        """Activate E-stop. Returns True if E-stop was newly activated."""
        if self._estop_active:
            return False
        
        self._estop_active = True
        self._active_faults.add(1)

        if self.state != RobotState.ESTOPPED:
            self.transition_to(RobotState.ESTOPPED, trigger="estop")

        return True
    
    def deactivate_estop(self) -> bool:
        """Deactivate E-stop (requires physical reset within real system)."""
        if not self._estop_active:
            return False
        
        self._estop_active = False
        self._active_faults.discard(1)
        return True
    
    def check_degraded_timeout(self, timeout: float = 30.0) -> bool:
        """
        Check if degraded communication state has timed out.

        Args:
            timeout: Timeout duration in seconds

        Returns:
            True if timed out and transitioned to FAULT state
        """
        if self.state != RobotState.DEGRADED_COMM or not self._degraded_comm_entry_time:
            return False
        
        elapsed = time.time() - self._degraded_comm_entry_time
        if elapsed >= timeout:
            self.add_fault(70) # COMM_WATCHDOG_TIMEOUT
            return True
        return False
        
    def _has_blocking_fault(self) -> bool:
        """Check if any active fault is blocking."""
        return any(self._is_blocking_fault(code) for code in self._active_faults)
    
    @staticmethod
    def _is_blocking_fault(code: int) -> bool:
        """Query catalog to see if this fault forces a FAULT state."""
        fault = FAULT_CATALOG.get(code)
        if not fault:
            return False
        return fault.severity == FaultSeverity.CRITICAL
    
    @staticmethod
    def _is_latched_fault(code: int) -> bool:
        """Query catalog to see if this fault requires manual reset."""
        fault = FAULT_CATALOG.get(code)
        return fault.latched if fault else False

    def get_status(self) -> Dict[str, Any]:
        """Returns comprehensive status dictionary."""
        return {
            "state": self.state.value,
            "estop_active": self._estop_active,
            "active_faults": list(self._active_faults),
            "motion_enabled": self.is_motion_enabled(),
            "is_operational": self.state.is_operational(),
            "auto_clear_enabled": self._auto_clear_non_latched,
        }
