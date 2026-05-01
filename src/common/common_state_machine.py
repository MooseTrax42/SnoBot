"""
SnoBot Common State Machine
Generic state machine framework with tranitions, callbacks, and history.

Provides base state machine functionality for:
- SBCP client state management
- SBVS processing states
- Navigation states
- Any other stateful components
"""

from typing import TypeVar, Generic, Set, Optional, Callable, List, Dict, Any, Type
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from threading import RLock
from collections import deque
import time

from common.common_types import ErrorSeverity
from common.common_metrics import MetricCollector

# Type variable for state enum.
StateType = TypeVar('StateType', bound=Enum)

@dataclass
class Transition(Generic[StateType]):
    """
    Represents a state transition.
    """
    from_state: StateType
    to_state: StateType
    timestamp: datetime
    trigger: str
    metadata: Optional[Dict[str, Any]] = None
    duration_sec: float = 0.0 # Time spent in from_state.

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "from_state": self.from_state.value if isinstance(self.from_state, Enum) else str(self.from_state),
            "to_state": self.to_state.value if isinstance(self.to_state, Enum) else str(self.to_state),
            "timestamp": self.timestamp.isoformat(),
            "trigger": self.trigger,
            "metadata": self.metadata,
            "duration_sec": self.duration_sec
        }
    
@dataclass
class StateInfo(Generic[StateType]):
    """
    Information about the current state.
    """
    state: StateType
    entered_at: datetime
    previous_state: Optional[StateType] = None
    transition_count: int = 0

    def time_in_state(self) -> float:
        """
        Get time spend in current state (seconds).
        """
        return (datetime.now() - self.entered_at).total_seconds()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "state": self.state.value if isinstance(self.state, Enum) else str(self.state),
            "entered_at": self.entered_at.isoformat(),
            "time_in_state_sec": self.time_in_state(),
            "previous_state": self.previous_state.value if self.previous_state and isinstance(self.previous_state, Enum) else str(self.previous_state) if self.previous_state else None,
            "transition_count": self.transition_count
        }
    
# Callback type definitions.
TransitionCallback = Callable[[StateType, StateType, str], None]
StateEnterCallback = Callable[[StateType, StateType], None]
StateExitCallback = Callable[[StateType, StateType], None]
GuardCallback = Callable[[StateType, StateType], bool]

class StateMachineError(Exception):
    """
    State machine related errors.
    """
    pass

class InvalidTransitionError(Exception):
    """
    Attempted transition is not allowed.
    """
    pass

class StateMachine(Generic[StateType]):
    """
    Generic state machine with transition validation and callbacks.

    Provides:
    - Allowed transition validation
    - State entry/exit callbacks
    - Transition guards (conditions)
    - Transition history tracking
    - Metrics and event integration
    """

    def __init__(
        self, 
        initial_state: StateType,
        allowed_transitions: Optional[Dict[StateType, Set[StateType]]] = None,
        name: str = "",
        max_history: int = 100,
        metrics_collector: Optional[MetricCollector] = None,
        event_bus = None
    ):
        """
        Initialize state machine.

        Args:
            initial_state: Starting state
            allowed_transitions: Dict of state -> set of allowed next states
            name: State machine name for identification
            max_history: Maximum transitions to keep in history
            metrics_collectos: Optional metrics integration
            event_bus: Optional event bus integration
        """
        self.name = name or f"state_machine_{id(self)}"
        self.metrics = metrics_collector
        self.events = event_bus

        # State tracking.
        self._current_state = initial_state
        self._state_info = StateInfo(state=initial_state, entered_at=datetime.now(), previous_state=None, transition_count=0)
        self._lock = RLock()

        # Allowed transitions (None = all allowed).
        self._allowed_transitions = allowed_transitions or {}

        # Transition history.
        self._max_history = max_history
        self._history: deque[Transition[StateType]] = deque(maxlen=max_history)

        # Callbacks.
        self._transition_callbacks: List[TransitionCallback] = []
        self._enter_callbacks: Dict[StateType, List[StateEnterCallback]] = {}
        self._exit_callbacks: Dict[StateType, List[StateExitCallback]] = {}
        self._guards: Dict[tuple[StateType, StateType], List[GuardCallback]] = {}

        # Statistics.
        self._state_durations: Dict[StateType, List[float]] = {}
        self._transition_counts: Dict[tuple[StateType, StateType], int] = {}

    @property
    def state(self) -> StateType:
        """
        Get current state.
        """
        with self._lock:
            return self._current_state
        
    @property
    def state_info(self) -> StateInfo[StateType]:
        """
        Get current state information.
        """
        with self._lock:
            return self._state_info
        
    def transition_to(
        self, 
        new_state: StateType,
        trigger: str = "manual",
        metadata: Optional[Dict[str, Any]] = None,
        force: bool = False
    ) -> bool:
        """
        Transition to a new state.

        Args:
            new_state: Target state
            trigger: What triggered this transition
            metadata: Optional metadata about transition
            force: Force transition even if not in allowed_transitions

        Returns:
            True if transition succeeded

        Raises:
            InvalidTransitionError: If transition not allowed
        """
        with self._lock:
            old_state = self._current_state

            # Check if already in target state.
            if old_state == new_state:
                return False
            
            # Validate transition.
            if not force and not self._is_transition_allowed(old_state, new_state):
                raise InvalidTransitionError(f"Transition from {old_state} to {new_state} is not allowed")
            
            # Check guards.
            if not self._check_guards(old_state, new_state):
                return False
            
            # Calculate time in old state.
            duration = self._state_info.time_in_state()

            # Call exit callbacks for old state.
            self._call_exit_callbacks(old_state, new_state)

            # Perform transition.
            self._current_state = new_state
            self._state_info = StateInfo(state=new_state, entered_at=datetime.now(), previous_state=old_state, transition_count=self._state_info.transition_count + 1)

            # Record transition.
            transition = Transition(from_state=old_state, to_state=new_state, timestamp=datetime.now(), trigger=trigger, metadata=metadata, duration_sec=duration)
            self._history.append(transition)

            # Update statistics.
            if old_state not in self._state_durations:
                self._state_durations[old_state] = []
            self._state_durations[old_state].append(duration)

            trans_key = (old_state, new_state)
            self._transition_counts[trans_key] = self._transition_counts.get(trans_key, 0) + 1

            # Record metrics.
            if self.metrics:
                self.metrics.record(f"state_machine.{self.name}.{old_state.value}.duration_sec", duration)
                self.metrics.increment(f"state_machine.{self.name}.transitions.{old_state.value}_to_{new_state.value}")
                self.metrics.increment(f"state_machine.{self.name}.transition_count")

            # Emit event.
            if self.events:
                from common.common_events import EventType, EventPriority
                self.events.emit(
                    EventType.STATE_CHANGED,
                    source=f"state_machine.{self.name}",
                    priority=EventPriority.NORMAL,
                    state_machine=self.name,
                    old_state=old_state.value if isinstance(old_state, Enum) else str(old_state),
                    new_state=new_state.value if isinstance(new_state, Enum) else str(new_state),
                    trigger=trigger,
                    duration_sec=duration,
                    metadata=metadata
                )
            
            # Call enter callbacks for new state.
            self._call_enter_callbacks(old_state, new_state)

            # Call transition callbacks.
            self._call_transition_callbacks(old_state, new_state, trigger)

            return True
        
    def can_transition_to(self, new_state: StateType) -> bool:
        """
        Check if transition to new state is allowed.

        Args:
            new_state: Target state

        Returns:
            True if transition is allowed
        """
        with self._lock:
            if self._current_state == new_state:
                return False
            
            if not self._is_transition_allowed(self._current_state, new_state):
                return False
            
            return self._check_guards(self._current_state, new_state)
        
    def is_in_state(self, *states: StateType) -> bool:
        """
        Check if current state is one of the given states.

        Args:
            *states: States to check

        Returns:
            True if in any of the given states
        """
        return self._current_state in states
    
    def add_allowed_transition(self, from_state: StateType, to_state: StateType):
        """
        Add an allowed transition.

        Args:
            from_state: Source state
            to_state: Target state
        """
        with self._lock:
            if from_state not in self._allowed_transitions:
                self._allowed_transitions[from_state] = set()
            self._allowed_transitions[from_state].add(to_state)

    def on_transition(self, callback: TransitionCallback):
        """
        Register callback for any transition.

        Args:
            callback: Function(old_state, new_state, trigger)
        """
        with self._lock:
            self._transition_callbacks.append(callback)

    def on_enter(self, state: StateType, callback: StateEnterCallback):
        """
        Register callback for entering a specific state.

        Args:
            state: State to monitor
            callback: Function(old_state, new_state)
        """
        with self._lock:
            if state not in self._enter_callbacks:
                self._enter_callbacks[state] = []
            self._enter_callbacks[state].append(callback)

    def on_exit(self, state: StateType, callback: StateExitCallback):
        """
        Register callback for exiting a specific state.

        Args:
            state: State to monitor
            callback: Function(old_state, new_state)
        """
        with self._lock:
            if state not in self._exit_callbacks:
                self._exit_callbacks[state] = []
            self._exit_callbacks[state].append(callback)

    def add_guard(self, from_state: StateType, to_state: StateType, guard: GuardCallback):
        """
        Add guard condition for a transition.

        Guard is called before transition and can block it.

        Args:
            from_state: Source state
            to_state: Target state
            guard: Function(from_state, to_state) -> bool
        """
        with self._lock:
            key = (from_state, to_state)
            if key not in self._guards:
                self._guards[key] = []
            self._guards[key].append(guard)

    def get_history(
        self, 
        max_items: Optional[int] = None,
        from_state: Optional[StateType] = None,
        to_state: Optional[StateType] = None
    ) -> List[Transition[StateType]]:
        """
        Get transition history.

        Args:
            max_items: Maximum items to return
            from_state: Filter by source state
            to_state: Filter by target state

        Returns:
            List of transitions
        """
        with self._lock:
            history = list(self._history)

            # Apply filters.
            if from_state is not None:
                history = [t for t in history if t.from_state == from_state]

            if to_state is not None:
                history = [t for t in history if t.to_state == to_state]

            # Limit results.
            if max_items is not None:
                history = history[-max_items:]

            return history
        
    def get_state_statistics(self, state: StateType) -> Dict[str, Any]:
        """
        Get statistics for a specific state.
        
        Args:
            state: State to analyze
            
        Returns:
            Dictionary with statistics
        """
        with self._lock:
            durations = self._state_durations.get(state, [])
            
            if not durations:
                return {
                    "state": state.value if isinstance(state, Enum) else str(state),
                    "visit_count": 0,
                    "total_time_sec": 0.0,
                    "avg_duration_sec": 0.0,
                    "min_duration_sec": 0.0,
                    "max_duration_sec": 0.0
                }
            
            return {
                "state": state.value if isinstance(state, Enum) else str(state),
                "visit_count": len(durations),
                "total_time_sec": sum(durations),
                "avg_duration_sec": sum(durations) / len(durations),
                "min_duration_sec": min(durations),
                "max_duration_sec": max(durations)
            }
    
    def get_transition_statistics(self, from_state: Optional[StateType] = None, to_state: Optional[StateType] = None) -> Dict[str, int]:
        """
        Get transition counts.
        
        Args:
            from_state: Filter by source state
            to_state: Filter by target state
            
        Returns:
            Dictionary of transition -> count
        """
        with self._lock:
            stats = {}
            
            for (f_state, t_state), count in self._transition_counts.items():
                if from_state is not None and f_state != from_state:
                    continue
                if to_state is not None and t_state != to_state:
                    continue
                
                key = f"{f_state.value if isinstance(f_state, Enum) else str(f_state)} -> {t_state.value if isinstance(t_state, Enum) else str(t_state)}"
                stats[key] = count
            
            return stats
    
    def get_all_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive statistics.
        
        Returns:
            Dictionary with all statistics
        """
        with self._lock:
            # Get all unique states from history.
            all_states = set()
            all_states.add(self._current_state)
            for transition in self._history:
                all_states.add(transition.from_state)
                all_states.add(transition.to_state)
            
            return {
                "current_state": self._state_info.to_dict(),
                "total_transitions": len(self._history),
                "state_statistics": {
                    state.value if isinstance(state, Enum) else str(state): self.get_state_statistics(state)
                    for state in all_states
                },
                "transition_counts": self.get_transition_statistics()
            }
        
    def reset(self, initial_state: Optional[StateType] = None):
        """
        Reset state machine.
        
        Args:
            initial_state: State to reset to (uses original if None)
        """
        with self._lock:
            if initial_state is None:
                # Try to get first state from history, or use current.
                if self._history:
                    initial_state = self._history[0].from_state
                else:
                    initial_state = self._current_state
            
            self._current_state = initial_state
            self._state_info = StateInfo(
                state=initial_state,
                entered_at=datetime.now(),
                previous_state=None,
                transition_count=0
            )
            self._history.clear()
            self._state_durations.clear()
            self._transition_counts.clear()
    
    def _is_transition_allowed(self, from_state: StateType, to_state: StateType) -> bool:
        """
        Check if transition is in allowed_transitions.
        """
        # If no allowed_transitions defined, all transitions allowed.
        if not self._allowed_transitions:
            return True
        
        # Check if from_state has allowed transitions.
        if from_state not in self._allowed_transitions:
            return False
        
        # Check if to_state is in allowed set.
        return to_state in self._allowed_transitions[from_state]
    
    def _check_guards(self, from_state: StateType, to_state: StateType) -> bool:
        """
        Check all guards for transition.
        """
        key = (from_state, to_state)
        guards = self._guards.get(key, [])
        
        for guard in guards:
            try:
                if not guard(from_state, to_state):
                    return False
            except Exception as e:
                print(f"[STATE MACHINE] Guard exception: {e}")
                return False
        
        return True
    
    def _call_transition_callbacks(self, old_state: StateType, new_state: StateType, trigger: str):
        """
        Call all transition callbacks.
        """
        for callback in self._transition_callbacks:
            try:
                callback(old_state, new_state, trigger)
            except Exception as e:
                print(f"[STATE MACHINE] Transition callback exception: {e}")
    
    def _call_enter_callbacks(self, old_state: StateType, new_state: StateType):
        """
        Call enter callbacks for new state.
        """
        callbacks = self._enter_callbacks.get(new_state, [])
        for callback in callbacks:
            try:
                callback(old_state, new_state)
            except Exception as e:
                print(f"[STATE MACHINE] Enter callback exception: {e}")
    
    def _call_exit_callbacks(self, old_state: StateType, new_state: StateType):
        """
        Call exit callbacks for old state.
        """
        callbacks = self._exit_callbacks.get(old_state, [])
        for callback in callbacks:
            try:
                callback(old_state, new_state)
            except Exception as e:
                print(f"[STATE MACHINE] Exit callback exception: {e}")

class TimedStateMachine(StateMachine[StateType]):
    """
    State machine with automatic timeout transitions.

    Extends base state machine to support automatic transitions after spending too long in a state.
    """
    def __init__(
        self,
        initial_state: StateType,
        allowed_transitions: Optional[Dict[StateType, Set[StateType]]] = None,
        name: str = "",
        max_history: int = 100,
        metrics_collector = None,
        event_bus = None
    ):
        """
        Initialize timed state machine.
        """
        super().__init__(initial_state=initial_state, allowed_transitions=allowed_transitions, name=name, max_history=max_history, metrics_collector=metrics_collector, event_bus=event_bus)

        # Timeout configuration.
        self._timeouts: Dict[StateType, tuple[float, StateType]] = {}

    def add_timeout(self, state: StateType, timeout_sec: float, timeout_target: StateType):
        """
        Add timeout for a state.

        If state machine stays in 'state' for longer than timeout_sec, automatically transition to timeout_target.

        Args:
            state: State to monitor
            timeout_sec: Timeout in seconds
            timeout_target: State to transition to on timeout
        """
        with self._lock:
            self._timeouts[state] = (timeout_sec, timeout_target)

    def check_timeouts(self):
        """
        Check for timeout conditions.

        Call this periodically to enforce timeouts.
        Should be called from a timer or update loop.
        """
        with self._lock:
            current = self._current_state

            if current in self._timeouts:
                timeout_sec, timeout_target = self._timeouts[current]
                time_in_state = self._state_info.time_in_state()

            if time_in_state >= timeout_sec:
                self.transition_to(timeout_target, trigger="timeout", metadata={"timeout_sec": timeout_sec})