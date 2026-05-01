"""
SnoBot Common Watchdog System
Timeout monitoring with configurable actions and callbacks.

Provides generic watchdog timers for detecting:
- Communication timeouts
- Motion command timeouts
- Processing hangs
- Health check features
"""

from typing import Callable, Optional, Dict, Any, List
from dataclasses import dataclass, field
from threading import Thread, Event, Lock, RLock
from enum import Enum
import time

from common.common_types import ErrorSeverity

class WatchdogState(Enum):
    """
    Watchdog states.
    """
    STOPPED = "stopped"
    RUNNING = "running"
    TRIGGERED = "triggered"
    DISABLED = "disabled"

@dataclass
class WatchdogConfig:
    """
    Watchdog configuration.
    """
    timeout_sec: float
    callback: Callable[[], None]
    auto_reset: bool = False
    grace_period_sec: float = 0.0
    max_triggers: int = 0   # Unlimited.
    name: str = ""

    def validate(self):
        """
        Validate configuration.
        """
        if self.timeout_sec <= 0:
            raise ValueError("timeout_sec must be positive")
        
        if self.grace_period_sec < 0:
            raise ValueError("grace_period_sec cannot be negative")
        
        if self.max_triggers < 0:
            raise ValueError("max_triggers cannot be negative")
        
@dataclass
class WatchdogStats:
    """
    Watchdog statistics.
    """
    name: str
    state: WatchdogState
    timeout_sec: float
    time_remaining_sec: float
    trigger_count: int
    last_feed_time: float
    last_trigger_time: Optional[float]
    total_runtime_sec: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "name": self.name,
            "state": self.state.value,
            "timeout_sec": self.timeout_sec,
            "time_remaining_sec": self.time_remaining_sec,
            "trigger_count": self.trigger_count,
            "last_feed_time": self.last_feed_time,
            "last_trigger_time": self.last_trigger_time,
            "total_runtime_sec": self.total_runtime_sec,
            "is_expired": self.time_remaining_sec <= 0
        }
    
class Watchdog:
    """
    Generic watchdog timer.

    Monitors for timeout and calls callback if not fed within timeout period.
    Thread-safe and suitable for detecting various timeout conditions.
    """
    def __init__(
        self, 
        timeout_sec: float,
        callback: Callable[[], None],
        auto_reset: bool = False,
        grace_period_sec: float = 0.0,
        max_triggers: int = 0,
        name: str = "",
        metrics_collector = None,
        event_bus = None
    ):
        """
        Initialize watchdog.

        Args:
            timeout_sec: Timeout period in seconds
            callback: Function to call on timeout
            auto_reset: Automatically reset after triggering
            grace_period_sec: Grace period before first trigger
            max_triggers: Maximum times to trigger (0 = unlimited)
            name: Watchdog name for identification
            metrics_collector: Optional metrics integration
            event_bus: Optional event bus integration
        """
        self.config = WatchdogConfig(
            timeout_sec=timeout_sec,
            callback=callback,
            auto_reset=auto_reset,
            grace_period_sec=grace_period_sec,
            max_triggers=max_triggers,
            name=name or f"watchdog_{id(self)}"
        )
        self.config.validate()

        self.metrics = metrics_collector
        self.events = event_bus

        # State.
        self._state = WatchdogState.STOPPED
        self._state_lock = RLock()

        # Timing.
        self._last_feed_time = 0.0
        self._start_time = 0.0
        self._trigger_count = 0
        self._last_trigger_time: Optional[float] = None

        # Thread control.
        self._thread: Optional[Thread] = None
        self._stop_event = Event()
        self._feed_event = Event()

    def start(self):
        """
        Start the watchdog.
        """
        with self._state_lock:
            if self._state == WatchdogState.RUNNING:
                return
            
            if self._state == WatchdogState.DISABLED:
                raise RuntimeError("Cannot start disabled watchdog")
            
            self._state = WatchdogState.RUNNING
            self._stop_event.clear()
            self._feed_event.clear()
            self._last_feed_time = time.time()
            self._start_time = time.time()

            # Start monitoring thread.
            self._thread = Thread(target=self._monitor_loop, daemon=True)
            self._thread.start()

            # Record metric.
            if self.metrics:
                self.metrics.increment(f"watchdog.{self.config.name}.started")

            # Emit event.
            if self.events:
                from common.common_events import EventType
                self.events.emit(
                    EventType.SYSTEM_STARTED,
                    source=f"watchdog.{self.config.name}",
                    watchdog_name=self.config.name,
                    timeout_sec=self.config.timeout_sec
                )

    def stop(self):
        """
        Stop the watchdog.
        """
        with self._state_lock:
            if self._state == WatchdogState.STOPPED:
                return
            
            old_state = self._state
            self._state = WatchdogState.STOPPED
            self._stop_event.set()

            # Wait for thread to finish.
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)

            # Record metric.
            if self.metrics:
                self.metrics.increment(f"watchdog.{self.config.name}.stopped")

            # Emit event.
            if self.events:
                from common.common_events import EventType
                self.events.emit(
                    EventType.SYSTEM_STOPPED,
                    source=f"watchdog,{self.config.name}",
                    watchdog_name=self.config.name,
                    was_triggered=old_state == WatchdogState.TRIGGERED
                )

    def feed(self):
        """
        Feed the watchdog (reset timer).

        Call this periodically to prevent timeout.
        """
        with self._state_lock:
            if self._state != WatchdogState.RUNNING:
                return
            
            self._last_feed_time = time.time()
            self._feed_event.set()

            # If was triggered and auto_reset enabled, reset state.
            if self._state == WatchdogState.TRIGGERED and self.config.auto_reset:
                self._state = WatchdogState.RUNNING

            # Record metric.
            if self.metrics:
                self.metrics.increment(f"watchdog.{self.config.name}.fed")

    def reset(self):
        """
        Reset watchdog (clear triggered state).

        Useful after handling a timeout condition.
        """
        with self._state_lock:
            if self._state == WatchdogState.TRIGGERED:
                self._state = WatchdogState.RUNNING
                self._last_feed_time = time.time()
                self._feed_event.set()

    def disable(self):
        """
        Disable watchdog (stop and prevent restart).
        """
        self.stop()
        with self._state_lock:
            self._state = WatchdogState.DISABLED

    def enable(self):
        """
        Enable watchdog (allow restart).
        """
        with self._state_lock:
            if self._state == WatchdogState.DISABLED:
                self._state = WatchdogState.STOPPED

    def is_running(self) -> bool:
        """
        Check if watchdog is running.
        """
        return self._state == WatchdogState.RUNNING
    
    def is_triggered(self) -> bool:
        """
        Check if wathdog has triggered.
        """
        return self._state == WatchdogState.TRIGGERED
    
    def time_remaining(self) -> float:
        """
        Get time remaining until timeout.

        Returns:
            Seconds remaining (negative if expired)
        """
        with self._state_lock:
            if self._state != WatchdogState.RUNNING:
                return 0.0
            
            elapsed = time.time() - self._last_feed_time
            return self.config.timeout_sec - elapsed
        
    def get_stats(self) -> WatchdogStats:
        """
        Get watchdog statistics.

        Returns:
            WatchdogStats object
        """
        with self._state_lock:
            total_runtime = time.time() - self._start_time if self._start_time > 0 else 0.0
            
            return WatchdogStats(
                name=self.config.name,
                state=self._state,
                timeout_sec=self.config.timeout_sec,
                time_remaining_sec=self.time_remaining(),
                trigger_count=self._trigger_count,
                last_feed_time=self._last_feed_time,
                last_trigger_time=self._last_trigger_time,
                total_runtime_sec=total_runtime
            )
        
    def _monitor_loop(self):
        """
        Main monitoring loop (runs in separate thread).
        """
        # Apply grace period if configured.
        if self.config.grace_period_sec > 0:
            if self._stop_event.wait(self.config.grace_period_sec):
                return  # Stopped during grace period.
        
        while not self._stop_event.is_set():
            # Calculate time until timeout.
            with self._state_lock:
                time_remaining = self.time_remaining()
            
            if time_remaining <= 0:
                # Timeout occurred.
                self._handle_timeout()
                
                # Check if should continue monitoring
                with self._state_lock:
                    if not self.config.auto_reset:
                        break  # Stop monitoring after trigger
                    
                    # Check max triggers
                    if self.config.max_triggers > 0 and self._trigger_count >= self.config.max_triggers:
                        self._state = WatchdogState.STOPPED
                        break
            else:
                # Wait for timeout or feed event.
                # Use small timeout to periodically check.
                wait_time = min(time_remaining, 0.1)
                if self._feed_event.wait(wait_time):
                    self._feed_event.clear()

    def _handle_timeout(self):
        """
        Handle watchdog timeout.
        """
        with self._state_lock:
            self._state = WatchdogState.TRIGGERED
            self._trigger_count += 1
            self._last_trigger_time = time.time()
            
            # Record metric
            if self.metrics:
                self.metrics.increment(f"watchdog.{self.config.name}.triggered")
                self.metrics.record(
                    f"watchdog.{self.config.name}.timeout_sec",
                    time.time() - self._last_feed_time
                )
            
            # Emit event
            if self.events:
                from common.common_events import EventType, EventPriority
                self.events.emit(
                    EventType.WARNING_ISSUED,
                    source=f"watchdog.{self.config.name}",
                    priority=EventPriority.HIGH,
                    watchdog_name=self.config.name,
                    timeout_sec=self.config.timeout_sec,
                    trigger_count=self._trigger_count,
                    time_since_feed=time.time() - self._last_feed_time
                )
        
        # Call callback outside of lock to prevent deadlock.
        try:
            self.config.callback()
        except Exception as e:
            # Don't let callback exceptions kill the watchdog.
            print(f"[WATCHDOG] Callback exception in {self.config.name}: {e}")
            
            if self.metrics:
                self.metrics.increment(f"watchdog.{self.config.name}.callback_errors")

class WatchdogManager:
    """
    Manages multiple watchdogs.

    Provides centralized control and monitoring of all watchdogs in the system.
    """
    def __init__(self, metrics_collector=None, event_bus=None):
        """
        Initialize watchdog manager.
        
        Args:
            metrics_collector: Optional metrics integration
            event_bus: Optional event bus integration
        """
        self.metrics = metrics_collector
        self.events = event_bus
        
        self._watchdogs: Dict[str, Watchdog] = {}
        self._lock = Lock()
    
    def create_watchdog(
        self,
        name: str,
        timeout_sec: float,
        callback: Callable[[], None],
        auto_reset: bool = False,
        grace_period_sec: float = 0.0,
        max_triggers: int = 0,
        auto_start: bool = True
    ) -> Watchdog:
        """
        Create and register a new watchdog.
        
        Args:
            name: Watchdog name (must be unique)
            timeout_sec: Timeout period
            callback: Timeout callback
            auto_reset: Auto-reset after trigger
            grace_period_sec: Grace period
            max_triggers: Max triggers
            auto_start: Start immediately
            
        Returns:
            Created Watchdog instance
        """
        with self._lock:
            if name in self._watchdogs:
                raise ValueError(f"Watchdog '{name}' already exists")
            
            watchdog = Watchdog(
                timeout_sec=timeout_sec,
                callback=callback,
                auto_reset=auto_reset,
                grace_period_sec=grace_period_sec,
                max_triggers=max_triggers,
                name=name,
                metrics_collector=self.metrics,
                event_bus=self.events
            )
            
            self._watchdogs[name] = watchdog
            
            if auto_start:
                watchdog.start()
            
            return watchdog
    
    def get_watchdog(self, name: str) -> Optional[Watchdog]:
        """
        Get watchdog by name.
        
        Args:
            name: Watchdog name
            
        Returns:
            Watchdog instance or None
        """
        with self._lock:
            return self._watchdogs.get(name)
    
    def remove_watchdog(self, name: str) -> bool:
        """
        Remove watchdog.
        
        Args:
            name: Watchdog name
            
        Returns:
            True if removed, False if not found
        """
        with self._lock:
            watchdog = self._watchdogs.get(name)
            if watchdog:
                watchdog.stop()
                del self._watchdogs[name]
                return True
            return False
    
    def feed_watchdog(self, name: str) -> bool:
        """
        Feed a specific watchdog.
        
        Args:
            name: Watchdog name
            
        Returns:
            True if fed, False if not found
        """
        watchdog = self.get_watchdog(name)
        if watchdog:
            watchdog.feed()
            return True
        return False
    
    def start_all(self):
        """
        Start all watchdogs.
        """
        with self._lock:
            for watchdog in self._watchdogs.values():
                if not watchdog.is_running():
                    watchdog.start()
    
    def stop_all(self):
        """
        Stop all watchdogs.
        """
        with self._lock:
            for watchdog in self._watchdogs.values():
                watchdog.stop()
    
    def feed_all(self):
        """
        Feed all watchdogs.
        """
        with self._lock:
            for watchdog in self._watchdogs.values():
                watchdog.feed()
    
    def get_all_stats(self) -> Dict[str, WatchdogStats]:
        """
        Get statistics for all watchdogs.
        
        Returns:
            Dictionary of name -> stats
        """
        with self._lock:
            return {
                name: watchdog.get_stats()
                for name, watchdog in self._watchdogs.items()
            }
    
    def get_triggered_watchdogs(self) -> List[str]:
        """
        Get list of triggered watchdog names.
        
        Returns:
            List of watchdog names that have triggered
        """
        with self._lock:
            return [
                name for name, watchdog in self._watchdogs.items()
                if watchdog.is_triggered()
            ]
    
    def health_check(self) -> Dict[str, Any]:
        """
        Perform health check on all watchdogs.
        
        Returns:
            Health status dictionary
        """
        stats = self.get_all_stats()
        triggered = self.get_triggered_watchdogs()
        
        return {
            "total_watchdogs": len(stats),
            "running": sum(1 for s in stats.values() if s.state == WatchdogState.RUNNING),
            "triggered": len(triggered),
            "triggered_names": triggered,
            "healthy": len(triggered) == 0,
            "watchdogs": {name: stat.to_dict() for name, stat in stats.items()}
        }
    
# Global watchdog manager.
_global_watchdog_manager: Optional[WatchdogManager] = None
_manager_lock = Lock()

def get_watchdog_manager() -> WatchdogManager:
    """
    Get or create global watchdog manager.

    Returns;
        Global WathdogManager instance
    """
    global _global_watchdog_manager

    if _global_watchdog_manager is None:
        with _manager_lock:
            if _global_watchdog_manager is None:
                _global_watchdog_manager = WatchdogManager()

    return _global_watchdog_manager

def reset_watchdog_manager():
    """
    Reset global watchdog manager (mainly for testing).
    """
    global _global_watchdog_manager
    with _manager_lock:
        if _global_watchdog_manager:
            _global_watchdog_manager.stop_all()
        _global_watchdog_manager = None

# Convenience functions.
def create_watchdog(name: str, timeout_sec: float, callback: Callable[[], None], **kwargs) -> Watchdog:
    """
    Create a watchdog using the global manager.
    """
    return get_watchdog_manager().create_watchdog(name=name, timeout_sec=timeout_sec, callback=callback, **kwargs)

def feed_watchdog(name: str) -> bool:
    """
    Feed watchdog using global manager.
    """
    return get_watchdog_manager().feed_watchdog(name)

def get_watchdog(name: str) -> Optional[Watchdog]:
    """
    Get watchdog from global manager.
    """
    return get_watchdog_manager().get_watchdog(name)