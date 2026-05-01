"""
SnoBot Common Event System
Event-driven architecture for inter-component communication.

Provieds a publish-subscribe pattern with priority handling, filtering,
and weak references to prevent memory leaks.
"""

from typing import Callable, Any, Dict, List, Optional, Set, TypeVar
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from threading import Lock, RLock
from collections import defaultdict, deque
import weakref
import uuid

from common.common_types import ErrorSeverity

# Type definitions.
EventHandler = Callable[['Event'], None]
EventFilter = Callable[['Event'], bool]
T = TypeVar('T')

class EventPriority(Enum):
    """
    Event priority levels for ordering.
    """
    LOW = 0
    NORMAL = 1
    HIGH = 2
    CRITICAL = 3

    def __lt__(self, other):
        if isinstance(other, EventPriority):
            return self.value < other.value
        return NotImplemented

class EventType(Enum):
    """
    Standard event types across the system.
    """
    # SBCP events.
    COMMAND_SENT = "command.sent"
    COMMAND_RECEIVED = "command.recieved"
    COMMAND_FAILED = "command.failed"
    TELEMETRY_RECEIVED = "telemetry.received"
    STATE_CHANGED = "state.changed"
    FAULT_DETECTED = "fault.detected"
    FAULT_CLEARED = "fault.cleared"
    CONNECTION_OPENED = "connection.opened"
    CONNECTION_CLOSED = "connection.closed"
    ESTOP_ACTIVATED = "estop.activated"
    ESTOP_RELEASED = "estop.released"

    # SBVS events.
    FRAME_CAPTURED = "frame.captured"
    FRAME_PROCESSED = "frame.processed"
    OBJECT_DETECTED = "object.detected"
    OBJECT_TRACKED = "object.tracked"
    OBJECT_LOST = "object.lost"
    STEREO_SYNCED = "stereo.synced"
    CALIBRATION_COMPLETE = "calibration.complete"

    # System events.
    SYSTEM_STARTED = "system.started"
    SYSTEM_STOPPED = "system.stopped"
    ERROR_OCCURRED = "error.occurred"
    WARNING_ISSUED = "warning.issued"
    METRIC_RECORDED = "metric.recorded"

@dataclass
class Event:
    """
    Base event class.

    All events in the system inherit from this base class.
    Events are immutable after creation (frozen dataclass).
    """
    type: str
    timestamp: datetime = field(default_factory=datetime.now)
    source: Optional[str] = None
    priority: EventPriority = EventPriority.NORMAL
    data: Dict[str, Any] = field(default_factory=dict)
    event_id: str = field(default_factory=lambda: str(uuid.uuid4()))

    def __post_init__(self):
        """
        Ensure type is string.
        """
        # Convert EventType enum to string value
        if isinstance(self.type, EventType):
            object.__setattr__(self, 'type', self.type.value)
        # Ensure type is always a string
        elif not isinstance(self.type, str):
            object.__setattr__(self, 'type', str(self.type))

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get data field with default.
        """
        return self.data.get(key, default)
    
    def has(self, key: str) -> bool:
        """
        Check if data field exists.
        """
        return key in self.data
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert event to dictionary.
        """
        return {
            "type": self.type,
            "timestamp": self.timestamp.isoformat(),
            "source": self.source,
            "priority": self.priority.value,
            "data": self.data,
            "event_id": self.event_id
        }
    
class EventSubscription:
    """
    Represents a subscription to events.

    Tracks handler, filters, and statistics.
    """
    def __init__(self, 
        handler: EventHandler, 
        event_types: Optional[Set[str]] = None, 
        filters: Optional[List[EventFilter]] = None,
        priority_filter: Optional[EventPriority] = None
    ):
        """
        Initialize subscription.

        Args:
            handler: Callback function for events
            event_types: Set of event types to receive (None  = all)
            filters: Additional filter functions
            priority_filter: Minimum priority level
        """
        self.handler = handler
        self.filters = filters or []
        self.priority_filter = priority_filter
        self.subscription_id = str(uuid.uuid4())

        # Normalize event types.
        if isinstance(event_types, EventType):
            self.event_types = {event_types.value}
        elif isinstance(event_types, str):
            self.event_types = {event_types}
        elif isinstance(event_types, (list, set, tuple)):
            self.event_types = set(
                et.value if isinstance(et, EventType) else et
                for et in event_types
            )
        else:
            self.event_types = None  # None = subscribe to all.

        # Statistics.
        self.events_received = 0
        self.events_filtered = 0
        self.events_processed = 0
        self.last_event_time: Optional[datetime] = None
    
    def matches(self, event: Event) -> bool:
        """
        Check if event matched subscription criteria.

        Args:
            event: Event to check

        Returns:
            True if event should be delivered to this subscription
        """
        # Check event types.
        if self.event_types and event.type not in self.event_types:
            self.events_filtered += 1
            return False
        
        # Check priority filter.
        if self.priority_filter and event.priority < self.priority_filter:
            self.events_filtered += 1
            return False
        
        # Check custom filters.
        for filter_func in self.filters:
            try:
                if not filter_func(event):
                    self.events_filtered += 1
                    return False
            except Exception:
                # Filter raised exception, treat as no match.
                self.events_filtered += 1
                return False
            
        return True
    
    def deliver(self, event: Event):
        """
        Deliver event to handler.

        Args:
            event: Event to deliver
        """
        self.events_received += 1

        try:
            self.handler(event)
            self.events_processed += 1
            self.last_event_time = datetime.now()
        except Exception as e:
            # Handler raised exception. Log but do not propagate.
            print(f"[EVENT] Handler exception: {e}")

    def get_stats(self) -> Dict[str, Any]:
        """
        Get subscription statistics.
        """
        return {
            "subscription_id": self.subscription_id,
            "events_received": self.events_received,
            "events_filtered": self.events_filtered,
            "events_processed": self.events_processed,
            "last_event_time": self.last_event_time.isoformat() if self.last_event_time else None
        }
    
class EventBus:
    """
    Central event bus for publish-subscribe messaging.

    Thread-safe event distribution with filtering and priority handling.
    Uses weak references for handlers to prevent memory leaks.
    """
    def __init__(self, max_history: int = 100, enable_history: bool = True):
        """
        Initialize event bus.

        Args:
            max_history: Maximum events to keep in history
            senable_history: Enable event history tracking
        """
        self.subscriptions: List[EventSubscription] = []
        self._lock = RLock()

        # Event history.
        self.enable_history = enable_history
        self.max_history = max_history
        self.history: deque = deque(maxlen=max_history)

        # Statistics.
        self.events_published = 0
        self.events_delivered = 0
        self.events_dropped = 0

        # Event type tracking.
        self.event_counts: Dict[str, int] = defaultdict(int)

    def subscribe(
        self, 
        handler: EventHandler,
        event_types: Optional[Set[str]] = None,
        filters: Optional[List[EventFilter]] = None, 
        priority_filter: Optional[EventPriority] = None
    ) -> str:
        """
        Subscribe to events.

        Args:
            handler: Callback function for events
            event_types: Set of event types to receive (None  = all)
            filters: Additional filter functions
            priority_filter: Minimum priority level

        Returns:
            Subscription ID for later unsubscribe
        """
        subscription = EventSubscription(handler=handler, event_types=event_types, filters=filters, priority_filter=priority_filter)

        with self._lock:
            self.subscriptions.append(subscription)

        return subscription.subscription_id
    
    def unsubscribe(self, subscription_id: str) -> bool:
        """
        Unsubscribe from events.

        Args:
            subscription_id: ID returned from subscribe()

        Returns:
            True if subscription was found and removed
        """
        with self._lock:
            for i, sub in enumerate(self.subscriptions):
                if sub.subscription_id == subscription_id:
                    del self.subscriptions[i]
                    return True
                
        return False
    
    def publish(self, event: Event):
        """
        Publish event to all matching subscribers.

        Args:
            event: Event to publish
        """
        with self._lock:
            self.events_published += 1
            self.event_counts[event.type] += 1

            # Add to history.
            if self.enable_history:
                self.history.append(event)

            # Deliver to matching subscriptions.
            delivered_count = 0
            for sub in self.subscriptions:
                if sub.matches(event):
                    sub.deliver(event)
                    delivered_count += 1

            self.events_delivered += delivered_count
            if delivered_count == 0:
                self.events_dropped += 1

    def emit(
        self,
        event_type: str | EventType,
        source: Optional[str] = None,
        priority: EventPriority = EventPriority.NORMAL,
        **data
    ):
        """
        Convenience method to create and publish event.

        Args:
            event_type: Type of event
            source: Source component
            priority: Event priority
            **data: Event data as keyword arguments
        """
        if isinstance(event_type, EventType):
            event_type = event_type.value
        
        event = Event(type=event_type, source=source, priority=priority, data=data)
        self.publish(event)

    def get_history(self, event_type: Optional[str] = None, limit: Optional[int] = None) -> List[Event]:
        """
        Get event history.

        Args:
            event_type: Filter by event type (None = all)
            limit: Maximum events to return

        Returns:
            List of events
        """
        with self._lock:
            events = list(self.history)
        
        if event_type:
            events = [e for e in events if e.type == event_type]

        if limit:
            events = events[-limit:]

        return events
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get event bus statistics.
        """
        with self._lock:
            return {
                "events_published": self.events_published,
                "events_delivered": self.events_delivered,
                "events_dropped": self.events_dropped,
                "active_subscriptions": len(self.subscriptions),
                "event_counts": dict(self.event_counts),
                "history_size": len(self.history)
            }
        
    def clear_history(self):
        """
        Clear event history.
        """
        with self._lock:
            self.history.clear()

    def reset_stats(self):
        """
        Reset statistics counters.
        """
        with self._lock:
            self.events_published = 0
            self.events_delivered = 0
            self.events_dropped = 0
            self.event_counts.clear()

# Global event bus instance
_global_event_bus: Optional[EventBus] = None
_bus_lock = Lock()

def get_event_bus() -> EventBus:
    """
    Get or create global event bus instance.

    Returns:
        Global EventBus instance
    """
    global _global_event_bus
    with _bus_lock:
        if _global_event_bus is None:
            _global_event_bus = EventBus()

    return _global_event_bus

def reset_event_bus():
    """
    Reset global event bus (mainly for testing).
    """
    global _global_event_bus
    with _bus_lock:
        _global_event_bus = None

# Convenience function using global bus.
def subscribe(
    handler: EventHandler,
    event_types: Optional[Set[str]] = None,
    filters: Optional[List[EventFilter]] = None,
    priority_filter: Optional[EventPriority] = None
) -> str:
    """
    Subscribe to global event bus.
    """
    return get_event_bus().subscribe(handler, event_types, filters, priority_filter)

def unsubscribe(subscription_id: str) -> bool:
    """
    Unsubscribe from global event bus.
    """
    return get_event_bus().unsubscribe(subscription_id)

def publish(event: Event):
    """
    Publish to global event bus.
    """
    get_event_bus().publish(event)

def emit(
    event_type: str | EventType,
    source: Optional[str] = None,
    priority: EventPriority = EventPriority.NORMAL,
    **data
):
    """
    Emit event to global event bus.
    """
    get_event_bus().emit(event_type, source, priority, **data)

# Decorator for event handlers.
def event_handler(*event_types: str | EventType):
    """
    Decorator to mark a method as an event handler.

    Usage:
        @eventhandler(EventType.COMMAND_SENT, EventType.COMMAND_RECEIVED)
        def on_command(self, event: Event):
            print(f"Command: {event.data})
    """
    def decorator(func: Callable) -> Callable:
        # Store event types on function for later registration.
        type_set = set()
        for et in event_types:
            if isinstance(et, EventType):
                type_set.add(et.value)
            else:
                type_set.add(et)
        func._event_types = type_set
        return func
    return decorator