"""
SnoBot Common Queue Utilities
Thread-safe queues with monitoring, prioritization, and statistics.

Provides enhances queue implementations with:
- Overflow detection and tracking
- Statistical monitoring
- Priority-based ordering
- Timeout handling
- Integration with metrics and events
"""

from typing import TypeVar, Generic, Optional, Any, Callable, List, Dict
from dataclasses import dataclass, field
from queue import Queue, PriorityQueue, Empty, Full
from collections import deque
from threading import Lock, Condition, Event
from enum import IntEnum
import time
import heapq
from common.common_metrics import MetricCollector
from itertools import count, islice

T = TypeVar('T')

class QueueOverflowPolicy(IntEnum):
    """
    Policies for handling queue overflow.
    """
    BLOCK = 0       # Block until space available.
    DROP_NEWEST = 1 # Drop the new item being added.
    DROP_OLDEST = 2 # Remove oldest item to make space.
    RAISE = 3       # Raise Full exception.

@dataclass
class QueueStats:
    """
    Queue statistics.
    """
    name: str
    current_size: int
    max_size: int
    total_items_added: int
    total_items_removed: int
    overflow_count: int
    drop_count: int
    max_size_reached: int
    average_wait_time_ms: float
    current_wait_time_ms: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "name": self.name,
            "current_size": self.current_size,
            "max_size": self.max_size,
            "total_items_added": self.total_items_added,
            "total_items_removed": self.total_items_removed,
            "overflow_count": self.overflow_count,
            "drop_count": self.drop_count,
            "max_size_reached": self.max_size_reached,
            "average_wait_time_ms": self.average_wait_time_ms,
            "current_wait_time_ms": self.current_wait_time_ms,
            "utilization": self.current_size / self.max_size if self.max_size > 0 else 0.0
        }
    
class MonitoredQueue(Generic[T]):
    """
    Thread-safe queue with overflow tracking and statistics.

    Extends standard Queue with monitoring capabilities, overflow policies,
    and integration with metrics system.
    """
    def __init__(
        self, 
        max_size: int = 0,
        name: str = "",
        overflow_policy: QueueOverflowPolicy = QueueOverflowPolicy.BLOCK,
        metrics_collector: Optional[MetricCollector] = None,
        event_bus = None
    ):
        """
        Initialize monitored queue.

        Args:
            max_size: Maximum queue size (0 = unlimited)
            name: Queue name for identification
            overflow_policy: How to handle overflow
            metrics_collector: Optional metrics collector for integration
            event_bus: Optional event bus for notifications
        """
        self.name = name or f"queue_{id(self)}"
        self.max_size = max_size
        self.overflow_policy = overflow_policy
        self.metrics = metrics_collector
        self.events = event_bus

        # Internal queue exclusively for data movement.
        self._queue: Queue[T] = Queue(maxsize=max_size if overflow_policy == QueueOverflowPolicy.BLOCK else 0)
        
        # Lock to protect stats.
        self._lock = Lock()

        # Statistics.
        self.items_added = 0
        self.items_removed = 0
        self.overflow_count = 0
        self.drop_count = 0
        self.max_size_reached = 0
        
        # Wait time tracking.
        self._wait_times: deque[float] = deque(maxlen=100)
        self._last_add_time = 0.0      

    def put(self, item: T, block: bool = True, timeout: Optional[float] = None):
        """
        Put item into queue.

        Args:
            item: Item to add
            block: Whether to block in queue if full
            timeout: Timeout in seconds (None = wait forever)

        Returns:
            True if item was added , False if dropped

        Raises:
            Full: If overflow_policy is RAISE and queue is full
        """
        start_time = time.monotonic()

        try:
            # BLOCK policy delegates fully to Queue.
            if self.overflow_policy == QueueOverflowPolicy.BLOCK:
                self._queue.put(item, block=block, timeout=timeout)
            else:
                # Non-blocking attempt.
                self._queue.put(item, block=False)
                
        except Full:
            # Overflow handling (no locks held).
            self._handle_overflow()
            return False
        
        wait_ms = (time.monotonic() - start_time) * 1000.0
        
        # Update stats quickly.
        with self._lock:
            self.items_added += 1
            size = self._queue.qsize()
            if size > self.max_size_reached:
                self.max_size_reached = size
            self._wait_times.append(wait_ms)
        
        if self.metrics:
            self.metrics.increment(f"queue.{self.name}.added")
            self.metrics.set_gauge(f"queue.{self.name}.size", size)
            self.metrics.record(f"queue.{self.name}.wait_time_ms", wait_ms)

        return True

    def get(self, block: bool = True, timeout: Optional[float] = None) -> Optional[T]:
        """
        Get item from queue.

        Args:
            block: Whether to block if queue is empty
            timeout: Timeout in seconds (None = wait forever)

        Returns:
            Item from queue, or None if timeout

        Raises:
            Empty: If non-blocking and queue is empty
        """
        try:
            item = self._queue.get(block=block, timeout=timeout)

        except Empty:
            if not block:
                raise
            return None
        
        with self._lock:
            self.items_removed += 1
            size = self._queue.qsize()
            
        if self.metrics:
            self.metrics.increment(f"queue.{self.name}.removed")
            self.metrics.set_gauge(f"queue.{self.name}.size", size)
            
        return item
        
    def get_nowait(self) -> Optional[T]:
        """
        Get item without blocking.

        Returns:
            Item or None if queue is empty
        """
        try:
            return self.get(block=False)
        except Empty:
            return None
        
    def put_nowait(self, item: T) -> bool:
        """
        Put item without blocking.

        Args:
            item: Item to add

        Returns:
            True if added, False if dropped/full
        """
        return self.put(item, block=False)
    
    def _handle_overflow(self):
        """
        Handle queue overflows depending on policy.
        """
        self.overflow_count += 1
        
        if self.overflow_policy == QueueOverflowPolicy.DROP_NEWEST:
            self.drop_count += 1
        elif self.overflow_policy == QueueOverflowPolicy.DROP_OLDEST:
            try:
                self._queue.get_nowait()
                self.drop_count += 1
            except Empty:
                pass
        
        elif self.overflow_policy == QueueOverflowPolicy.RAISE:
            raise Full(f"Queue {self.name} is full")
        
        # Metrics intentionally moved outside any locks.
        if self.metrics:
            self.metrics.increment(f"queue.{self.name}.overflow")
            if self.drop_count:
                self.metrics.increment(f"queue.{self.name}.dropped")

        if self.events:
            from common.common_events import EventType, EventPriority

            self.events.emit(
                EventType.WARNING_ISSUED,
                source=f"queue.{self.name}",
                priority=EventPriority.HIGH,
                message=f"Queue {self.name} overflow",
                queue_size=self._queue.qsize(),
                max_size=self.max_size,
            )
    
    def qsize(self) -> int:
        """
        Get approximate queue size.
        """
        return self._queue.qsize()
    
    def empty(self) -> bool:
        """
        Check if queue is empty.
        """
        return self._queue.empty()
    
    def full(self) -> bool:
        """
        Check if queue is full.
        """
        return self.maxsize > 0 and self._queue.qsize() >= self.maxsize
    
    def clear(self):
        """
        Clear all items from queue.
        """
        with self._lock:
            while not self._queue.empty():
                try:
                    self._queue.get_nowait()
                except Empty:
                    break
    
    def get_stats(self) -> QueueStats:
        """
        Get queue statistics.
        
        Returns:
            QueueStats object
        """
        with self._lock:
            avg_wait = sum(self._wait_times) / len(self._wait_times) if self._wait_times else 0.0
            current_wait = self._wait_times[-1] if self._wait_times else 0.0
            
            return QueueStats(
                name=self.name,
                current_size=self._queue.qsize(),
                max_size=self.maxsize,
                total_items_added=self.items_added,
                total_items_removed=self.items_removed,
                overflow_count=self.overflow_count,
                drop_count=self.drop_count,
                max_size_reached=self.max_size_reached,
                average_wait_time_ms=avg_wait,
                current_wait_time_ms=current_wait
            )
    
    def reset_stats(self):
        """
        Reset statistics counters.
        """
        with self._lock:
            self.items_added = 0
            self.items_removed = 0
            self.overflow_count = 0
            self.drop_count = 0
            self.max_size_reached = 0
            self._wait_times.clear()

@dataclass
class PriorityItem(Generic[T]):
    """
    Item with priority for priority queues.
    """
    priority: int
    sequence: int   # For stable sorting
    item: T

    def __lt__(self, other: 'PriorityItem') -> bool:
        """
        Compare by priority, then sequence.
        """
        if self.priority != other.priority:
            return self.priority < other.priority
        return self.sequence < other.sequence
    
class MonitoredPriorityQueue(Generic[T]):
    """
    Priority queue with monitoring.

    Items with lower priority values are retrieved first.
    Supports same monitoring features as MonitoredQueue.
    """
    def __init__(
        self,
        maxsize: int = 0,
        name: str = "",
        overflow_policy: QueueOverflowPolicy = QueueOverflowPolicy.BLOCK,
        metrics_collector: Optional[MetricCollector] = None,
        event_bus=None
    ):
        """
        Initialize monitored priority queue.
        
        Args:
            maxsize: Maximum queue size (0 = unlimited)
            name: Queue name for identification
            overflow_policy: How to handle overflow
            metrics_collector: Optional metrics collector
            event_bus: Optional event bus
        """
        self.name = name or f"pqueue_{id(self)}"
        self.maxsize = maxsize
        self.overflow_policy = overflow_policy
        self.metrics = metrics_collector
        self.events = event_bus
        
        # Internal priority queue.
        self._queue: PriorityQueue[PriorityItem[T]] = PriorityQueue(
            maxsize=maxsize if overflow_policy == QueueOverflowPolicy.BLOCK else 0
        )
        self._lock = Lock()
        
        self._sequence = count()
        
        # Statistics.
        self.items_added = 0
        self.items_removed = 0
        self.overflow_count = 0
        self.drop_count = 0
        self.max_size_reached = 0
        self._wait_times: deque = deque(maxlen=100)
    
    def put(
        self,
        item: T,
        priority: int = 0,
        block: bool = True,
        timeout: Optional[float] = None
    ) -> bool:
        """
        Put item into priority queue.
        
        Args:
            item: Item to add
            priority: Priority (lower = higher priority)
            block: Whether to block if queue is full
            timeout: Timeout in seconds
            
        Returns:
            True if item was added, False if dropped
        """
        start_time = time.monotonic()
        
        wrapped = PriorityItem(priority=priority, sequence=next(self._sequence), item=item)
        
        try:
            if self.overflow_policy == QueueOverflowPolicy.BLOCK:
                self._queue.put(wrapped, block=block, timeout=timeout)
            else:
                self._queue.put(wrapped, block=False)
        except Full:
            self._handle_overflow()
            return False
        
        wait_ms = (time.monotonic() - start_time) * 1000.0
        
        with self._lock:
            self.items_added += 1
            size = self._queue.qsize()
            if size > self.max_size_reached:
                self.max_size_reached = size
            self._wait_times.append(wait_ms)
            
        if self.metrics:
            self.metrics.increment(f"queue.{self.name}.added")
            self.metrics.set_gauge(f"queue.{self.name}.size", size)
            self.metrics.record(f"queue.{self.name}.wait_time_ms", wait_ms)

        return True
    
    def get(
        self,
        block: bool = True,
        timeout: Optional[float] = None
    ) -> Optional[T]:
        """
        Get highest priority item from queue.
        
        Args:
            block: Whether to block if queue is empty
            timeout: Timeout in seconds
            
        Returns:
            Item with highest priority (lowest priority value)
        """
        try:
            wrapped = self._queue.get(block=block, timeout=timeout)
        except Empty:
            if not block:
                raise
            return None

        with self._lock:
            self.items_removed += 1
            size = self._queue.qsize()

        if self.metrics:
            self.metrics.increment(f"queue.{self.name}.removed")
            self.metrics.set_gauge(f"queue.{self.name}.size", size)

        return wrapped.item
    
    def _handle_overflow(self):
        self.overflow_count += 1

        if self.overflow_policy == QueueOverflowPolicy.DROP_NEWEST:
            self.drop_count += 1

        elif self.overflow_policy == QueueOverflowPolicy.DROP_OLDEST:
            try:
                self._queue.get_nowait()
                self.drop_count += 1
            except Empty:
                pass

        elif self.overflow_policy == QueueOverflowPolicy.RAISE:
            raise Full(f"Queue {self.name} is full")

        if self.metrics:
            self.metrics.increment(f"queue.{self.name}.overflow")
            if self.drop_count:
                self.metrics.increment(f"queue.{self.name}.dropped")

        if self.events:
            from common.common_events import EventType, EventPriority

            self.events.emit(
                EventType.WARNING_ISSUED,
                source=f"queue.{self.name}",
                priority=EventPriority.HIGH,
                message=f"Priority queue {self.name} overflow",
                queue_size=self._queue.qsize(),
                max_size=self.maxsize,
            )
    
    def qsize(self) -> int:
        """
        Get approximate queue size.
        """
        return self._queue.qsize()
    
    def empty(self) -> bool:
        """
        Check if queue is empty.
        """
        return self._queue.empty()
    
    def full(self) -> bool:
        """
        Check if queue is full.
        """
        return self.maxsize > 0 and self._queue.qsize() >= self.maxsize
    
    def get_stats(self) -> QueueStats:
        """
        Get queue statistics.
        """
        with self._lock:
            avg_wait = sum(self._wait_times) / len(self._wait_times) if self._wait_times else 0.0
            current_wait = self._wait_times[-1] if self._wait_times else 0.0
            
            return QueueStats(
                name=self.name,
                current_size=self._queue.qsize(),
                max_size=self.maxsize,
                total_items_added=self.items_added,
                total_items_removed=self.items_removed,
                overflow_count=self.overflow_count,
                drop_count=self.drop_count,
                max_size_reached=self.max_size_reached,
                average_wait_time_ms=avg_wait,
                current_wait_time_ms=current_wait
            )
        
class CircularBuffer(Generic[T]):
    """
    Fixed-size circular buffer (ring buffer).

    Automatically overwrites data when full.
    Useful for fixed-size frame buffers or telemetry history.
    """
    __slots__ = (
        "name",
        "max_size",
        "metrics",
        "_buffer",
        "_index",
        "_count",
        "_lock",
        "items_added",
        "items_overwritten",
    )

    def __init__(self, max_size: int, name: str = "", metrics_collector: Optional[MetricCollector] = None):
        if max_size <= 0:
            raise ValueError("max_size must be positive")

        self.name = name or f"circular_{id(self)}"
        self.max_size = max_size
        self.metrics = metrics_collector

        # Pre-allocated storage
        self._buffer: List[Optional[T]] = [None] * max_size

        # Write position (next write)
        self._index = 0

        # Number of valid items (<= max_size)
        self._count = 0

        self._lock = Lock()

        # Stats
        self.items_added = 0
        self.items_overwritten = 0

    # --------------------------------------------------
    # Write path
    # --------------------------------------------------

    def append(self, item: T) -> None:
        overwritten = False

        with self._lock:
            if self._count == self.max_size:
                overwritten = True
                self.items_overwritten += 1
            else:
                self._count += 1

            self._buffer[self._index] = item
            self._index = (self._index + 1) % self.max_size
            self.items_added += 1

            size = self._count

        # Metrics outside lock
        if self.metrics:
            self.metrics.set_gauge(f"buffer.{self.name}.size", size)
            if overwritten:
                self.metrics.increment(f"buffer.{self.name}.overwritten")

    def get_latest(self, n: int) -> List[T]:
        """
        Return up to n most recent items (oldest → newest).
        """
        if n <= 0:
            return []

        with self._lock:
            n = min(n, self._count)
            if n == 0:
                return []

            start = (self._index - n) % self.max_size

            if start + n <= self.max_size:
                # Contiguous slice
                return self._buffer[start:start + n].copy()
            else:
                # Wrapped slice
                return (
                    self._buffer[start:].copy() +
                    self._buffer[: (start + n) % self.max_size].copy()
                )

    def get_latest_one(self) -> Optional[T]:
        """
        Fastest possible latest-item access.
        """
        with self._lock:
            if self._count == 0:
                return None
            return self._buffer[(self._index - 1) % self.max_size]

    def get_latest_pair(self) -> Optional[List[T]]:
        """
        Optimized for n=2 without list slicing overhead.
        """
        with self._lock:
            if self._count < 2:
                return None

            i2 = (self._index - 1) % self.max_size
            i1 = (self._index - 2) % self.max_size
            return [self._buffer[i1], self._buffer[i2]]
        
    def clear(self):
        """
        Clear all items.
        """
        with self._lock:
            self._index = 0
            self._count = 0

    def size(self) -> int:
        """
        Get current buffer size.
        """
        return self._count
    
    def is_full(self) -> bool:
        """
        Check if buffer is full.
        """
        return self._count == self.max_size
    
    def is_empty(self) -> bool:
        """
        Check if buffer is empty.
        """
        return self._count == 0
    
    def pop_left(self) -> Optional[T]:
        """
        Remove and return oldest item (leftmost).
        """
        with self._lock:
            if self._count == 0:
                return None

            idx = (self._index - self._count) % self.max_size
            item = self._buffer[idx]
            self._buffer[idx] = None

            self._count -= 1
            return item
    
    def pop_right(self) -> Optional[T]:
        """
        Remove and return newest item (rightmost).
        """
        with self._lock:
            if self._count == 0:
                return None

            idx = (self._index - 1) % self.max_size
            item = self._buffer[idx]
            self._buffer[idx] = None

            self._index = idx
            self._count -= 1

            return item
        
    def get(self, index: int) -> Optional[T]:
        """
        Get item by index.

        Args:
            index: Index (0 = oldest, -1 = newest)

        Returns:
            Item at index or None if out of bounds
        """
        with self._lock:
            if not self._buffer:
                return None
            # Handle negative indices
            if index < 0:
                index += len(self._buffer)
            if 0 <= index < len(self._buffer):
                return self._buffer[index]
            return None

    
class FrameBuffer(CircularBuffer):
    """
    Specialized circular buffer for camera frames.

    Adds timestamp tracking and frame synchronization support.
    """

    @dataclass(slots=True)
    class Frame:
        """
        Frame with metadata.
        """
        data: Any
        timestamp: float
        sequence: int
        metadata: Dict[str, Any] = field(default_factory=dict)

    def __init__(self, max_size: int, name: str = "", metrics_collector = None):
        """
        Initialize frame buffer.
        """
        super().__init__(max_size, name, metrics_collector)
        self._sequence = 0

    def add_frame(self, frame_data: Any, timestamp: Optional[float] = None, **metadata):
        """
        Add frame to buffer.

        Args:
            frame_data: Frame data (numpy array, etc)
            timestamp: Frame timestamp (uses current time if None)
            **metadata: Additional frame metadata
        """
        frame = self.Frame(
            data=frame_data,
            timestamp=timestamp if timestamp is not None else time.monotonic(),
            sequence=self._sequence,
            metadata=metadata
        )

        self._sequence += 1
        self.append(frame)

    def get_latest_frame(self) -> Optional[Frame]:
        """
        Get most recent frame.
        """
        return self.get_latest_one()
        
    def get_latest_pair(self) -> Optional[List[Frame]]:
        """
        Gets most recent two frames.
        """
        with self._lock:
            if self._count < 2:
                return None

            i2 = (self._index - 1) % self.max_size
            i1 = (self._index - 2) % self.max_size

            return [self._buffer[i1], self._buffer[i2]]
    
    def get_frame_by_timestamp(self, target_timestamp: float, tolerance: float = 0.1) -> Optional[Frame]:
        """
        Find frame closest to target timestamp.

        Args:
            target_timestamp: Target timestamp
            tolerance: Maximum acceptable time difference

        Returns:
            Closest frame within tolerance, or None
        """
        with self._lock:
            if self._count == 0:
                return None

            best = None
            best_delta = float("inf")

            for i in range(self._count):
                idx = (self._index - 1 - i) % self.max_size
                frame = self._buffer[idx]
                if frame is None:
                    continue

                delta = abs(frame.timestamp - target_timestamp)
                if delta < best_delta:
                    best = frame
                    best_delta = delta
                    if best_delta == 0:
                        break  # exact match

            if best is not None and best_delta <= tolerance:
                return best
            return None
        
    def pop_latest_frame(self) -> Optional["Frame"]:
        """
        Remove and return the newest frame.
        """
        return self.pop_right()

    def pop_oldest_frame(self) -> Optional["Frame"]:
        """
        Remove and return the oldest frame.
        """
        return self.pop_left()
    
# Queue utilities.
def drain_queue(queue: Queue, max_items: Optional[int] = None) -> List[Any]:
    """
    Drain all items from a queue without blocking.
    
    Args:
        queue: Queue to drain
        max_items: Maximum items to drain (None = all)
        
    Returns:
        List of items from queue
    """
    items = []
    count = 0
    
    while True:
        if max_items is not None and count >= max_items:
            break
        
        try:
            item = queue.get_nowait()
            items.append(item)
            count += 1
        except Empty:
            break
    
    return items

def transfer_queue(
    source: Queue,
    dest: Queue,
    max_items: Optional[int] = None,
    timeout: Optional[float] = None
) -> int:
    """
    Transfer items from one queue to another.
    
    Args:
        source: Source queue
        dest: Destination queue
        max_items: Maximum items to transfer
        timeout: Timeout for each transfer
        
    Returns:
        Number of items transferred
    """
    count = 0
    
    while True:
        if max_items is not None and count >= max_items:
            break
        
        try:
            item = source.get(timeout=timeout)
            dest.put(item, timeout=timeout)
            count += 1
        except (Empty, Full):
            break
    
    return count