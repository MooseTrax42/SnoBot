"""
SnoBot Common Metrics System
Performance monitoring and statistical aggregation.

Provides lightweight time-series metrics collection with statistical analysis,
threshold monitoring, and optional integration with logging and event systems.
"""

from typing import Dict, Any, Optional, List, Callable, Deque
from dataclasses import dataclass, field
from collections import deque, defaultdict
from threading import RLock, Thread, Event
from enum import Enum
import time
import numpy as np

from common.common_types import ErrorSeverity

class MetricType(Enum):
    """
    Types of metrics.
    """
    COUNTER = "counter"     # Monotonically increasing.
    GAUGE = "gauge"         # Point-in-time value.
    HISTOGRAM = "histogram" # Distribution of values.
    RATE = "rate"           # Events per second.

class AggregationType(Enum):
    """
    Statistical aggregation types.
    """
    MEAN = "mean"
    MEDIAN = "meadian"
    MIN = "min"
    MAX = "max"
    SUM = "sum"
    STDDEV = "stddev"
    PERCENTILE = "percentile"

@dataclass
class MetricSample:
    """
    Single metric sample.
    """
    value: float
    timestamp: float
    tags: Dict[str, str] = field(default_factory=dict)

@dataclass
class MetricStats:
    """
    Statistical summary of a metric.
    """
    name: str
    count: int
    mean: float
    median: float
    min: float
    max: float
    stddev: float
    p50: float
    p90: float
    p99: float
    sum: float
    rate_hz: float

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert to dictionary.
        """
        return {
            "name": self.name,
            "count": self.count,
            "mean": self.mean,
            "median": self.median,
            "min": self.min,
            "max": self.max,
            "stddev": self.stddev,
            "p50": self.p50,
            "p90": self.p90,
            "p99": self.p99,
            "sum": self.sum,
            "rate_hz": self.rate_hz
        }
    
@dataclass
class MetricThreshold:
    """
    Threshold configuration for a metric.
    """
    metric_name: str
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    callback: Optional[Callable[[str, float, 'MetricThreshold'], None]] = None
    severity: ErrorSeverity = ErrorSeverity.WARNING
    cooldown_sec: float = 60.0 # Minimum time between alerts.
    last_triggered: float = 0.0

    def check(self, value: float) -> bool:
        """
        Check if value violates threshold.

        Args:
            value: Metric value to check

        Returns:
            True if threshold violated and cooldown expired
        """
        now = time.time()

        # Check if in cooldown.
        if (now - self.last_triggered) < self.cooldown_sec:
            return False
        
        # Check thresholds.
        violated = False
        if self.min_value is not None and value < self.min_value:
            violated = True
        if self.max_value is not None and value > self.max_value:
            violated = True

        if violated:
            self.last_triggered = now
            if self.callback:
                self.callback(self.metric_name, value, self)
        
        return violated
    
@dataclass
class BatchedMetricUpdate:
    """
    Represents a batched metric update operation.
    """
    name: str
    value: float
    tags: Optional[Dict[str, str]]
    timestamp: float
    operation: str # record, increment, set_gauge, etc.

class Metric:
    """
    Time-series metric with statistical aggregation.

    Stores samples in a circular buffer and computes statistics on demand.
    Thread-safe for concurrent access.
    """
    def __init__(
        self,
        name: str, 
        metric_type: MetricType = MetricType.HISTOGRAM,
        window_size: int = 100,
        unit: str = ""
    ):
        """
        Initialize metric.

        Args:
            name: Metric name (e.g., "sbcp.command.latency_ms")
            metric_type: Type of metric
            window_size: Number of samples to keep in circular buffer
            unit: Unit of measurement (e.g., "ms", "fps", "C")
        """
        self.name = name
        self.metric_type = metric_type
        self.unit = unit
        self.window_size = window_size

        # Circular buffer for samples.
        self.samples: Deque[MetricSample] = deque(maxlen=window_size)
        self._lock = RLock()

        # Counter-specific state.
        self._counter_value = 0.0

        # Rate calculation.
        self._last_rate_calc = time.time()
        self._rate_count = 0

    def record(self, value: float, tags: Optional[Dict[str, str]] = None):
        """
        Record a metric value.

        Args:
            value: Metric value
            tags: Optional tags/labels for this sample.
        """
        with self._lock:
            sample = MetricSample(value=value, timestamp=time.time(), tags=tags or {})

            if self.metric_type == MetricType.COUNTER:
                # Counters accumulate.
                self._counter_value += value
                sample.value = self._counter_value

            self.samples.append(sample)

            # Update rate counter.
            self._rate_count += 1

    def increment(self, amount: float = 1.0, tags: Optional[Dict[str, str]] = None):
        """
        Increment counter (convenience for COUNTER type).

        Args:
            amount: Amount to increment by
            tags: Optional tags
        """
        self.record(amount, tags)

    def set(self, value: float, tags: Optional[Dict[str, str]] = None):
        """
        Set gauge value (convenience for GAUGE type).

        Args:
            value: Gauge value
            tags: Optional tags
        """
        self.record(value, tags)

    def get_samples(self, limit: Optional[int] = None) -> List[MetricSample]:
        """
        Get recent samples.

        Args:
            limit: Maximum number of samples to return

        Returns:
            List of samples (most recent last)
        """
        with self._lock:
            samples = list(self.samples)
            if limit: 
                samples = samples[-limit:]
            return samples
        
    def get_values(self, limit: Optional[int] = None) -> np.ndarray:
        """
        Get recent values as a numpy array.

        Args:
            limit: Maximum number of values.
        
        Returns:
            Numpy array of values
        """
        samples = self.get_samples(limit)
        if not samples:
            return np.array([])
        return np.array([s.value for s in samples])
    
    def count(self) -> int:
        """
        Get number of samples.
        """
        with self._lock:
            return len(self.samples)
        
    def mean(self) -> float:
        """
        Calculate mean value.
        """
        values = self.get_values()
        return float(np.mean(values)) if len(values) > 0 else 0.0
    
    def median(self) -> float:
        """
        Calculate median value.
        """
        values = self.get_values()
        return float(np.median(values)) if len(values) > 0 else 0.0
    
    def min(self) -> float:
        """
        Get minimum value.
        """
        values = self.get_values()
        return float(np.min(values)) if len(values) > 0 else 0.0
    
    def max(self) -> float:
        """
        Get maximum value.
        """
        values = self.get_values()
        return float(np.max(values)) if len(values) > 0 else 0.0
    
    def stddev(self) -> float:
        """
        Calculate standard deviation.
        """
        values = self.get_values()
        return float(np.std(values)) if len(values) > 1 else 0.0
    
    def sum(self) -> float:
        """
        Calculate sum of values.
        """
        values = self.get_values()
        return float(np.sum(values)) if len(values) > 0 else 0.0
    
    def percentile(self, p: float) -> float:
        """
        Calculate percentile.

        Args:
            p: Percentile (0 - 100)

        Returns:
            Value at percentile
        """
        values = self.get_values()
        return float(np.percentile(values, p)) if len(values) > 0 else 0.0
    
    def rate(self) -> float:
        """
        Calculate rate (samples per second).

        Returns:
            Rate in Hz
        """
        with self._lock:
            now = time.time()
            elapsed = now - self._last_rate_calc

            if elapsed < 1.0:
                return 0.0
            
            rate = self._rate_count / elapsed

            # Reset for next calculation.
            self._last_rate_calc = now
            self._rate_count = 0

            return rate
        
    def get_stats(self) -> MetricStats:
        """
        Get comprehensive statistics.

        Returns:
            MetricStats object with all statistics
        """        
        return MetricStats(
            name=self.name,
            count=self.count(),
            mean=self.mean(),
            median=self.median(),
            min=self.min(),
            max=self.max(),
            stddev=self.stddev(),
            p50=self.percentile(50),
            p90=self.percentile(90),
            p99=self.percentile(99),
            sum=self.sum(),
            rate_hz=self.rate()
        )
    
    def reset(self):
        """
        Reset metric and clear all samples.
        """
        with self._lock:
            self.samples.clear()
            self._counter_value = 0.0
            self._rate_count = 0
            self._last_rate_calc = time.time()
    
    def trend(self, method: str = "linear") -> float:
        """
        Calculate trend (rate of change).

        Args:
            method: Trend calculation method ("linear" or "exponential")

        Returns:
            Trend coefficient (positive = incerasing, negative = decreasing)
        """
        values = self.get_values()

        if len(values) > 2:
            return 0.0
        
        if method == "linear":
            # Simple linear regression.
            x = np.arange(len(values))
            slope, _ = np.polyfit(x, values, 1)
            return float(slope)
        else:
            # Exponential weighted moving average trend.
            alpha = 0.3
            ema = values[0]
            prev_ema = ema

            for val in values[1:]:
                ema = alpha * val + (1 - alpha) * ema
            
            return ema - prev_ema
        
    def is_stable(self, threshold: float = 0.1) -> bool:
        """
        Check if metric is stable (low variance).

        Args:
            threshold: Maximum coefficient of variation (stddev/mean)

        Returns: 
            true if stable
        """
        mean_val = self.mean()
        if mean_val == 0:
            return True
        
        cv = self.stddev() / mean_val
        return cv < threshold

class MetricCollector:
    """
    Central metric collection and monitoring system with automatic batching.

    Manages multiple metrics with threshold monitoring and optional 
    integration with logging and event systems.

    Batching mechanism buffers updates and flushes them, either:
    - When the bacth reaches a configured size
    - After a configured time interval
    - When explicitly flushed

    This reduces lock contention and improves performance when recording many
    metrics in high-frequency scenarios.
    """
    def __init__(
        self,
        enable_batching: bool = True, 
        batch_size: int = 100,
        flush_interval_sec: float = 1.0,
        logger: Optional[Any] = None,
        event_bus: Optional[Any] = None
    ):
        """
        Initialize metric collector.

        Args:
            enable_batching: Enable automatic batching of updates
            batch_size: Number of updates to batch before auto-flush
            flush_interval_sec: Seconds between automatic flushes
            logger: Optional logger for threshold violations
            event_bus: Optional event bus for notifications
        """
        self.metrics: Dict[str, Metric] = {}
        self.thresholds: Dict[str, List[MetricThreshold]] = defaultdict(list)
        self.logger = logger
        self.event_bus = event_bus
        self._lock = RLock()

        # Batching configuration.
        self.enable_batching = enable_batching
        self.batch_size = batch_size
        self.flush_interval_sec = flush_interval_sec

        # Background flush thread.
        self._flush_thread: Optional[Thread] = None
        self._stop_event = Event()

        # Batch buffer.
        self._batch_buffer: Deque[BatchedMetricUpdate] = deque()
        self._batch_lock = RLock()
        self._last_flush_time = time.time()

        if self.enable_batching:
            self._start_flush_thread()

    def _start_flush_thread(self):
        """
        Start background thread for perioding batch flushing.
        """
        if self._flush_thread is not None and self._flush_thread.is_alive():
            return
        
        self._stop_event.clear()
        self._flush_thread = Thread(target=self._flush_loop, daemon=True, name="MetricCollector-Flush")
        self._flush_thread.start()

    def _flush_loop(self):
        """
        Background loop that periodically flushes batched metrics.
        """
        while not self._stop_event.is_set():
            time.sleep(self.flush_interval_sec)
            self._flush_batch()

    def _add_to_batch(self, update: BatchedMetricUpdate):
        """
        Add update to batch buffer.

        Args:
            update: Metric update to batch
        """
        with self._batch_lock:
            self._batch_buffer.append(update)

            # Auto-flush if batch size reached.
            if len(self._batch_buffer) >= self.batch_size:
                self._flush_batch()

    def _flush_batch(self):
        """
        Flush all batched metric updates to actual metrics.

        This method processes all buffered updatees and applies them 
        to the underlying metrics in a single batch operation.
        """
        with self._batch_lock:
            if not self._batch_buffer:
                return
            
            # Extract all updates.
            updates = list(self._batch_buffer)
            self._batch_buffer.clear()
            self._last_flush_time= time.time()

        # Process updates outside of batch lock to minimize contention.
        for update in updates:
            metric = self.get_or_create(update.name)

            if update.operation == 'record':
                metric.record(update.value, update.tags)
            elif update.operation == 'increment':
                metric.increment(update.value, update.tags)
            elif update.operation == 'set_gauge':
                metric.set(update.value, update.tags)
            
            # Check thresholds.
            self._check_thresholds(update.name, update.value)

    def flush(self):
        """
        Manually flush all batched updates.

        This forces immediate processing of all buffered metric updates.
        Useful when metrics need to be up-to-date before reading or shutting down.
        """
        if self.enable_batching:
            self._flush_batch()

    def shutdown(self):
        """
        Shut down metric collector and flush remaining updates.

        Stops the background flush thread and processes any remaining 
        batched updates. Should be called before program exit.
        """
        if self._flush_thread is not None:
            self._stop_event.set()
            self._flush_thread.join(timeout=5.0)

        # Final flush.
        self._flush_batch()

    def get_or_create(
        self,
        name: str,
        metric_type: MetricType = MetricType.HISTOGRAM,
        window_size: int = 100,
        unit: str = ""      
    ) -> Metric:
        """
        Get existing metric or create new one.

        Args:
            name: Metric name
            metric_type: Type of metric
            window_size: Sample window size (default if None)
            unit: Unit of measurement

        Returns:
            Metric instance
        """
        with self._lock:
            if name not in self.metrics:
                self.metrics[name] = Metric(name=name, metric_type=metric_type, window_size=window_size, unit=unit)
            return self.metrics[name]
        
    def record(
        self, 
        name: str,
        value: float,
        tags: Optional[Dict[str, str]] = None,
        metric_type: MetricType = MetricType.HISTOGRAM
    ):
        """
        Record a metric value.

        Args:
            name: Metric name
            value: Metric value
            tags: Optional tags
            metric_type: Type of metric (for auto-creation)
        """
        if self.enable_batching:
            update = BatchedMetricUpdate(name=name, value=value, tags=tags, timestamp=time.time(), operation='record')
            self._add_to_batch(update)
        else:
            metric = self.get_or_create(name, metric_type)
            metric.record(value, tags)
            self._check_thresholds(name, value)

    def increment(
        self,
        name: str,
        amount: float = 1.0,
        tags: Optional[Dict[str, str]] = None
    ):
        """
        Increment a counter.
        
        Args:
            name: Metric name
            amount: Amount to increment
            tags: Optional tags
        """
        if self.enable_batching:
            update = BatchedMetricUpdate(name=name, value=amount, tags=tags, timestamp=time.time(), operation='increment')
            self._add_to_batch(update)
        else:
            metric = self.get_or_create(name, MetricType.COUNTER)
            metric.increment(amount, tags)
    
    def set_gauge(
        self,
        name: str,
        value: float,
        tags: Optional[Dict[str, str]] = None
    ):
        """
        Set a gauge value.
        
        Args:
            name: Metric name
            value: Gauge value
            tags: Optional tags
        """
        if self.enable_batching:
            update = BatchedMetricUpdate(name=name, value=value, tags=tags, timestamp=time.time(), operation='set_gauge')
            self._add_to_batch(update)
        else:
            metric = self.get_or_create(name, MetricType.GAUGE)
            metric.set(value, tags)
    
    def get(self, name: str) -> Optional[Metric]:
        """
        Get metric by name.
        
        Args:
            name: Metric name
            
        Returns:
            Metric instance or None
        """
        with self._lock:
            return self.metrics.get(name)
    
    def get_all_metrics(self) -> List[str]:
        """Get list of all metric names."""
        with self._lock:
            return list(self.metrics.keys())
    
    def add_threshold(
        self,
        metric_name: str,
        min_value: Optional[float] = None,
        max_value: Optional[float] = None,
        callback: Optional[Callable] = None,
        severity: ErrorSeverity = ErrorSeverity.WARNING,
        cooldown_sec: float = 60.0
    ) -> MetricThreshold:
        """
        Add threshold monitoring for a metric.
        
        Args:
            metric_name: Name of metric to monitor
            min_value: Minimum acceptable value
            max_value: Maximum acceptable value
            callback: Optional callback on threshold violation
            severity: Severity level for logging
            cooldown_sec: Minimum time between alerts
            
        Returns:
            MetricThreshold instance
        """
        threshold = MetricThreshold(
            metric_name=metric_name,
            min_value=min_value,
            max_value=max_value,
            callback=callback,
            severity=severity,
            cooldown_sec=cooldown_sec
        )
        
        with self._lock:
            self.thresholds[metric_name].append(threshold)
        
        return threshold
    
    def _check_thresholds(self, metric_name: str, value: float):
        """
        Check if value violates any thresholds.
        
        Args:
            metric_name: Metric name
            value: Current value
        """
        thresholds = self.thresholds.get(metric_name, [])
        
        for threshold in thresholds:
            if threshold.check(value):
                self._handle_threshold_violation(metric_name, value, threshold)
    
    def _handle_threshold_violation(
        self,
        metric_name: str,
        value: float,
        threshold: MetricThreshold
    ):
        """
        Handle threshold violation.
        
        Args:
            metric_name: Metric name
            value: Violating value
            threshold: Violated threshold
        """
        # Log if logger available
        if self.logger:
            msg = f"Metric '{metric_name}' threshold violated: value={value}"
            if threshold.min_value is not None:
                msg += f", min={threshold.min_value}"
            if threshold.max_value is not None:
                msg += f", max={threshold.max_value}"
            
            if threshold.severity == ErrorSeverity.CRITICAL:
                self.logger.log_error("METRIC_THRESHOLD", msg)
            else:
                self.logger.log_warning("METRIC_THRESHOLD", msg)
        
        # Emit event if event bus available
        if self.event_bus:
            from common.common_events import EventType, EventPriority
            
            priority = EventPriority.CRITICAL if threshold.severity == ErrorSeverity.CRITICAL else EventPriority.HIGH
            
            self.event_bus.emit(
                EventType.WARNING_ISSUED,
                source="metrics",
                priority=priority,
                metric_name=metric_name,
                value=value,
                threshold_min=threshold.min_value,
                threshold_max=threshold.max_value,
                severity=threshold.severity.value
            )
    
    def get_stats(self, name: str) -> Optional[MetricStats]:
        """
        Get statistics for a metric.
        
        Args:
            name: Metric name
            
        Returns:
            MetricStats or None
        """
        # Ensure all updates are processed.
        if self.enable_batching:
            self._flush_batch()

        metric = self.get(name)
        return metric.get_stats() if metric else None
    
    def get_all_stats(self) -> Dict[str, MetricStats]:
        """
        Get statistics for all metrics.
        
        Returns:
            Dictionary of metric name -> stats
        """
        # Ensure all updates are processed.
        if self.enable_batching:
            self._flush_batch()

        with self._lock:
            return {
                name: metric.get_stats()
                for name, metric in self.metrics.items()
            }
        
    def get_batch_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the batching system.

        Returns:
            Dictionary with batching statistics.
        """
        with self._batch_lock:
            return {
                "enabled": self.enable_batching,
                "buffer_size": len(self._batch_buffer),
                "batch_size_limit": self.batch_size,
                "flush_interval_sec": self.flush_interval_sec,
                "time_since_last_flush": time.time() - self._last_flush_time,
                "thread_alive": self._flush_thread.is_alive() if self._flush_thread else False
            }
    
    def reset(self, name: Optional[str] = None):
        """
        Reset metric(s).
        
        Args:
            name: Metric name to reset (None = reset all)
        """
        # Flush before resetting.
        if self.enable_batching:
            self._flush_batch()

        with self._lock:
            if name:
                metric = self.metrics.get(name)
                if metric:
                    metric.reset()
            else:
                for metric in self.metrics.values():
                    metric.reset()
    
    def summary(self) -> str:
        """
        Get text summary of all metrics.
        
        Returns:
            Formatted summary string
        """
        # Flush before generating summary.
        if self.enable_batching:
            self._flush_batch()

        lines = ["=== Metric Summary ===\n"]
        
        stats = self.get_all_stats()
        for name, stat in sorted(stats.items()):
            metric = self.metrics[name]
            lines.append(f"{name} [{metric.unit}]:")
            lines.append(f"  Count: {stat.count}")
            lines.append(f"  Mean: {stat.mean:.2f}")
            lines.append(f"  Median: {stat.median:.2f}")
            lines.append(f"  Min: {stat.min:.2f}, Max: {stat.max:.2f}")
            lines.append(f"  StdDev: {stat.stddev:.2f}")
            lines.append(f"  90: {stat.p90:.2f}, P99: {stat.p99:.2f}")
            lines.append(f"  Rate: {stat.rate_hz:.2f} Hz\n")
        
        # Add batching stats if enabled.
        if self.enable_batching:
            batch_stats = self.get_batch_stats()
            lines.append("\n=== Batching Stats ===")
            lines.append(f"  Buffer Size: {batch_stats['buffer_size']}/{batch_stats['batch_size_limit']}")
            lines.append(f"  Flush Interval: {batch_stats['flush_interval_sec']:.2f}s")
            lines.append(f"  Time Since Last Flush: {batch_stats['time_since_last_flush']:.2f}s")

        return "\n".join(lines)
    
# Global metric collector instance.
_global_metrics: Optional[MetricCollector] = None
_metrics_lock = RLock()

def get_metrics() -> MetricCollector:
    """
    Get or create global metrics collector.

    Returns:
        Global MetricCollector instance
    """
    global _global_metrics
    
    if _global_metrics is None:
        with _metrics_lock:
            if _global_metrics is None:
                _global_metrics = MetricCollector()
    
    return _global_metrics


def reset_metrics():
    """
    Reset global metrics collector (mainly for testing).
    """
    global _global_metrics
    with _metrics_lock:
        _global_metrics = None


# Convenience functions using global collector
def record(name: str, value: float, tags: Optional[Dict[str, str]] = None):
    """
    Record metric value to global collector.
    """
    get_metrics().record(name, value, tags)


def increment(name: str, amount: float = 1.0, tags: Optional[Dict[str, str]] = None):
    """
    Increment counter in global collector.
    """
    get_metrics().increment(name, amount, tags)


def set_gauge(name: str, value: float, tags: Optional[Dict[str, str]] = None):
    """
    Set gauge value in global collector.
    """
    get_metrics().set_gauge(name, value, tags)