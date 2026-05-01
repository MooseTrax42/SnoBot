"""
SnoBot Common Module
Shared utilities, types, and infrastructure for sub-modules.

This module provides:
- Event system for pub/sub messaging
- Metrics collection and monitoring
- Configuration management
- Queue utilities with monitoring
- Watchdog timers
- State machines
- Rate limiting
- Retry logic with backoff
- Logging utilities
- Common types and errors
"""

# Types
from common.common_types import (
    ErrorSeverity,
    LogLevel,
    LogFormat,
    SystemState,
    Timestamp,
    VersionInfo,
    Statistics,
    clamp,
    clamp_int,
    normalize,
    lerp,
    map_range,
    DEFAULT_TIMEOUT_S,
    SHORT_TIMEOUT_S,
    LONG_TIMEOUT_S,
    DEFAULT_QUEUE_SIZE,
    SMALL_QUEUE_SIZE,
    LARGE_QUEUE_SIZE,
    DEFAULT_MAX_RETRIES,
    MAX_CONSECUTIVE_FAILURES
)

# Errors
from common.common_errors import (
    BaseError,
    ValidationError,
    CommunicationError,
    ParseError,
    TimeoutError,
    ResourceError,
    ConfigurationError,
    StateError,
    ThreadError,
    InitializationError,
    is_recoverable_error,
    get_error_severity,
    format_error_for_logging
)

# Logging
from common.common_logging import (
    LogEntry,
    BaseLogger
)

from common.common_events import (
    Event, 
    EventType, 
    EventPriority, 
    EventBus, 
    EventSubscription,
    get_event_bus,
    reset_event_bus,
    subscribe,
    unsubscribe,
    publish,
    emit,
    event_handler
)

from common.common_metrics import (
    Metric, 
    MetricType,
    MetricStats,
    MetricCollector,
    MetricThreshold,
    get_metrics,
    reset_metrics,
    record,
    increment,
    set_gauge
)

from common.common_config import (
    ConfigBase,
    ConfigError,
    ConfigValidationError,
    ConfigFormat,
    ConfigManager,
    SBCPConfig,
    SBVSConfig,
    CameraConfig,
    StereoCameraConfig,
    DetectorConfig,
    LoggingConfig,
    MetricsConfig,
    SnoBotConfig,
    get_config_manager,
    get_config
)

from common.common_queues import (
    QueueOverflowPolicy,
    QueueStats,
    MonitoredQueue,
    MonitoredPriorityQueue,
    CircularBuffer,
    drain_queue,
    transfer_queue
)

from common.common_watchdog import (
    Watchdog,
    WatchdogConfig,
    WatchdogManager,
    WatchdogState,
    WatchdogStats,
    get_watchdog,
    get_watchdog_manager,
    reset_watchdog_manager,
    create_watchdog,
    feed_watchdog
)

from common.common_state_machine import (
    Transition,
    StateInfo,
    StateMachine,
    TimedStateMachine,
    StateMachineError,
    InvalidTransitionError
)

from common.common_rate_limit import (
    RateLimiter,
    RateLimitStats,
    RateLimitStrategy,
    TokenBucketRateLimiter,
    LeakyBucketRateLimiter,
    FixedWindowRateLimiter,
    SlidingWindowRateLimiter,
    AdaptiveRateLimiter,
    rate_limit,
    create_rate_limiter    
)

from common.common_retry import (
    BackoffStrategy,
    RetryConfig,
    RetryStats,
    RetryExhaustedError,
    Retrier,
    RetryPresets,
    RetryContext,
    AsyncRetrier,
    retry
)

__all__ = [
    # Types
    "ErrorSeverity",
    "LogLevel",
    "LogFormat",
    "SystemState",
    "Timestamp",
    "VersionInfo",
    "Statistics",
    "clamp",
    "clamp_int",
    "normalize",
    "lerp",
    "map_range",
    "DEFAULT_TIMEOUT_S",
    "SHORT_TIMEOUT_S",
    "LONG_TIMEOUT_S",
    "DEFAULT_QUEUE_SIZE",
    "SMALL_QUEUE_SIZE",
    "LARGE_QUEUE_SIZE",
    "DEFAULT_MAX_RETRIES",
    "MAX_CONSECUTIVE_FAILURES",
    
    # Errors
    "BaseError",
    "ValidationError",
    "CommunicationError",
    "ParseError",
    "TimeoutError",
    "ResourceError",
    "ConfigurationError",
    "StateError",
    "ThreadError",
    "InitializationError",
    "is_recoverable_error",
    "get_error_severity",
    "format_error_for_logging",
    
    # Logging
    "LogEntry",
    "BaseLogger",
    
    # Events
    "Event",
    "EventType",
    "EventPriority",
    "EventBus",
    "EventSubscription",
    "get_event_bus",
    "reset_event_bus",
    "subscribe",
    "unsubscribe",
    "publish",
    "emit",
    "event_handler",
    
    # Metrics
    "Metric",
    "MetricType",
    "MetricStats",
    "MetricCollector",
    "MetricThreshold",
    "get_metrics",
    "reset_metrics",
    "record",
    "increment",
    "set_gauge",
    
    # Config
    "ConfigBase",
    "ConfigError",
    "ConfigValidationError",
    "ConfigFormat",
    "ConfigManager",
    "SBCPConfig",
    "SBVSConfig",
    "CameraConfig",
    "StereoCameraConfig",
    "DetectorConfig",
    "LoggingConfig",
    "MetricsConfig",
    "SnoRobotConfig",
    "get_config_manager",
    "get_config",
    
    # Queues
    "QueueOverflowPolicy",
    "QueueStats",
    "MonitoredQueue",
    "MonitoredPriorityQueue",
    "CircularBuffer",
    "FrameBuffer",
    "drain_queue",
    "transfer_queue",
    
    # Watchdog
    "WatchdogState",
    "WatchdogConfig",
    "WatchdogStats",
    "Watchdog",
    "WatchdogManager",
    "get_watchdog_manager",
    "reset_watchdog_manager",
    "create_watchdog",
    "feed_watchdog",
    "get_watchdog",
    
    # State Machine
    "Transition",
    "StateInfo",
    "StateMachine",
    "TimedStateMachine",
    "StateMachineError",
    "InvalidTransitionError",
    
    # Rate Limiting
    "RateLimitStrategy",
    "RateLimitStats",
    "RateLimiter",
    "TokenBucketRateLimiter",
    "LeakyBucketRateLimiter",
    "FixedWindowRateLimiter",
    "SlidingWindowRateLimiter",
    "AdaptiveRateLimiter",
    "rate_limit",
    "create_rate_limiter",
    
    # Retry
    "BackoffStrategy",
    "RetryConfig",
    "RetryStats",
    "RetryExhaustedError",
    "Retrier",
    "AsyncRetrier",
    "RetryContext",
    "RetryPresets",
    "retry",
]