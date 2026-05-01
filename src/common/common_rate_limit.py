"""
SnoBot Common Rate Limiting
Rate limiting algorithms for controlling operation frequency.

Provides multiple rate limiting strategies:
- Token Bucket: Allows bursts up to capacity
- Leaky Bucket: Smooth, consistent rate
- Fixed Window: Simple time-based windows
- Sliding Window: More accurate than fixed window
"""

from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from threading import Lock, RLock
from collections import deque
from enum import Enum
import time

class RateLimitStrategy(Enum):
    """
    Rate limiting strategies.
    """
    TOKEN_BUCKET = "token_bucket"
    LEAKY_BUCKET = "leaky_bucket"
    FIXED_WINDOW = "fixed_window"
    SLIDING_WINDOW = "sliding_window"

@dataclass
class RateLimitStats:
    """
    Rate limiter statistics.
    """
    name: str
    strategy: str
    max_rate_hz: float
    current_rate_hz: float
    allowed_count: int
    rejected_count: int
    total_requests: int
    rejection_rate: float
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert to dictionary.
        """
        return {
            "name": self.name,
            "strategy": self.strategy,
            "max_rate_hz": self.max_rate_hz,
            "current_rate_hz": self.current_rate_hz,
            "allowed_count": self.allowed_count,
            "rejected_count": self.rejected_count,
            "total_requests": self.total_requests,
            "rejection_rate": self.rejection_rate
        }
    
class RateLimiter:
    """
    Base class for rate limiters.
    
    All rate limiter implementations inherit from this.
    """
    
    def __init__(self, max_rate_hz: float, name: str = "", metrics_collector=None):
        """
        Initialize rate limiter.
        
        Args:
            max_rate_hz: Maximum operations per second
            name: Limiter name for identification
            metrics_collector: Optional metrics integration
        """
        if max_rate_hz <= 0:
            raise ValueError("max_rate_hz must be positive")
        
        self.max_rate_hz = max_rate_hz
        self.name = name or f"limiter_{id(self)}"
        self.metrics = metrics_collector
        
        self._lock = Lock()
        
        # Statistics.
        self.allowed_count = 0
        self.rejected_count = 0
        self._recent_timestamps: deque = deque(maxlen=100)
    
    def allow(self, tokens: float = 1.0) -> bool:
        """
        Check if operation is allowed.
        
        Args:
            tokens: Number of tokens to consume (default 1.0)
            
        Returns:
            True if operation is allowed
        """
        raise NotImplementedError("Subclass must implement allow()")
    
    def wait(self, tokens: float = 1.0, timeout: Optional[float] = None) -> bool:
        """
        Wait until operation is allowed.
        
        Args:
            tokens: Number of tokens to consume
            timeout: Maximum time to wait (None = wait forever)
            
        Returns:
            True if operation was allowed, False if timed out
        """
        start_time = time.time()
        
        while True:
            if self.allow(tokens):
                return True
            
            # Check timeout
            if timeout is not None:
                elapsed = time.time() - start_time
                if elapsed >= timeout:
                    return False
            
            # Sleep briefly before retry
            time.sleep(0.001)  # 1ms
    
    def reset(self):
        """
        Reset rate limiter state.
        """
        raise NotImplementedError("Subclass must implement reset()")
    
    def get_stats(self) -> RateLimitStats:
        """
        Get rate limiter statistics.
        
        Returns:
            RateLimitStats object
        """
        with self._lock:
            total = self.allowed_count + self.rejected_count
            rejection_rate = self.rejected_count / total if total > 0 else 0.0
            
            # Calculate current rate from recent timestamps.
            current_rate = 0.0
            if len(self._recent_timestamps) > 1:
                time_span = self._recent_timestamps[-1] - self._recent_timestamps[0]
                if time_span > 0:
                    current_rate = len(self._recent_timestamps) / time_span
            
            return RateLimitStats(
                name=self.name,
                strategy=self.__class__.__name__,
                max_rate_hz=self.max_rate_hz,
                current_rate_hz=current_rate,
                allowed_count=self.allowed_count,
                rejected_count=self.rejected_count,
                total_requests=total,
                rejection_rate=rejection_rate
            )
    
    def _record_allowed(self):
        """
        Record allowed operation.
        """
        self.allowed_count += 1
        self._recent_timestamps.append(time.time())
        
        if self.metrics:
            self.metrics.increment(f"rate_limit.{self.name}.allowed")
    
    def _record_rejected(self):
        """
        Record rejected operation.
        """
        self.rejected_count += 1
        
        if self.metrics:
            self.metrics.increment(f"rate_limit.{self.name}.rejected")

class TokenBucketRateLimiter(RateLimiter):
    """
    Token bucket rate limiter.

    Allow bursts up to bucket capacity, then enforces steady rate.
    Tokens are added at constant rate and consumed by operations.

    Good for: Allowing short bursts while maintaining average rate.
    Use case: SBCP commands (quick bursts, but limit sustained rate).
    """
    def __init__(self, max_rate_hz: float, burst_size: Optional[int] = None, name: str = "", metrics_collector = None):
        """
        Initialize token bucket.

        Args:
            max_rate_hz: Token refill rate (operations per second)
            burst_size: Bucket capacity (None = same as rate)
            name: Limiter name
            metrics_collector: Optional metrics
        """
        super().__init__(max_rate_hz, name, metrics_collector)

        self.burst_size = burst_size if burst_size is not None else int(max_rate_hz)
        self.tokens = float(self.burst_size)
        self.last_update = time.time()

    def allow(self, tokens: float = 1.0) -> bool:
        """
        Check if operation is allowed and consume tokens.
        """
        with self._lock:
            # Refill tokens based on time elapsed.
            now = time.time()
            elapsed = now - self.last_update
            self.tokens = min(self.burst_size, self.tokens + elapsed * self.max_rate_hz)
            self.last_update = now

            # Check if enough tokens.
            if self.tokens >= tokens:
                self.tokens -= tokens
                self._record_allowed()
                return True
            else:
                self._record_rejected()
                return False
            
    def reset(self):
        """
        Reset to full bucket.
        """
        with self._lock:
            self.tokens = float(self.burst_size)
            self.last_update = time.time()

    def available_tokens(self) -> float:
        """
        Get current available tokens.

        Returns:
            Number of tokens currently available
        """
        with self._lock:
            now = time.time()
            elapsed = now - self.last_update
            return min(self.burst_size, self.tokens + elapsed * self.max_rate_hz)
        
class LeakyBucketRateLimiter(RateLimiter):
    """
    Leaky bucket rate limiter.
    
    Operations are added to bucket and "leak out" at constant rate.
    If bucket overflows, operations are rejected.
    
    Good for: Smooth, constant rate output
    Use case: Telemetry publishing (prevent flooding receiver)
    """
    
    def __init__(self, max_rate_hz: float, bucket_size: Optional[int] = None, name: str = "", metrics_collector=None):
        """
        Initialize leaky bucket.
        
        Args:
            max_rate_hz: Leak rate (operations per second)
            bucket_size: Bucket capacity (None = 2x rate)
            name: Limiter name
            metrics_collector: Optional metrics
        """
        super().__init__(max_rate_hz, name, metrics_collector)
        
        self.bucket_size = bucket_size if bucket_size is not None else int(max_rate_hz * 2)
        self.bucket_level = 0.0
        self.last_leak = time.time()
    
    def allow(self, tokens: float = 1.0) -> bool:
        """
        Check if operation is allowed and add to bucket.
        """
        with self._lock:
            # Leak tokens based on time elapsed.
            now = time.time()
            elapsed = now - self.last_leak
            leaked = elapsed * self.max_rate_hz
            self.bucket_level = max(0.0, self.bucket_level - leaked)
            self.last_leak = now
            
            # Check if bucket has space.
            if self.bucket_level + tokens <= self.bucket_size:
                self.bucket_level += tokens
                self._record_allowed()
                return True
            else:
                self._record_rejected()
                return False
    
    def reset(self):
        """
        Reset bucket to empty.
        """
        with self._lock:
            self.bucket_level = 0.0
            self.last_leak = time.time()
    
    def bucket_utilization(self) -> float:
        """
        Get bucket utilization percentage.
        
        Returns:
            Utilization from 0.0 to 1.0
        """
        with self._lock:
            return self.bucket_level / self.bucket_size if self.bucket_size > 0 else 0.0
        
class FixedWindowRateLimiter(RateLimiter):
    """
    Fixed window rate limiter.
    
    Allows N operations per fixed time window.
    Simple but can allow bursts at window boundaries.
    
    Good for: Simple rate limiting with low overhead
    Use case: API rate limits, logging rate limits
    """
    
    def __init__(self, max_operations: int, window_sec: float, name: str = "", metrics_collector=None):
        """
        Initialize fixed window limiter.
        
        Args:
            max_operations: Maximum operations per window
            window_sec: Window duration in seconds
            name: Limiter name
            metrics_collector: Optional metrics
        """
        max_rate_hz = max_operations / window_sec
        super().__init__(max_rate_hz, name, metrics_collector)
        
        self.max_operations = max_operations
        self.window_sec = window_sec
        self.window_start = time.time()
        self.operations_in_window = 0
    
    def allow(self, tokens: float = 1.0) -> bool:
        """
        Check if operation is allowed in current window.
        """
        with self._lock:
            now = time.time()
            
            # Check if window has expired.
            if now - self.window_start >= self.window_sec:
                # Start new window.
                self.window_start = now
                self.operations_in_window = 0
            
            # Check if under limit.
            if self.operations_in_window + tokens <= self.max_operations:
                self.operations_in_window += tokens
                self._record_allowed()
                return True
            else:
                self._record_rejected()
                return False
    
    def reset(self):
        """
        Reset to new window.
        """
        with self._lock:
            self.window_start = time.time()
            self.operations_in_window = 0
    
    def operations_remaining(self) -> int:
        """
        Get operations remaining in current window.
        
        Returns:
            Number of operations remaining
        """
        with self._lock:
            return max(0, self.max_operations - int(self.operations_in_window))


class SlidingWindowRateLimiter(RateLimiter):
    """
    Sliding window rate limiter.
    
    Tracks operations in a sliding time window.
    More accurate than fixed window but higher overhead.
    
    Good for: Accurate rate limiting without boundary issues
    Use case: Critical rate limits that need precision
    """
    
    def __init__(self, max_operations: int, window_sec: float, name: str = "", metrics_collector=None):
        """
        Initialize sliding window limiter.
        
        Args:
            max_operations: Maximum operations per window
            window_sec: Window duration in seconds
            name: Limiter name
            metrics_collector: Optional metrics
        """
        max_rate_hz = max_operations / window_sec
        super().__init__(max_rate_hz, name, metrics_collector)
        
        self.max_operations = max_operations
        self.window_sec = window_sec
        self.timestamps: deque = deque()
    
    def allow(self, tokens: float = 1.0) -> bool:
        """
        Check if operation is allowed in sliding window.
        """
        with self._lock:
            now = time.time()
            
            # Remove timestamps outside window.
            cutoff = now - self.window_sec
            while self.timestamps and self.timestamps[0] < cutoff:
                self.timestamps.popleft()
            
            # Check if under limit.
            if len(self.timestamps) + tokens <= self.max_operations:
                for _ in range(int(tokens)):
                    self.timestamps.append(now)
                self._record_allowed()
                return True
            else:
                self._record_rejected()
                return False
    
    def reset(self):
        """
        Clear all timestamps.
        """
        with self._lock:
            self.timestamps.clear()
    
    def operations_in_window(self) -> int:
        """
        Get current operations in window.
        
        Returns:
            Number of operations in current window
        """
        with self._lock:
            now = time.time()
            cutoff = now - self.window_sec
            
            # Remove old timestamps.
            while self.timestamps and self.timestamps[0] < cutoff:
                self.timestamps.popleft()
            
            return len(self.timestamps)


class AdaptiveRateLimiter(RateLimiter):
    """
    Adaptive rate limiter that adjusts rate based on feedback.
    
    Increases rate when operations succeed, decreases when they fail.
    Useful for automatic backoff/recovery.
    
    Good for: Systems that need dynamic rate adjustment
    Use case: Network requests with automatic backoff
    """
    
    def __init__(
        self,
        initial_rate_hz: float,
        min_rate_hz: float,
        max_rate_hz: float,
        increase_factor: float = 1.1,
        decrease_factor: float = 0.5,
        name: str = "",
        metrics_collector=None
    ):
        """
        Initialize adaptive limiter.
        
        Args:
            initial_rate_hz: Starting rate
            min_rate_hz: Minimum allowed rate
            max_rate_hz: Maximum allowed rate
            increase_factor: Multiply rate by this on success
            decrease_factor: Multiply rate by this on failure
            name: Limiter name
            metrics_collector: Optional metrics
        """
        super().__init__(initial_rate_hz, name, metrics_collector)
        
        self.current_rate_hz = initial_rate_hz
        self.min_rate_hz = min_rate_hz
        self.max_rate_hz = max_rate_hz
        self.increase_factor = increase_factor
        self.decrease_factor = decrease_factor
        
        # Use token bucket internally.
        self._bucket = TokenBucketRateLimiter(max_rate_hz=initial_rate_hz, burst_size=int(initial_rate_hz), name=f"{name}_internal")
    
    def allow(self, tokens: float = 1.0) -> bool:
        """
        Check if operation is allowed.
        """
        return self._bucket.allow(tokens)
    
    def report_success(self):
        """
        Report successful operation (increase rate).
        """
        with self._lock:
            new_rate = min(self.max_rate_hz, self.current_rate_hz * self.increase_factor)
            
            if new_rate != self.current_rate_hz:
                self.current_rate_hz = new_rate
                self._bucket.max_rate_hz = new_rate
                
                if self.metrics:
                    self.metrics.record(f"rate_limit.{self.name}.current_rate_hz", new_rate)
    
    def report_failure(self):
        """
        Report failed operation (decrease rate).
        """
        with self._lock:
            new_rate = max(self.min_rate_hz, self.current_rate_hz * self.decrease_factor)
            
            if new_rate != self.current_rate_hz:
                self.current_rate_hz = new_rate
                self._bucket.max_rate_hz = new_rate
                
                if self.metrics:
                    self.metrics.record(f"rate_limit.{self.name}.current_rate_hz", new_rate)
    
    def reset(self):
        """
        Reset to initial rate.
        """
        with self._lock:
            self.current_rate_hz = self.max_rate_hz
            self._bucket.reset()

# Decorator for rate limiting.
def rate_limit(limiter: RateLimiter, wait: bool = False):
    """
    Decorator to rate limit function calls.

    Args:
        limiter: RateLimiter instance
        wait: If True, wait for rate limit; if False, raise exception

    Example:
        limiter = TokenBucketRateLimiter(max_rate_hz=10.0)

        @rate_limit(limiter)
        def send_command(cmd):
            # ... send command ...
            pass
    """
    def decorator(func: Callable) -> Callable:
        def wrapper(*args, **kwargs):
            if wait:
                limiter.wait()
            else:
                if not limiter.allow():
                    raise RuntimeError(f"Rate limit exceeded for {func.__name__}")
            
            return func(*args, **kwargs)
        
        return wrapper
    
    return decorator

# Rate limiter factory.
def create_rate_limiter(strategy: RateLimitStrategy, max_rate_hz: float, **kwargs) -> RateLimiter:
    """
    Factory function to create rate limiters.
    
    Args:
        strategy: Rate limiting strategy
        max_rate_hz: Maximum rate
        **kwargs: Strategy-specific arguments
        
    Returns:
        RateLimiter instance
    """
    if strategy == RateLimitStrategy.TOKEN_BUCKET:
        return TokenBucketRateLimiter(max_rate_hz=max_rate_hz, **kwargs)
    
    elif strategy == RateLimitStrategy.LEAKY_BUCKET:
        return LeakyBucketRateLimiter(max_rate_hz=max_rate_hz, **kwargs)
    
    elif strategy == RateLimitStrategy.FIXED_WINDOW:
        window_sec = kwargs.pop('window_sec', 1.0)
        max_ops = int(max_rate_hz * window_sec)
        return FixedWindowRateLimiter(max_operations=max_ops, window_sec=window_sec, **kwargs)
    
    elif strategy == RateLimitStrategy.SLIDING_WINDOW:
        window_sec = kwargs.pop('window_sec', 1.0)
        max_ops = int(max_rate_hz * window_sec)
        return SlidingWindowRateLimiter(max_operations=max_ops, window_sec=window_sec, **kwargs)
    
    else:
        raise ValueError(f"Unknown strategy: {strategy}")