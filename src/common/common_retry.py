"""
SnoBot Common Retry Logic
Configurable retry with exponential backoff and jitter.

Provides retry functionality for:
- Failed commands
- Network reconnection
- Camera initialization
- Model loading
- File operations
"""

from typing import Callable, TypeVar, Optional, Type, Tuple, Any, List
from dataclasses import dataclass, field
from enum import Enum
from functools import wraps
import time
import random

T = TypeVar('T')

class BackoffStrategy(Enum):
    """
    Backoff strategies for retries.
    """
    CONSTANT = "constant"       # Same delay between retries.
    LINEAR = "LINEAR"           # Linearly increasing delay.
    EXPONENTIAL = "exponential" # Exponentially increasing delay.
    FIBONACCI = "fibonacci"     # Fibonacci sequence delays.

@dataclass
class RetryConfig:
    """
    Configuration for retry behavior.
    """
    max_attempts: int = 3
    initial_delay_sec: float = 0.1
    max_delay_sec: float = 60.0
    backoff_strategy: BackoffStrategy = BackoffStrategy.EXPONENTIAL
    backoff_factor: float = 2.0
    jitter: bool = True
    jitter_factor: float = 0.1
    exceptions: Tuple[Type[Exception], ...] = (Exception,)
    
    def validate(self):
        """
        Validate configuration.
        """
        if self.max_attempts < 1:
            raise ValueError("max_attempts must be at least 1")
        
        if self.initial_delay_sec < 0:
            raise ValueError("initial_delay_sec cannot be negative")
        
        if self.max_delay_sec < self.initial_delay_sec:
            raise ValueError("max_delay_sec must be >= initial_delay_sec")
        
        if self.backoff_factor <= 0:
            raise ValueError("backoff_factor must be positive")
        
        if not 0 <= self.jitter_factor <= 1:
            raise ValueError("jitter_factor must be between 0 and 1")
        
@dataclass
class RetryStats:
    """
    Statistics for retry operations.
    """
    total_attempts: int = 0
    successful_attempts: int = 0
    failed_attempts: int = 0
    total_retries: int = 0
    total_delay_sec: float = 0.0
    
    def to_dict(self) -> dict:
        """
        Convert to dictionary.
        """
        return {
            "total_attempts": self.total_attempts,
            "successful_attempts": self.successful_attempts,
            "failed_attempts": self.failed_attempts,
            "total_retries": self.total_retries,
            "total_delay_sec": self.total_delay_sec,
            "success_rate": self.successful_attempts / self.total_attempts if self.total_attempts > 0 else 0.0
        }
    
class RetryExhaustedError(Exception):
    """
    Raised when all retry attempts are exhausted.
    """
    def __init__(self, attempts: int, last_exception: Exception):
        self.attempts = attempts
        self.last_exception = last_exception
        super().__init__(
            f"All {attempts} retry attempts exhausted. "
            f"Last exception: {type(last_exception).__name__}: {last_exception}"
        )

class Retrier:
    """
    Retry executor with configurable backoff.
    
    Executes a function with automatic retry on failure.
    """
    def __init__(
        self,
        config: Optional[RetryConfig] = None,
        name: str = "",
        metrics_collector=None,
        on_retry: Optional[Callable[[int, Exception, float], None]] = None
    ):
        """
        Initialize retrier.
        
        Args:
            config: Retry configuration
            name: Retrier name for identification
            metrics_collector: Optional metrics integration
            on_retry: Optional callback(attempt, exception, delay)
        """
        self.config = config or RetryConfig()
        self.config.validate()
        
        self.name = name or f"retrier_{id(self)}"
        self.metrics = metrics_collector
        self.on_retry = on_retry
        
        # Statistics.
        self.stats = RetryStats()
        
        # Fibonacci sequence cache.
        self._fib_cache = [1, 1]
    
    def execute(self, func: Callable[..., T], *args, **kwargs) -> T:
        """
        Execute function with retry logic.
        
        Args:
            func: Function to execute
            *args: Positional arguments for func
            **kwargs: Keyword arguments for func
            
        Returns:
            Result from successful function call
            
        Raises:
            RetryExhaustedError: If all attempts fail
        """
        last_exception: Optional[Exception] = None
        
        for attempt in range(1, self.config.max_attempts + 1):
            self.stats.total_attempts += 1
            
            try:
                # Attempt function call.
                result = func(*args, **kwargs)
                
                # Success.
                self.stats.successful_attempts += 1
                
                if self.metrics:
                    self.metrics.increment(f"retry.{self.name}.success")
                    if attempt > 1:
                        self.metrics.record(f"retry.{self.name}.attempts", attempt)
                
                return result
                
            except self.config.exceptions as e:
                last_exception = e
                self.stats.failed_attempts += 1
                
                # Check if we should retry.
                if attempt >= self.config.max_attempts:
                    # Exhausted all attempts.
                    if self.metrics:
                        self.metrics.increment(f"retry.{self.name}.exhausted")
                    
                    raise RetryExhaustedError(attempt, e)
                
                # Calculate delay.
                delay = self._calculate_delay(attempt)
                self.stats.total_retries += 1
                self.stats.total_delay_sec += delay
                
                # Record metrics.
                if self.metrics:
                    self.metrics.increment(f"retry.{self.name}.retry")
                    self.metrics.record(f"retry.{self.name}.delay_sec", delay)
                
                # Call retry callback.
                if self.on_retry:
                    try:
                        self.on_retry(attempt, e, delay)
                    except Exception:
                        pass  # Don't let callback errors stop retry.
                
                # Wait before retry.
                time.sleep(delay)
        
        # Should never reach here, but just in case.
        raise RetryExhaustedError(self.config.max_attempts, last_exception or Exception("Unknown error"))
    
    def _calculate_delay(self, attempt: int) -> float:
        """
        Calculate delay before next retry.
        
        Args:
            attempt: Current attempt number (1-indexed)
            
        Returns:
            Delay in seconds
        """
        # Calculate base delay based on strategy.
        if self.config.backoff_strategy == BackoffStrategy.CONSTANT:
            delay = self.config.initial_delay_sec
        
        elif self.config.backoff_strategy == BackoffStrategy.LINEAR:
            delay = self.config.initial_delay_sec * attempt
        
        elif self.config.backoff_strategy == BackoffStrategy.EXPONENTIAL:
            delay = self.config.initial_delay_sec * (self.config.backoff_factor ** (attempt - 1))
        
        elif self.config.backoff_strategy == BackoffStrategy.FIBONACCI:
            fib_index = attempt - 1
            
            # Extend fibonacci cache if needed.
            while len(self._fib_cache) <= fib_index:
                self._fib_cache.append(
                    self._fib_cache[-1] + self._fib_cache[-2]
                )
            
            delay = self.config.initial_delay_sec * self._fib_cache[fib_index]
        
        else:
            delay = self.config.initial_delay_sec
        
        # Apply max delay cap.
        delay = min(delay, self.config.max_delay_sec)
        
        # Apply jitter if enabled.
        if self.config.jitter:
            jitter_range = delay * self.config.jitter_factor
            jitter = random.uniform(-jitter_range, jitter_range)
            delay = max(0, delay + jitter)
        
        return delay
    
    def get_stats(self) -> RetryStats:
        """
        Get retry statistics.
        """
        return self.stats
    
    def reset_stats(self):
        """
        Reset statistics.
        """
        self.stats = RetryStats()

# Decorator for Retry.
def retry(
    max_attempts: int = 3,
    initial_delay_sec: float = 0.1,
    max_delay_sec: float = 60.0,
    backoff_strategy: BackoffStrategy = BackoffStrategy.EXPONENTIAL,
    backoff_factor: float = 2.0,
    jitter: bool = True,
    exceptions: Tuple[Type[Exception], ...] = (Exception,),
    on_retry: Optional[Callable[[int, Exception, float], None]] = None
):
    """
    Decorator to add retry logic to a function.
    
    Args:
        max_attempts: Maximum number of attempts
        initial_delay_sec: Initial delay between retries
        max_delay_sec: Maximum delay between retries
        backoff_strategy: Backoff strategy to use
        backoff_factor: Factor for exponential backoff
        jitter: Whether to add jitter to delays
        exceptions: Tuple of exceptions to catch and retry
        on_retry: Optional callback(attempt, exception, delay)
        
    Example:
        @retry(max_attempts=3, backoff_strategy=BackoffStrategy.EXPONENTIAL)
        def unreliable_function():
            # ... might fail ...
            pass
    """
    config = RetryConfig(
        max_attempts=max_attempts,
        initial_delay_sec=initial_delay_sec,
        max_delay_sec=max_delay_sec,
        backoff_strategy=backoff_strategy,
        backoff_factor=backoff_factor,
        jitter=jitter,
        exceptions=exceptions
    )
    
    def decorator(func: Callable) -> Callable:
        retrier = Retrier(config=config, name=func.__name__, on_retry=on_retry)
        
        @wraps(func)
        def wrapper(*args, **kwargs):
            return retrier.execute(func, *args, **kwargs)
        
        # Attach retrier for access to stats.
        wrapper.retrier = retrier
        
        return wrapper
    
    return decorator

# Predefined retry configurations.
class RetryPresets:
    """Predefined retry configurations for common scenarios."""
    
    @staticmethod
    def quick() -> RetryConfig:
        """
        Quick retry for fast operations.
        
        3 attempts, 100ms initial delay, exponential backoff up to 1s
        """
        return RetryConfig(
            max_attempts=3,
            initial_delay_sec=0.1,
            max_delay_sec=1.0,
            backoff_strategy=BackoffStrategy.EXPONENTIAL,
            backoff_factor=2.0,
            jitter=True
        )
    
    @staticmethod
    def network() -> RetryConfig:
        """
        Network retry for connection issues.
        
        5 attempts, 500ms initial delay, exponential backoff up to 30s
        """
        return RetryConfig(
            max_attempts=5,
            initial_delay_sec=0.5,
            max_delay_sec=30.0,
            backoff_strategy=BackoffStrategy.EXPONENTIAL,
            backoff_factor=2.0,
            jitter=True
        )
    
    @staticmethod
    def persistent() -> RetryConfig:
        """
        Persistent retry for critical operations.
        
        10 attempts, 1s initial delay, exponential backoff up to 60s
        """
        return RetryConfig(
            max_attempts=10,
            initial_delay_sec=1.0,
            max_delay_sec=60.0,
            backoff_strategy=BackoffStrategy.EXPONENTIAL,
            backoff_factor=2.0,
            jitter=True
        )
    
    @staticmethod
    def constant(attempts: int = 3, delay_sec: float = 1.0) -> RetryConfig:
        """
        Constant delay retry.
        
        Args:
            attempts: Number of attempts
            delay_sec: Delay between attempts
        """
        return RetryConfig(
            max_attempts=attempts,
            initial_delay_sec=delay_sec,
            max_delay_sec=delay_sec,
            backoff_strategy=BackoffStrategy.CONSTANT,
            jitter=False
        )
    
    @staticmethod
    def fibonacci(attempts: int = 7, base_delay_sec: float = 0.1) -> RetryConfig:
        """
        Fibonacci backoff retry.
        
        Args:
            attempts: Number of attempts
            base_delay_sec: Base delay (multiplied by fibonacci number)
        """
        return RetryConfig(
            max_attempts=attempts,
            initial_delay_sec=base_delay_sec,
            max_delay_sec=60.0,
            backoff_strategy=BackoffStrategy.FIBONACCI,
            jitter=True
        )
    
# Async retry support.
class AsyncRetrier:
    """
    Async retry executor for async functions.
    
    Similar to Retrier but for async/await functions.
    """
    
    def __init__(
        self,
        config: Optional[RetryConfig] = None,
        name: str = "",
        metrics_collector=None,
        on_retry: Optional[Callable[[int, Exception, float], None]] = None
    ):
        """
        Initialize async retrier.
        """
        self.config = config or RetryConfig()
        self.config.validate()
        
        self.name = name or f"async_retrier_{id(self)}"
        self.metrics = metrics_collector
        self.on_retry = on_retry
        
        self.stats = RetryStats()
        self._fib_cache = [1, 1]
    
    async def execute(self, func: Callable[..., T], *args, **kwargs) -> T:
        """
        Execute async function with retry logic.
        
        Args:
            func: Async function to execute
            *args: Positional arguments
            **kwargs: Keyword arguments
            
        Returns:
            Result from successful function call
            
        Raises:
            RetryExhaustedError: If all attempts fail
        """
        import asyncio
        
        last_exception: Optional[Exception] = None
        
        for attempt in range(1, self.config.max_attempts + 1):
            self.stats.total_attempts += 1
            
            try:
                result = await func(*args, **kwargs)
                self.stats.successful_attempts += 1
                
                if self.metrics:
                    self.metrics.increment(f"retry.{self.name}.success")
                
                return result
                
            except self.config.exceptions as e:
                last_exception = e
                self.stats.failed_attempts += 1
                
                if attempt >= self.config.max_attempts:
                    if self.metrics:
                        self.metrics.increment(f"retry.{self.name}.exhausted")
                    
                    raise RetryExhaustedError(attempt, e)
                
                delay = self._calculate_delay(attempt)
                self.stats.total_retries += 1
                self.stats.total_delay_sec += delay
                
                if self.metrics:
                    self.metrics.increment(f"retry.{self.name}.retry")
                    self.metrics.record(f"retry.{self.name}.delay_sec", delay)
                
                if self.on_retry:
                    try:
                        self.on_retry(attempt, e, delay)
                    except Exception:
                        pass
                
                await asyncio.sleep(delay)
        
        raise RetryExhaustedError(self.config.max_attempts, last_exception or Exception("Unknown error"))
    
    def _calculate_delay(self, attempt: int) -> float:
        """
        Calculate delay (same as sync version).
        """
        if self.config.backoff_strategy == BackoffStrategy.CONSTANT:
            delay = self.config.initial_delay_sec
        elif self.config.backoff_strategy == BackoffStrategy.LINEAR:
            delay = self.config.initial_delay_sec * attempt
        elif self.config.backoff_strategy == BackoffStrategy.EXPONENTIAL:
            delay = self.config.initial_delay_sec * (self.config.backoff_factor ** (attempt - 1))
        elif self.config.backoff_strategy == BackoffStrategy.FIBONACCI:
            fib_index = attempt - 1
            while len(self._fib_cache) <= fib_index:
                self._fib_cache.append(
                    self._fib_cache[-1] + self._fib_cache[-2]
                )
            delay = self.config.initial_delay_sec * self._fib_cache[fib_index]
        else:
            delay = self.config.initial_delay_sec
        
        delay = min(delay, self.config.max_delay_sec)
        
        if self.config.jitter:
            jitter_range = delay * self.config.jitter_factor
            jitter = random.uniform(-jitter_range, jitter_range)
            delay = max(0, delay + jitter)
        
        return delay
    
# Context manager for Retry.
class RetryContext:
    """
    Context manager for retry operations.
    
    Usage:
        with RetryContext(max_attempts=3) as retry:
            result = retry.execute(lambda: risky_operation())
    """
    def __init__(
        self,
        max_attempts: int = 3,
        initial_delay_sec: float = 0.1,
        backoff_strategy: BackoffStrategy = BackoffStrategy.EXPONENTIAL,
        **kwargs
    ):
        """
        Initialize retry context.
        """
        config = RetryConfig(max_attempts=max_attempts, initial_delay_sec=initial_delay_sec, backoff_strategy=backoff_strategy, **kwargs)
        self.retrier = Retrier(config=config)
    
    def __enter__(self):
        """
        Enter context.
        """
        return self.retrier
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exit context.
        """
        return False  # Don't suppress exceptions
    
    def execute(self, func: Callable[..., T], *args, **kwargs) -> T:
        """
        Execute function with retry.
        """
        return self.retrier.execute(func, *args, **kwargs)