"""
SnoBot Common Error System
Base error classes for sub-systems.
"""

from typing import Optional
from common.common_types import ErrorSeverity

class BaseError(Exception):
    """
    Base exception for all SnoBot errors.
    
    Provides common error handling functionality that can be extended
    by system-specific error classes.
    """
    def __init__(self, message: str, detail: Optional[str] = None, severity: ErrorSeverity = ErrorSeverity.ERROR):
        """
        Initialize base error.
        
        Args:
            message: Error message
            detail: Additional error details
            severity: Error severity level
        """
        self.message = message
        self.detail = detail
        self.severity = severity
        super().__init__(self._format_message())
        
    def _format_message(self) -> str:
        """
        Format error message with detail.
        
        Returns:
            Formatted error string.
        """
        parts = [self.message]
        if self.detail:
            parts.append(f"- {self.detail}")
        return " ".join(parts)
    
    def is_recoverable(self) -> bool:
        """
        Check if error is potentially recoverable.
        
        Returns:
            True if error severity suggessts recoverability
        """
        return self.severity in (ErrorSeverity.INFO, ErrorSeverity.WARNING)
    
    def is_critical(self) -> bool:
        """
        Check if error is critical.
        
        Returns:
            True if error is critical severity.
        """
        return self.severity == ErrorSeverity.CRITICAL
    
    def to_dict(self) -> dict:
        """
        Convert error to dictionary for logging/serialization.
        
        Returns:
            Dictionary representation of error
        """
        return {
            "type": self.__class__.__name__,
            "message": self.message,
            "detail": self.detail,
            "severity": self.severity.value
        }
        
class ValidationError(BaseError):
    """
    Raised when parameter validation fails.
    
    Common to sub-systems for validating configuration and input parameters.
    """
    def __init__(self, message: str, param_name: Optional[str] = None, param_value = None):
        """
        Initialize validation error.
        
        Args:
            message: Error message
            param_name: Name of invalid parameter
            param_value: Invalid parameter value
        """
        self.param_name = param_name
        self.param_value = param_value
        
        detail = None
        if param_name and param_value is not None:
            detail = f"{param_name}={param_value}"
        elif param_name:
            detail = f"parameter: {param_name}"
            
        super().__init__(message, detail=detail, severity=ErrorSeverity.WARNING)
        
    def to_dict(self) -> dict:
        """
        Include parameter info in dict.
        """
        base_dict = super().to_dict()
        base_dict["param_name"] = self.param_name
        base_dict["param_value"] = str(self.param_value) if self.param_value is not None else None
        return base_dict
    
class CommunicationError(BaseError):
    """
    Raised when communication operations fail.
    
    Used for SBCP serial communication or network communication.
    """
    def __init__(self, message: str, detail: Optional[str] = None):
        """
        Initialize communication error.
        
        Args:
            message: Error message
            detail: Additional error details
        """
        super().__init__(message, detail=detail, severity=ErrorSeverity.ERROR)
        
class ParseError(BaseError):
    """
    Raised when parsing/decoding fails.
    
    Common to SBCP JSON parsing or SVBS image decoding.
    """
    def __init__(self, raw_data: str, parse_exception: Exception):
        """
        Initialize parse error.
        
        Args:
            raw_data: Raw data that failed to parse
            parse_exception: Original parsing exception
        """
        self.raw_data = raw_data
        self.parse_exception = parse_exception
        
        detail = f"{type(parse_exception).__name__}: {str(parse_exception)}"
        super().__init__("Failed to parse data", detail=detail, severity=ErrorSeverity.ERROR)
        
    def to_dict(self) -> dict:
        """
        Include parse details in dict.
        """
        base_dict = super().to_dict()
        base_dict["raw_data_preview"] = self.raw_data[:100] if len(self.raw_data) > 100 else self.raw_data
        base_dict["parse_exception"] = str(self.parse_exception)
        return base_dict
    
class TimeoutError(BaseError):
    """
    Raised when an operation times out.
    
    Common to command timeouts (SBCP) or processing timeouts (SBVS).
    """
    
    def __init__(
        self, 
        operation: str, 
        timeout_sec: float, 
        detail: Optional[str] = None
    ):
        """
        Initialize timeout error.
        
        Args:
            operation: Operation that timed out
            timeout_sec: Timeout duration in seconds
            detail: Additional timeout details
        """
        self.operation = operation
        self.timeout_sec = timeout_sec
        
        message = f"Operation '{operation}' timed out after {timeout_sec:.2f}s"
        super().__init__(message, detail=detail, severity=ErrorSeverity.WARNING)
    
    def to_dict(self) -> dict:
        """Include timeout info in dict."""
        base_dict = super().to_dict()
        base_dict["operation"] = self.operation
        base_dict["timeout_sec"] = self.timeout_sec
        return base_dict


class ResourceError(BaseError):
    """
    Raised when system resources are exhausted.
    
    Common to memory, disk, CPU, or other resource exhaustion.
    """
    
    def __init__(
        self, 
        resource_type: str, 
        detail: Optional[str] = None
    ):
        """
        Initialize resource error.
        
        Args:
            resource_type: Type of resource exhausted (memory, disk, etc.)
            detail: Additional resource details
        """
        self.resource_type = resource_type
        
        message = f"Resource exhausted: {resource_type}"
        super().__init__(message, detail=detail, severity=ErrorSeverity.CRITICAL)
    
    def to_dict(self) -> dict:
        """
        Include resource info in dict.
        """
        base_dict = super().to_dict()
        base_dict["resource_type"] = self.resource_type
        return base_dict


class ConfigurationError(BaseError):
    """
    Raised when configuration is invalid or missing.
    
    Common to system configuration issues.
    """
    
    def __init__(
        self, 
        message: str, 
        config_key: Optional[str] = None,
        detail: Optional[str] = None
    ):
        """
        Initialize configuration error.
        
        Args:
            message: Error message
            config_key: Configuration key that caused the error
            detail: Additional configuration details
        """
        self.config_key = config_key
        
        full_detail = f"config_key={config_key}" if config_key else None
        if detail:
            full_detail = f"{full_detail}, {detail}" if full_detail else detail
        
        super().__init__(message, detail=full_detail, severity=ErrorSeverity.ERROR)
    
    def to_dict(self) -> dict:
        """
        Include config info in dict.
        """
        base_dict = super().to_dict()
        base_dict["config_key"] = self.config_key
        return base_dict


class StateError(BaseError):
    """
    Raised when an operation is invalid in the current state.
    
    Common to state machine violations.
    """
    
    def __init__(
        self, 
        message: str, 
        current_state: Optional[str] = None, 
        required_state: Optional[str] = None
    ):
        """
        Initialize state error.
        
        Args:
            message: Error message
            current_state: Current state
            required_state: Required state for operation
        """
        self.current_state = current_state
        self.required_state = required_state
        
        detail_parts = []
        if current_state:
            detail_parts.append(f"current={current_state}")
        if required_state:
            detail_parts.append(f"required={required_state}")
        
        detail = ", ".join(detail_parts) if detail_parts else None
        super().__init__(message, detail=detail, severity=ErrorSeverity.ERROR)
    
    def to_dict(self) -> dict:
        """
        Include state info in dict.
        """
        base_dict = super().to_dict()
        base_dict["current_state"] = self.current_state
        base_dict["required_state"] = self.required_state
        return base_dict


class ThreadError(BaseError):
    """
    Raised when thread operations fail.
    
    Common to threading issues in sub-systems.
    """
    
    def __init__(
        self, 
        message: str, 
        thread_name: Optional[str] = None, 
        detail: Optional[str] = None
    ):
        """
        Initialize thread error.
        
        Args:
            message: Error message
            thread_name: Name of thread that failed
            detail: Additional thread details
        """
        self.thread_name = thread_name
        
        full_detail = f"thread={thread_name}" if thread_name else None
        if detail:
            full_detail = f"{full_detail}, {detail}" if full_detail else detail
        
        super().__init__(message, detail=full_detail, severity=ErrorSeverity.ERROR)
    
    def to_dict(self) -> dict:
        """
        Include thread info in dict.
        """
        base_dict = super().to_dict()
        base_dict["thread_name"] = self.thread_name
        return base_dict


class InitializationError(BaseError):
    """
    Raised when initialization fails.
    
    Common to component initialization failures.
    """
    
    def __init__(
        self, 
        component: str, 
        detail: Optional[str] = None
    ):
        """
        Initialize initialization error.
        
        Args:
            component: Component that failed to initialize
            detail: Additional initialization details
        """
        self.component = component
        
        message = f"Failed to initialize {component}"
        super().__init__(message, detail=detail, severity=ErrorSeverity.CRITICAL)
    
    def to_dict(self) -> dict:
        """
        Include component info in dict.
        """
        base_dict = super().to_dict()
        base_dict["component"] = self.component
        return base_dict


# Utility functions.
def is_recoverable_error(error: Exception) -> bool:
    """
    Check if an error is recoverable.
    
    Args:
        error: Exception to check
        
    Returns:
        True if error is likely recoverable
    """
    # Check if it's a BaseError with severity info.
    if isinstance(error, BaseError):
        return error.is_recoverable()
    
    # Timeout errors are usually recoverable.
    if isinstance(error, TimeoutError):
        return True
    
    # Communication errors might be recoverable.
    if isinstance(error, CommunicationError):
        return True
    
    # Parse errors are often transient.
    if isinstance(error, ParseError):
        return True
    
    # Default to not recoverable for unknown errors.
    return False


def get_error_severity(error: Exception) -> ErrorSeverity:
    """
    Get severity of an error.
    
    Args:
        error: Exception to check
        
    Returns:
        ErrorSeverity enum value
    """
    if isinstance(error, BaseError):
        return error.severity
    
    # Default severity for unknown errors.
    return ErrorSeverity.ERROR


def format_error_for_logging(error: Exception) -> dict:
    """
    Format error for logging.
    
    Args:
        error: Exception to format
        
    Returns:
        Dictionary representation suitable for logging
    """
    if isinstance(error, BaseError):
        return error.to_dict()
    
    # Format non-BaseError exceptions
    return {
        "type": type(error).__name__,
        "message": str(error),
        "severity": ErrorSeverity.ERROR.value
    }