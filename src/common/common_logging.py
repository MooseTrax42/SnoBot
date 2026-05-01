"""
SnoBot Common Logging System
Base logger class for SBCP and SBVS.
"""

import logging
import json
import csv
from pathlib import Path
from typing import Optional, Dict, Any, List
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
from abc import ABC, abstractmethod
from common.common_types import LogFormat, LogLevel
    
@dataclass
class LogEntry:
    """
    Base log entry structure.
    """
    timestamp: datetime
    level: str
    category: str
    message: str
    data: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert to dictionary.
        """
        return {
            "timestamp": self.timestamp.isoformat(),
            "level": self.level,
            "category": self.category,
            "message": self.message,
            "data": self.data
        }
        
    def to_json(self) -> str:
        """
        Convert to JSON string.
        """
        return json.dumps(self.to_dict())
    
    def to_csv_row(self) -> List[str]:
        """
        Convert to CSV row.
        """
        return [
            self.timestamp.isoformat(),
            self.level,
            self.category,
            self.message,
            json.dumps(self.data) if self.data else ""
        ]
        
class NullLogger:
    """
    Manages excessive logging checks by passing on all arguments,
    """
    def log_info(self, *args, **kwargs): pass
    def log_debug(self, *args, **kwargs): pass
    def log_error(self, *args, **kwargs): pass
    def log_warning(self, *args, **kwargs): pass
    def log_session_start(self, *args, **kwargs): pass
    def log_session_end(self, *args, **kwargs): pass

class BaseLogger(ABC):
    """
    Base logging system for the robot.
    
    Supports multiple output formats (text, JSON, CSV) and extensible categories.
    Subclasses should be their own category-specific logging methods.
    """
    def __init__(self, name: str, log_dir: Optional[str] = None, log_format: LogFormat = LogFormat.TEXT, enable_console: bool = True, enable_file: bool = True, level: LogLevel = LogLevel.INFO, max_file_size_mb: int = 100, rotation_count: int = 5):
        """
        Initialize base logger.
        
        Args:
            name: Logger name
            log_dir: Directory for log files (None = current directory)
            log_format: Output format (TEXT, JSON, CSV)
            enable_console: Enable console output
            enable_file: Enable file output
            level: Minimum log level
            max_file_size_mb: Max file size before rotation
            rotation_count: Number of rotated files to keep
        """
        self.name = name
        self.log_format = log_format
        self.log_dir = Path(log_dir) if log_dir else Path("logs")
        self.enable_console = enable_console
        self.enable_file = enable_file
        self.level = level
        self.max_file_size_bytes = max_file_size_mb * 1024 * 1024
        self.rotation_count = rotation_count
        
        # Create log directory.
        if self.enable_file:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            
        # Set up Python logging.
        self.logger = logging.getLogger(name)
        self.logger.setLevel(self._level_to_logging(level))
        self.logger.handlers.clear()
        
        # Console handler.
        if enable_console:
            console_handler = logging.StreamHandler()
            console_handler.setLevel(self._level_to_logging(level))
            console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            console_handler.setFormatter(console_formatter)
            self.logger.addHandler(console_handler)
            
        # File handlers.
        self.file_handles: Dict[str, Any] = {}
        if enable_file:
            self._setup_file_handlers()
            
        # Base statistics.
        self.stats = self._init_stats()
        
    @abstractmethod
    def _get_csv_categories(self) -> List[str]:
        """
        Return list of CSV category names for file creation.
        """
        pass

    @abstractmethod
    def _init_stats(self) -> Dict[str, Any]:
        """
        Initialize statistics dictionary.
        """
        return {
            "start_time": datetime.now()
        }
        
    def _level_to_logging(self, level: LogLevel) -> int:
        """
        Convert LogLevel to logging level.
        """
        mapping = {
            LogLevel.DEBUG: logging.DEBUG,
            LogLevel.INFO: logging.INFO,
            LogLevel.WARNING: logging.WARNING,
            LogLevel.ERROR: logging.ERROR,
            LogLevel.CRITICAL: logging.CRITICAL
        }
        return mapping[level]
    
    def _setup_file_handlers(self):
        """
        Set up file handlers for different categories.
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if self.log_format == LogFormat.CSV:
            # CSV files for each category.
            categories = ["commands", "responses", "telemetry", "state", "errors"]
            for category in categories:
                filename = self.log_dir / f"{self.name}_{category}_{timestamp}.csv"
                csvfile = open(filename, "w", newline="", encoding="utf-8")
                writer = csv.writer(csvfile)
                writer.writerow(["timestamp", "level", "category", "message", "data"])
                self.file_handles[category] = {"file": csvfile, "writer": writer}
        else:
            # Single log file for JSON or TEXT.
            ext = "jsonl" if self.log_format == LogFormat.JSON else "log"
            filename = self.log_dir / f"{self.name}_{timestamp}.{ext}"
            self.file_handles["main"] = open(filename, "w", encoding="utf-8")
            
    def _write_entry(self, entry: LogEntry):
        """
        Write log entry to a file.
        """
        if not self.enable_file:
            return
        
        if self.log_format == LogFormat.CSV:
            # Write to category-specific CSV.
            if entry.category in self.file_handles:
                writer = self.file_handles[entry.category]["writer"]
                writer.writerow(entry.to_csv_row())
                self.file_handles[entry.category]["file"].flush()
        
        elif self.log_format == LogFormat.JSON:
            # Write JSON line.
            handle = self.file_handles.get("main")
            if handle:
                handle.write(entry.to_json() + "\n")
                handle.flush()
                
        else:
            # Write formatted text.
            handle = self.file_handles.get("main")
            if handle:
                timestamp_str = entry.timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                data_str = f" | {json.dumps(entry.data)}" if entry.data else ""
                line = f"{timestamp_str} | {entry.level:8} | {entry.category:12} | {entry.message}{data_str}\n"
                handle.write(line)
                handle.flush()
                
    def _log(self, level: LogLevel, category: str, message: str, data: Optional[Dict] = None):
        """
        Internal logging method.
        """
        # Create entry.
        entry = LogEntry(timestamp=datetime.now(), level=level.value, category=category, message=message, data=data)
        
        # Write to file.
        self._write_entry(entry)
        
        # Log to Python logger.
        log_method = getattr(self.logger, level.value.lower())
        log_msg = f"[{category}] {message}"
        if data:
            log_msg += f" | {json.dumps(data, default=str)}"
        log_method(log_msg)
        
    def log_error(self, error_type: str, message: str, detail: Optional[Dict] = None):
        """
        Log error event.
        
        Args:
            error_type: Type of error
            message: Error message
            detail: Additional error details
        """
        self.stats["error_count"] += 1
        self._log(
            LogLevel.ERROR,
            "errors",
            f"ERROR: {error_type} - {message}",
            detail
        )
        
    def log_warning(self, category: str, message: str, data: Optional[Dict] = None):
        """
        Log warning message.
        
        Args:
            category: Warning category
            message: Warning message
            data: Additional data
        """
        self._log(LogLevel.WARNING, category, message, data)
    
    def log_info(self, category: str, message: str, data: Optional[Dict] = None):
        """
        Log info message.
        
        Args:
            category: Message category
            message: Info message
            data: Additional data
        """
        self._log(LogLevel.INFO, category, message, data)
    
    def log_debug(self, category: str, message: str, data: Optional[Dict] = None):
        """
        Log debug message.
        
        Args:
            category: Message category
            message: Debug message
            data: Additional data
        """
        self._log(LogLevel.DEBUG, category, message, data)
        
    # Session management.
    def log_session_start(self, config: Optional[Dict] = None):
        """
        Log session start.
        
        Args:
            config: Session configuration
        """
        self._log(
            LogLevel.INFO,
            "session",
            "=== SBCP Session Started ===",
            config
        )
    
    def log_session_end(self, stats: Optional[Dict] = None):
        """
        Log session end.
        
        Args:
            stats: Session statistics
        """
        session_stats = self.get_stats()
        if stats:
            session_stats.update(stats)
        
        self._log(
            LogLevel.INFO,
            "session",
            "=== SBCP Session Ended ===",
            session_stats
        )
        
    # Statistics and utilities.
    def get_stats(self) -> Dict[str, Any]:
        """
        Get logging statistics.
        
        Returns:
            Dictionary of statistics
        """
        runtime = (datetime.now() - self.stats["start_time"]).total_seconds()
        return {
            **self.stats,
            "runtime_s": round(runtime, 2)
        }
        
    def reset_stats(self):
        """
        Reset statistics counters.
        """
        self.stats = self._init_stats()
        
    def flush(self):
        """
        Flush all file handles.
        """
        for handle_data in self.file_handles.values():
            if isinstance(handle_data, dict):
                handle_data["file"].flush()
            else:
                handle_data.flush()
    
    def close(self):
        """
        Close all file handles.
        """
        for handle_data in self.file_handles.values():
            if isinstance(handle_data, dict):
                handle_data["file"].close()
            else:
                handle_data.close()
        self.file_handles.clear()
        
    def __enter__(self):
        """
        Context manager entry.
        """
        self.log_session_start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit.
        """
        self.log_session_end()
        self.close()
        
    def __repr__(self) -> str:
        """
        String representation.
        """
        return f"<{self.__class__.__name__} name={self.name} format={self.log_format.value} level={self.level.value}>"