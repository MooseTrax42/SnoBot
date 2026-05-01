"""
SnoBot Common Configuration Management
Centralized, type-safe configuration with validation and persistence.

Provides hierarchical configuration management with:
- Type safety and validation
- Environment variable overrides
- File persistence (YAML/JSON)
- Hot-reloading capability
- Default value inheritance
"""

from typing import Dict, Any, Optional, Type, TypeVar, Generic, List, Set, Callable
from dataclasses import dataclass, field, fields, asdict, is_dataclass
from pathlib import Path
from enum import Enum
import os
import yaml
import json
from threading import RLock

from common.common_types import ErrorSeverity

T = TypeVar('T')

class ConfigError(Exception):
    """
    Configuration-related errors.
    """
    pass

class ConfigValidationError(ConfigError):
    """
    Configuration validation failed.
    """
    pass

class ConfigFormat(Enum):
    """
    Configuration file formats.
    """
    YAML = "yaml"
    JSON = "json"
    ENV = "env"

@dataclass
class ConfigBase:
    """
    Base class for configuration sections.

    All configuration sections should inherit from this and use
    dataclass decorator for automatic serialization and validation.
    """

    def validate(self):
        """
        Override to add custom validation logic.

        Raises:
            ConfigValidationError: If validation fails
        """
        pass

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert configuration to dictionarty.

        Returns:
            Dictionary representation
        """
        result = {}
        for f in fields(self):
            value = getattr(self, f.name)

            if is_dataclass(value):
                result[f.name] = value.to_dict()
            elif isinstance(value, list):
                result[f.name] = [item.to_dict() if is_dataclass(item) else item for item in value]
            elif isinstance(value, dict):
                result[f.name] = {k: v.to_dict() if is_dataclass(v) else v for k, v in value.items}
            elif isinstance(value, Enum):
                result[f.name] = value.value
            else:
                result[f.name] = value
        
        return result
    
    @classmethod
    def from_dict(cls: Type[T], data: Dict[str, Any]) -> T:
        """
        Create configuration file from dictionary.

        Args:
            data: Dictionary data

        Returns:
            Configuration instance
        """
        # Filter to only valid fields.
        field_names = {f.name for f in fields(cls)}
        filtered_data = {k: v for k, v in data.items() if k in field_names}
        
        # Recursively convert nested dataclasses.
        for f in fields(cls):
            if f.name in filtered_data:
                value = filtered_data[f.name]
                
                # Handle nested dataclass.
                if is_dataclass(f.type) and isinstance(value, dict):
                    filtered_data[f.name] = f.type.from_dict(value)
                
                # Handle list of dataclasses.
                elif hasattr(f.type, '__origin__') and f.type.__origin__ is list:
                    if hasattr(f.type, '__args__') and is_dataclass(f.type.__args__[0]):
                        item_type = f.type.__args__[0]
                        filtered_data[f.name] = [
                            item_type.from_dict(item) if isinstance(item, dict) else item
                            for item in value
                        ]
                
                # Handle Enum.
                elif isinstance(f.type, type) and issubclass(f.type, Enum):
                    if not isinstance(value, f.type):
                        filtered_data[f.name] = f.type(value)
        
        instance = cls(**filtered_data)
        instance.validate()
        return instance
    
    def update_from_dict(self, data: Dict[str, Any]):
        """
        Update configuration from dictionary (partial update).
        
        Args:
            data: Dictionary with updates
        """
        for key, value in data.items():
            if hasattr(self, key):
                current_value = getattr(self, key)
                
                # Recursively update nested configs.
                if is_dataclass(current_value) and isinstance(value, dict):
                    current_value.update_from_dict(value)
                else:
                    setattr(self, key, value)
        
        self.validate()
    
    def clone(self: T) -> T:
        """
        Create a deep copy of this configuration.
        
        Returns:
            Cloned configuration
        """
        return self.from_dict(self.to_dict())
    
@dataclass
class SBCPConfig(ConfigBase):
    """
    SBCP (SnoBot Control Protocol) configuration.
    """
    
    # Connection settings
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200
    timeout_sec: float = 1.0
    max_retries: int = 3
    
    # Command settings
    default_priority: int = 5
    max_queue_size: int = 100
    command_timeout_sec: float = 0.5
    
    # Telemetry settings
    telemetry_enabled: bool = True
    telemetry_rate_hz: float = 10.0
    
    # State machine settings
    heartbeat_interval_sec: float = 1.0
    fault_timeout_sec: float = 5.0
    estop_timeout_sec: float = 0.1
    
    # Watchdog settings
    watchdog_enabled: bool = True
    watchdog_timeout_sec: float = 1.0
    
    def validate(self):
        """
        Validate SBCP configuration.
        """
        if self.baudrate <= 0:
            raise ConfigValidationError("baudrate must be positive")
        
        if self.timeout_sec <= 0:
            raise ConfigValidationError("timeout_sec must be positive")
        
        if self.telemetry_rate_hz <= 0 or self.telemetry_rate_hz > 100:
            raise ConfigValidationError("telemetry_rate_hz must be between 0 and 100")
        
        if not 0 <= self.default_priority <= 15:
            raise ConfigValidationError("default_priority must be between 0 and 15")


@dataclass
class CameraConfig(ConfigBase):
    """
    Camera configuration.
    """
    
    device_id: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    
    # Camera settings.
    auto_exposure: int = 1  # 1=manual, 3=auto
    exposure: int = 100
    auto_white_balance: int = 1
    brightness: int = 128
    contrast: int = 32
    saturation: int = 64
    
    # Buffer settings.
    buffer_size: int = 4
    
    def validate(self):
        """
        Validate camera configuration.
        """
        if self.width <= 0 or self.height <= 0:
            raise ConfigValidationError("width and height must be positive")
        
        if self.fps <= 0 or self.fps > 120:
            raise ConfigValidationError("fps must be between 1 and 120")
        
        if not 0 <= self.brightness <= 255:
            raise ConfigValidationError("brightness must be between 0 and 255")


@dataclass
class StereoCameraConfig(ConfigBase):
    """
    Stereo camera configuration.
    """
    
    left: CameraConfig = field(default_factory=lambda: CameraConfig(device_id=0))
    right: CameraConfig = field(default_factory=lambda: CameraConfig(device_id=2))
    
    # Sync settings.
    max_sync_offset_sec: float = 0.01
    calibration_samples: int = 30
    
    # Performance settings.
    delivery_timeout_sec: float = 0.1
    
    def validate(self):
        """
        Validate stereo camera configuration.
        """
        self.left.validate()
        self.right.validate()
        
        if self.left.device_id == self.right.device_id:
            raise ConfigValidationError("left and right cameras must have different device IDs")
        
        if self.max_sync_offset_sec <= 0:
            raise ConfigValidationError("max_sync_offset_sec must be positive")


@dataclass
class DetectorConfig(ConfigBase):
    """
    Object detector configuration.
    """
    
    # Model settings.
    model_path: str = "data/sbcp/yolo12n.engine"
    confidence_threshold: float = 0.25
    iou_threshold: float = 0.3
    max_detections: int = 100
    
    # Classes to detect (empty = all).
    target_classes: List[str] = field(default_factory=list)
    
    # Performance settings.
    device: str = "cuda"  # "cpu", "cuda", "mps"
    half_precision: bool = False
    
    # Tracking settings.
    tracker_enabled: bool = True
    tracker_type: str = "botsort"  # "botsort" or "bytetrack"
    
    def validate(self):
        """
        Validate detector configuration.
        """
        if not 0 <= self.confidence_threshold <= 1:
            raise ConfigValidationError("confidence_threshold must be between 0 and 1")
        
        if not 0 <= self.iou_threshold <= 1:
            raise ConfigValidationError("iou_threshold must be between 0 and 1")
        
        if self.max_detections <= 0:
            raise ConfigValidationError("max_detections must be positive")
        
        if self.device not in ["cpu", "cuda", "mps"]:
            raise ConfigValidationError("device must be 'cpu', 'cuda', or 'mps'")


@dataclass
class SBVSConfig(ConfigBase):
    """
    SBVS (SnoBot Vision System) configuration.
    """
    
    # Camera configuration.
    stereo: StereoCameraConfig = field(default_factory=StereoCameraConfig)
    
    # Detector configuration.
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    
    # Processing settings
    process_every_n_frames: int = 1
    visualize: bool = True
    save_detections: bool = False
    output_path: str = "detections"
    
    def validate(self):
        """
        Validate SBVS configuration.
        """
        self.stereo.validate()
        self.detector.validate()
        
        if self.process_every_n_frames <= 0:
            raise ConfigValidationError("process_every_n_frames must be positive")


@dataclass
class LoggingConfig(ConfigBase):
    """
    Logging configuration.
    """
    
    # Output settings.
    log_to_file: bool = True
    log_to_console: bool = True
    log_path: str = "logs"
    
    # Rotation settings.
    max_file_size_mb: int = 10
    max_files: int = 5
    
    # Verbosity.
    min_severity: str = "INFO"  # DEBUG, INFO, WARNING, ERROR
    
    # Subsystem filters.
    sbcp_enabled: bool = True
    sbvs_enabled: bool = True
    
    def validate(self):
        """
        Validate logging configuration.
        """
        valid_severities = {"DEBUG", "INFO", "WARNING", "ERROR"}
        if self.min_severity not in valid_severities:
            raise ConfigValidationError(f"min_severity must be one of {valid_severities}")
        
        if self.max_file_size_mb <= 0:
            raise ConfigValidationError("max_file_size_mb must be positive")


@dataclass
class MetricsConfig(ConfigBase):
    """
    Metrics collection configuration.
    """
    
    enabled: bool = True
    window_size: int = 100
    enable_history: bool = True
    
    # Export settings.
    export_enabled: bool = False
    export_interval_sec: float = 60.0
    export_path: str = "metrics"
    
    def validate(self):
        """
        Validate metrics configuration.
        """
        if self.window_size <= 0:
            raise ConfigValidationError("window_size must be positive")


@dataclass
class SnoBotConfig(ConfigBase):
    """
    Master configuration for entire SnoBot system.
    
    This is the root configuration that contains all subsystem configs.
    """
    
    # Subsystem configurations.
    sbcp: SBCPConfig = field(default_factory=SBCPConfig)
    sbvs: SBVSConfig = field(default_factory=SBVSConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    metrics: MetricsConfig = field(default_factory=MetricsConfig)
    
    # Global settings.
    robot_name: str = "SnoBot"
    debug_mode: bool = False
    
    def validate(self):
        """Validate entire configuration."""
        self.sbcp.validate()
        self.sbvs.validate()
        self.logging.validate()
        self.metrics.validate()

class ConfigManager:
    """
    Configuration management with persistence and hot-reloading.
    
    Provides centralized configuration access with file persistence,
    environment variable overrides, and validation.
    """
    
    def __init__(
        self,
        config_class: Type[ConfigBase] = SnoBotConfig,
        config_file: Optional[Path] = None,
        auto_load: bool = True
    ):
        """
        Initialize configuration manager.
        
        Args:
            config_class: Configuration class to manage
            config_file: Path to configuration file
            auto_load: Automatically load config on init
        """
        self.config_class = config_class
        self.config_file = Path(config_file) if config_file else None
        self._config: Optional[ConfigBase] = None
        self._lock = RLock()
        self._watchers: List[Callable[[ConfigBase], None]] = []
        
        if auto_load:
            self.load()
    
    @property
    def config(self) -> ConfigBase:
        """
        Get current configuration.
        
        Returns:
            Configuration instance
        """
        if self._config is None:
            self._config = self.config_class()
        return self._config
    
    def load(self, config_file: Optional[Path] = None) -> ConfigBase:
        """
        Load configuration from file.
        
        Args:
            config_file: Override config file path
            
        Returns:
            Loaded configuration
        """
        with self._lock:
            file_path = Path(config_file) if config_file else self.config_file
            
            if file_path and file_path.exists():
                data = self._load_file(file_path)
                self._config = self.config_class.from_dict(data)
            else:
                # Use defaults.
                self._config = self.config_class()
            
            # Apply environment variable overrides.
            self._apply_env_overrides()
            
            # Notify watchers.
            self._notify_watchers()
            
            return self._config
    
    def save(self, config_file: Optional[Path] = None, format: ConfigFormat = ConfigFormat.YAML):
        """
        Save configuration to file.
        
        Args:
            config_file: Override config file path
            format: File format
        """
        with self._lock:
            if self._config is None:
                raise ConfigError("No configuration to save")
            
            file_path = Path(config_file) if config_file else self.config_file
            
            if not file_path:
                raise ConfigError("No config file specified")
            
            # Ensure directory exists.
            file_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Convert to dict and save.
            data = self._config.to_dict()
            self._save_file(file_path, data, format)
    
    def update(self, updates: Dict[str, Any]):
        """
        Update configuration with partial updates.
        
        Args:
            updates: Dictionary of updates
        """
        with self._lock:
            if self._config is None:
                self._config = self.config_class()
            
            self._config.update_from_dict(updates)
            self._notify_watchers()
    
    def reset(self):
        """Reset configuration to defaults."""
        with self._lock:
            self._config = self.config_class()
            self._notify_watchers()
    
    def watch(self, callback: Callable[[ConfigBase], None]):
        """
        Register callback for configuration changes.
        
        Args:
            callback: Function to call when config changes
        """
        with self._lock:
            self._watchers.append(callback)
    
    def _load_file(self, file_path: Path) -> Dict[str, Any]:
        """Load configuration from file."""
        suffix = file_path.suffix.lower()
        
        with open(file_path, 'r') as f:
            if suffix in ['.yaml', '.yml']:
                return yaml.safe_load(f) or {}
            elif suffix == '.json':
                return json.load(f)
            else:
                raise ConfigError(f"Unsupported config file format: {suffix}")
    
    def _save_file(self, file_path: Path, data: Dict[str, Any], format: ConfigFormat):
        """Save configuration to file."""
        with open(file_path, 'w') as f:
            if format == ConfigFormat.YAML:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            elif format == ConfigFormat.JSON:
                json.dump(data, f, indent=2)
            else:
                raise ConfigError(f"Unsupported format: {format}")
    
    def _apply_env_overrides(self):
        """Apply environment variable overrides."""
        if self._config is None:
            return
        
        # Look for environment variables like SNOBOT_SBCP_PORT.
        prefix = "SNOBOT_"
        
        for key, value in os.environ.items():
            if not key.startswith(prefix):
                continue
            
            # Parse environment variable name.
            parts = key[len(prefix):].lower().split('_')
            
            # Navigate to nested config.
            current = self._config
            for part in parts[:-1]:
                if hasattr(current, part):
                    current = getattr(current, part)
                else:
                    break
            else:
                # Set the value.
                attr_name = parts[-1]
                if hasattr(current, attr_name):
                    field_type = type(getattr(current, attr_name))
                    
                    # Convert string to appropriate type.
                    try:
                        if field_type == bool:
                            converted = value.lower() in ['true', '1', 'yes']
                        elif field_type in [int, float, str]:
                            converted = field_type(value)
                        else:
                            converted = value
                        
                        setattr(current, attr_name, converted)
                    except (ValueError, TypeError):
                        pass  # Skip invalid conversions.
    
    def _notify_watchers(self):
        """
        Notify all watchers of configuration change.
        """
        if self._config is None:
            return
        
        for callback in self._watchers:
            try:
                callback(self._config)
            except Exception as e:
                # Don't let watcher exceptions break the manager.
                print(f"Config watcher error: {e}")

# Global configuration instance.
_global_config_manager: Optional[ConfigManager] = None
_config_lock = RLock()


def get_config_manager(
    config_file: Optional[Path] = None,
    reset: bool = False
) -> ConfigManager:
    """
    Get or create global configuration manager.
    
    Args:
        config_file: Path to config file (only used on first call)
        reset: Force recreate the manager
        
    Returns:
        Global ConfigManager instance
    """
    global _global_config_manager
    
    with _config_lock:
        if _global_config_manager is None or reset:
            _global_config_manager = ConfigManager(
                config_class=SnoBotConfig,
                config_file=config_file,
                auto_load=True
            )
        
        return _global_config_manager


def get_config() -> SnoBotConfig:
    """
    Get current global configuration.
    
    Returns:
        Current SnoRobotConfig instance
    """
    return get_config_manager().config