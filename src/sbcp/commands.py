"""
SBCP (SnoBot Commuication Protocol) v0.3.0
Simplified command implementation.
"""

from abc import ABC
from enum import Enum
from typing import Dict, Any, List
from sbcp.types import RobotState
from sbcp.schema import make_envelope

class RobotMode(Enum):
    """
    Operating modes for MODE command.
    """
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    AUTO = "AUTO"

class LightTarget(Enum):
    """
    Light control targets.
    """
    HEAD = "HEAD"
    STATUS = "STATUS"

class LightState(Enum):
    """
    Light states.
    """
    ON = "ON"
    OFF = "OFF"
    BLINK = "BLINK"

class Command(ABC):
    NAME: str
    REQUIRED_ARGS: List[str] = []
    OPTIONAL_ARGS: List[str] = []
    ALLOWED_STATES: List[RobotState] = []
    REQUIRE_ACK: bool

    def __init__(self, **kwargs):
        self.args = kwargs
        self.validate()

    def validate(self):
        for arg in self.REQUIRED_ARGS:
            if arg not in self.args:
                raise ValueError(f"{self.NAME}: missing argument '{arg}'")
    
    def to_message(self, seq: int) -> Dict[str, Any]:
        payload = {
            "cmd": self.NAME,
            "args": self.args,
        }
        return make_envelope("CMD", payload, seq=seq)
    
class Hello(Command):
    NAME = "HELLO"
    REQUIRED_ARGS = ["version"]
    ALLOWED_STATES = []
    REQUIRE_ACK = True

    def validate(self):
        super().validate()
        if not isinstance(self.args["version"], str):
            raise ValueError("HELLO, 'version' must be string")

class Ping(Command):
    NAME = "PING"
    REQUIRED_ARGS = []
    ALLOWED_STATES = []
    REQUIRE_ACK = False

class Status(Command):
    NAME = "STATUS"
    REQUIRED_ARGS = []
    ALLOWED_STATES = []
    REQUIRE_ACK = True

class Mode(Command):
    NAME = "MODE"
    REQUIRED_ARGS = ["mode"]
    ALLOWED_STATES = [RobotState.IDLE, RobotState.MANUAL, RobotState.AUTO, RobotState.STOPPED]
    REQUIRE_ACK = True

    def validate(self):
        super().validate()
        mode = self.args["mode"]

        # Accept string or RobotMode enum.
        if isinstance(mode, RobotMode):
            self.args["mode"] = mode.value
        elif isinstance(mode, str):
            # Validate string if valid mode.
            try:
                RobotMode[mode.upper()]
            except KeyError:
                raise ValueError(f"MODE: invalid mode '{mode}'")
            
class Stop(Command):
    NAME = "STOP"
    REQUIRED_ARGS = []
    ALLOWED_STATES = [RobotState.MANUAL, RobotState.AUTO, RobotState.DEGRADED_COMM]
    REQUIRE_ACK = True

class Resume(Command):
    NAME = "RESUME"
    REQUIRED_ARGS = []
    ALLOWED_STATES = [RobotState.STOPPED]
    REQUIRE_ACK = True

class SetVelocity(Command):
    NAME = "SET_VELOCITY"
    REQUIRED_ARGS = ["v", "w"]
    ALLOWED_STATES = [RobotState.MANUAL, RobotState.AUTO]
    REQUIRE_ACK = False

    MAX_V = 1.5
    MAX_W = 3.14

    def validate(self):
        super().validate()
        v = self.args["v"]
        w = self.args["w"]

        # Clamp value.
        self.args["v"] = max(-self.MAX_V, min(self.MAX_V, v))
        self.args["w"] = max(-self.MAX_W, min(self.MAX_W, w))

class SetAuger(Command):
    NAME = "SET_AUGER"
    REQUIRED_ARGS = ["enabled"]
    ALLOWED_STATES = [RobotState.MANUAL, RobotState.AUTO]
    REQUIRE_ACK = False

    def validate(self):
        super().validate()

        enabled = self.args["enabled"]

        if not isinstance(enabled, bool):
            raise ValueError("SET_AUGER: 'enabled' must be bool")
        
        self.args["enabled"] = enabled

class SetSalt(Command):
    NAME = "SET_SALT"
    REQUIRED_ARGS = ["enabled"]
    ALLOWED_STATES = [RobotState.MANUAL, RobotState.AUTO]
    REQUIRE_ACK = False

    def validate(self):
        super().validate()

        enabled = self.args["enabled"]

        if not isinstance(enabled, bool):
            raise ValueError("SET_AUGER: 'enabled' must be bool")
        
        self.args["enabled"] = enabled

class SetChute(Command):
    NAME = "SET_CHUTE"
    REQUIRED_ARGS = ["angle"]
    ALLOWED_STATES = [RobotState.MANUAL, RobotState.AUTO]
    REQUIRE_ACK = False

    MAX_ANGLE = 90
    MIN_ANGLE = -90

    def validate(self):
        super().validate()
        angle = self.args["angle"]

        if not isinstance(angle ,(int, float)):
            raise ValueError("SET_CHUTE: 'angle' must be numeric")
        
        # Clamp to valid range
        self.args["angle"] = max(self.MIN_ANGLE, min(self.MAX_ANGLE, float(angle)))

class SetLight(Command):
    NAME = "SET_LIGHT"
    REQUIRED_ARGS = ["which", "state"]
    OPTIONAL_ARGS = ["blink_hz"]
    ALLOWED_STATES = []
    REQUIRE_ACK = False

    MAX_BLINK_HZ = 10.0
    MIN_BLINK_HZ = 0.5

    def validate(self):
        super().validate()

        which = self.args["which"]
        state = self.args["state"]

        # Validate 'which'.
        if isinstance(which, LightTarget):
            self.args["which"] = which.value
        elif isinstance(which, str):
            try:
                LightTarget[which.upper()]
                self.args["which"] = which.upper()
            except KeyError:
                raise ValueError(f"SET_LIGHT: Invalid target '{which}'")
        else:
            raise ValueError("SET_LIGHT: 'which' must be a string or LightTarget enum")
        
        # Validate 'state'.
        if isinstance(state, LightState):
            self.args["state"] = state.value
        elif isinstance(state, str):
            try:
                LightState[state.upper()]
                self.args["state"] = state.upper()
            except KeyError:
                raise ValueError(f"SET_LIGHT: Invalid state '{state}'")
        else:
            raise ValueError("SET_LIGHT: 'state' must be a string or LightState enum")
        
        # Validate 'blink_hz' if state is BLINK.
        if self.args["state"] == "BLINK":
            if "blink_hz" not in self.args:
                raise ValueError("SET_LIGHT: 'blink_hz' required when state='BLINK'")
            
            blink_hz = self.args["blink_hz"]
            if not isinstance(blink_hz, (int, float)):
                raise ValueError("SET_LIGHT: 'blink_hz' must be numeric")
            
            self.args["blink_hz"] = max(self.MIN_BLINK_HZ, min(self.MAX_BLINK_HZ, blink_hz))

class ResetFaults(Command):
    NAME = "RESET_FAULTS"
    REQUIRED_ARGS = []
    OPTIONAL_ARGS = ["specified"]
    ALLOWED_STATES = [RobotState.IDLE, RobotState.MANUAL, RobotState.AUTO, RobotState.STOPPED, RobotState.FAULT, RobotState.ESTOPPED]
    REQUIRE_ACK = True

    def validate(self):
        super().validate()

        if "specified" in self.args:
            specified = self.args["specified"]
            if not isinstance(specified, list):
                raise ValueError("RESET_FAULTS: 'specified' must be a list of int")
            
            # Validate all elements are integers.
            if not all(isinstance(code, int) for code in specified):
                raise ValueError("RESET_FAUTLS: all fault codes must be int")

class Shutdown(Command):
    NAME = "SHUTDOWN"
    REQUIRED_ARGS = []
    OPTIONAL_ARGS = []
    ALLOWED_STATES = []
    REQUIRE_ACK = True

COMMAND_REGISTRY: Dict[str, type[Command]] = {
    "HELLO": Hello,
    "PING": Ping,
    "STATUS": Status,
    "MODE": Mode,
    "STOP": Stop, 
    "RESUME": Resume,
    "SET_VELOCITY": SetVelocity,
    "SET_AUGER": SetAuger,
    "SET_SALT": SetSalt,
    "SET_LIGHT": SetLight,
    "RESET_FAULTS": ResetFaults,
    "SHUTDOWN": Shutdown
}    

def create_command(name: str, **kwargs) -> Command:
    """
    Factory function to create command instances.

    Args:
        name: Command name (e.g., "SET_VELOCITY")
        **kwargs: Command arguments

    Returns:
        Command instance

    Raises:
        ValueError: If command name is unknown
    """
    name_upper = name.upper()

    if name_upper not in COMMAND_REGISTRY:
        raise ValueError(f"Unknown command: {name}")
    
    command_class = COMMAND_REGISTRY[name_upper]
    return command_class(**kwargs)
