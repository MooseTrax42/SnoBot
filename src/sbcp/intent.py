"""
SBCP (SnoBot Communication Protocol) v0.3.0
Intent-based control system.

This is a continually-mutated desired state to be periodically published as a snapshot to the Arduino.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple, List
import time
from sbcp.schema import make_envelope

@dataclass
class Intent:
    """
    Represents desired machine behavior.

    Intents are rate-independent as they represent the current desired state.
    All control goes through intent.
    """
    motion_ts: float = 0.0
    actuator_ts: float = 0.0
    light_ts: float = 0.0
    mode_ts: float = 0.0
    control_ts: float = 0.0 # For stop/resume/reset.

    # Motion intent.
    velocity: Optional[Tuple[float, float]] = None # (v, w)

    # Actuator intent.
    auger_enabled: Optional[bool] = None
    salt_enabled: Optional[bool] = None
    chute_angle: Optional[float] = None

    # Light intent.
    headlight: Optional[bool] = None
    status_light: Optional[str] = None # ON, OFF, BLINK.

    # Mode intent.
    target_mode: Optional[str] = None

    # Control intent.
    request_stop: Optional[bool] = None
    request_resume: Optional[bool] = None
    reset_faults: Optional[List[int]] = None
    request_estop: Optional[bool] = None

    def motion_age(self) -> float:
        """Get age of motion intent in seconds."""
        return time.time() - self.motion_ts
    
    def actuator_age(self) -> float:
        """Get age of actuator timestamp in seconds."""
        return time.time() - self.actuator_ts
    
    def light_age(self) -> float:
        """Get age of light timestamp in seconds."""
        return time.time() - self.light_ts
    
    def mode_age(self) -> float:
        """Get age of motion timestamp in seconds."""
        return time.time() - self.mode_ts
    
    def control_age(self) -> float:
        """Get age of control timestamp in seconds."""
        return time.time() - self.control_ts
    
    def to_command_dict(self) -> Dict[str, Any]:
        """
        Convert intent to command dictionary for Arduino. Only includes non-None fields.
        """
        cmd = {
            "ts": int(time.time() * 1000) # ms, represents packet generation time.
        }

        # Push additional information to the command packet.

        # Motion.
        if self.velocity is not None:
            cmd["v"] = self.velocity[0]
            cmd["w"] = self.velocity[1]

        # Actuators.
        if self.auger_enabled is not None:
            cmd["auger_en"] = self.auger_enabled

        if self.salt_enabled is not None:
            cmd["salt_en"] = self.salt_enabled

        if self.chute_angle is not None:
            cmd["chute_angle"] = self.chute_angle

        # Lights.
        if self.headlight is not None:
            cmd["headlight"] = self.headlight

        if self.status_light is not None:
            cmd["status_light"] = self.status_light

        # Mode and control.
        if self.target_mode is not None:
            cmd["mode"] = self.target_mode
            print(f"[Intent] Adding mode to command: {self.target_mode}")

        if self.request_stop is not None and self.request_stop:
            cmd["stop"] = True

        if self.request_resume is not None and self.request_resume:
            cmd["resume"] = True

        if self.reset_faults is not None:
            if len(self.reset_faults) == 0:
                cmd["reset_faults"] = "all"
            else:
                cmd["reset_faults"] = self.reset_faults
        if self.request_estop is not None and self.request_estop:
            cmd["estop"] = True

        return cmd

    def to_message(self, seq: Optional[int] = None) -> Dict[str, Any]:
        """Return intent as an enveloped message."""
        payload = self.to_command_dict()
        return make_envelope("INTENT", payload, seq=seq, ts_ms=payload.get("ts"))
    
class IntentGenerator:
    """
    Generates and manages robot intents.

    Maintains current desired state and produces intents for publishing.
    All operational control flows through here.
    """
    def __init__(self):
        # Current intents based on last desired state.
        self._intent = Intent()

        # Intent aging thresholds.
        self._motion_max_age = 0.25  # Motion controls are critical.
        self._actuator_max_age = 1.0 # Actuators are less critical than motors.
        self._light_max_age = 3.0    # Lights are not critical.
        self._mode_max_age = 1.0     # Should be acknowledged relatively quickly.
        self._control_max_age = 0.25 # Control commands are critical.

    # Motion intent.
    def set_motion(self, v: float, w: float):
        self._intent.velocity = (v, w)
        self._intent.motion_ts = time.time()

    def clear_motion(self):
        self._intent.velocity = None
        self._intent.motion_ts = time.time()

    # Actuator intent.
    def set_auger(self, enabled: bool):
        self._intent.auger_enabled = enabled
        self._intent.actuator_ts = time.time()

    def set_salt(self, enabled: bool):
        self._intent.salt_enabled = enabled
        self._intent.actuator_ts = time.time()

    def set_chute(self, angle: float):
        self._intent.chute_angle = angle
        self._intent.actuator_ts = time.time()

    def clear_actuators(self):
        self._intent.auger_enabled = None
        self._intent.salt_enabled = None
        self._intent.chute_angle = None
        self._intent.actuator_ts = time.time()

    # Light intent.
    def set_headlight(self, enabled: bool):
        self._intent.headlight = enabled
        self._intent.light_ts = time.time()

    def set_status_light(self, status: str):
        self._intent.status_light = status
        self._intent.light_ts = time.time()

    def clear_lights(self):
        self._intent.headlight = None
        self._intent.status_light = None
        self._intent.light_ts = time.time()

    # Mode intent.
    def set_mode(self, mode: str):
        self._intent.target_mode = mode
        self._intent.mode_ts = time.time()

    def clear_mode(self):
        self._intent.target_mode = None
        self._intent.mode_ts = time.time()

    # Control intent.
    def request_stop(self):
        self._intent.request_stop = True
        self._intent.control_ts = time.time()
        
        # Clear motion and actuators for safety.
        self.clear_motion()
        self.clear_actuators()

    def request_resume(self):
        self._intent.request_resume = True
        self._intent.control_ts = time.time()

    def request_reset_faults(self, fault_codes: Optional[List[int]] = None):
        self._intent.reset_faults = fault_codes if fault_codes else []
        self._intent.control_ts = time.time()

    def request_estop(self):
        self._intent.request_estop = True
        self._intent.control_ts = time.time()
        # Clear motion and actuators for safety.
        self.clear_motion()
        self.clear_actuators()

    def clear_control(self):
        self._intent.request_stop = None
        self._intent.request_resume = None
        self._intent.reset_faults = None
        self._intent.request_estop = None
        self._intent.control_ts = time.time()

    # Intent management.
    def get_fresh_intent(self) -> Optional[Intent]:
        """
        Returns intent if at least one field is within its max age.
        Automatically expires old intents for safety.
        """
        now = time.time()

        # Check age thresholds and expire motion.
        if self._intent.velocity is not None and now - self._intent.motion_ts > self._motion_max_age:
            self._intent.velocity = None

        # Same for actuators.
        if ((
            self._intent.auger_enabled is not None 
            or self._intent.salt_enabled is not None 
            or self._intent.chute_angle is not None) 
            and now - self._intent.actuator_ts > self._actuator_max_age):
            self._intent.auger_enabled = None
            self._intent.salt_enabled = None
            self._intent.chute_angle = None

        # As well as lights.
        if (self._intent.headlight is not None or self._intent.status_light is not None) and now - self._intent.light_ts > self._light_max_age:
            self._intent.headlight = None
            self._intent.status_light = None

        # ...and mode.
        if self._intent.target_mode is not None and now - self._intent.mode_ts > self._mode_max_age:
            self._intent.target_mode = None

        # And controls.
        if ((self._intent.request_stop is not None 
            or self._intent.request_resume is not None 
            or self._intent.reset_faults is not None)
            and now - self._intent.control_ts > self._control_max_age):
            self._intent.request_stop = None
            self._intent.request_resume = None
            self._intent.reset_faults = None

        # If nothing left to send, return None.
        fields = [
            self._intent.velocity, 
            self._intent.auger_enabled, 
            self._intent.salt_enabled, 
            self._intent.chute_angle, 
            self._intent.headlight, 
            self._intent.status_light, 
            self._intent.target_mode,
            self._intent.request_stop,
            self._intent.request_resume,
            self._intent.reset_faults,
            self._intent.request_estop
        ]
        if not any(f is not None for f in fields):
            return None

        return self._intent
    
    def get_intent_dict(self) -> Dict[str, Any]:
        """Get fresh intent as dictionary for publishing."""
        intent = self.get_fresh_intent()
        if intent:
            return intent.to_command_dict()
        return {}

    def clear_all(self):
        self._intent = Intent()
