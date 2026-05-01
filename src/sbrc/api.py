"""
Controller API — transport-agnostic command dispatch.

Maps flat JSON commands from the ESP32 controller to ControlLoop
and PerimeterRecorder calls.  Pure dict-in → dict-out: no I/O,
no sockets, no threads.

Protocol summary (ESP32 → Jetson, one JSON object per line):

  Motion
    {"cmd":"vel",  "v":0.5, "w":0.0}        set velocity (m/s, rad/s)
    {"cmd":"stop"}                           graceful stop (ramps to zero)
    {"cmd":"estop"}                          emergency stop (immediate)
    {"cmd":"resume"}                         resume from STOPPED

  Mode
    {"cmd":"mode", "mode":"MANUAL"}          set mode (IDLE / MANUAL / AUTO)

  Actuators
    {"cmd":"auger", "on":true}
    {"cmd":"salt",  "on":true}
    {"cmd":"chute", "angle":45.0}            degrees, clamped [-90, 90]

  Lights
    {"cmd":"light", "on":true}               headlight
    {"cmd":"slight","state":"BLINK"}         status light (ON / OFF / BLINK)

  Faults
    {"cmd":"reset_faults"}                   reset all (or "codes":[70])

  Perimeter recording
    {"cmd":"rec_start"}                     # optional "reset": false
    {"cmd":"rec_stop"}
    {"cmd":"rec_reset"}
    {"cmd":"rec_stats"}                      returns waypoint count, distance, state

  System
    {"cmd":"ping"}                           connectivity check
    {"cmd":"status"}                         returns full status snapshot
    {"cmd":"hb"}                             heartbeat (no response)

All commands may include an optional "seq" integer.  If present it is
echoed in the response so the ESP32 can correlate request/response.

Responses (Jetson → ESP32):
    {"ack":"vel","ok":true}
    {"ack":"vel","ok":false,"err":"motion not enabled"}

Telemetry push (Jetson → ESP32, ~5 Hz):
    {"t":1,"st":"MANUAL","v":[0.15,0.0],"vt":[0.4,0.0],"bat":46.5,
     "f":[],"e":0,"aug":0,"sal":0,"ch":90,"mot":1,
     "odo":[1.2,-0.4,1.57],"rec":"idle","rwp":0}
"""

import time
from typing import Dict, Any, Optional, Callable

from sbcp.control_loop import ControlLoop
from sbcp.schema import parse_envelope


class ControllerAPI:
    """
    Transport-agnostic command dispatch for remote control.

    Wraps ControlLoop + PerimeterRecorder into a flat command table.
    Every public entry point is synchronous and non-blocking.
    """

    def __init__(self, control_loop: ControlLoop, recorder=None):
        self._ctrl = control_loop
        self._rec = recorder  # Optional PerimeterRecorder

        # Command table: cmd string → handler method.
        self._handlers: Dict[str, Callable] = {
            "vel":          self._h_vel,
            "stop":         self._h_stop,
            "estop":        self._h_estop,
            "resume":       self._h_resume,
            "mode":         self._h_mode,
            "auger":        self._h_auger,
            "salt":         self._h_salt,
            "chute":        self._h_chute,
            "light":        self._h_light,
            "slight":       self._h_slight,
            "reset_faults": self._h_reset_faults,
            "status":       self._h_status,
            "rec_start":    self._h_rec_start,
            "rec_stop":     self._h_rec_stop,
            "rec_reset":    self._h_rec_reset,
            "rec_stats":    self._h_rec_stats,
            "ping":         self._h_ping,
            "hb":           self._h_hb,
        }

    def set_recorder(self, recorder) -> None:
        """Attach or replace the perimeter recorder."""
        self._rec = recorder

    # ------------------------------------------------------------------
    # Public entry points
    # ------------------------------------------------------------------

    def dispatch(self, msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Dispatch a command message.

        Args:
            msg: Parsed JSON dict.  Must contain a ``"cmd"`` key.

        Returns:
            Response dict, or None for fire-and-forget commands (``hb``).
        """
        cmd = msg.get("cmd")
        if not cmd:
            return self._err(msg, "missing 'cmd' field")

        handler = self._handlers.get(cmd)
        if handler is None:
            return self._err(msg, f"unknown command: {cmd}")

        try:
            return handler(msg)
        except Exception as e:
            return self._err(msg, str(e))

    def build_telemetry(self) -> Dict[str, Any]:
        """
        Build a compact telemetry snapshot for push to the controller.
        Designed to stay under ~200 bytes when serialised.
        """
        ctrl = self._ctrl
        sm = ctrl.state_machine
        v_cur = ctrl.vel_ramp.get_current()
        v_tgt = ctrl.vel_ramp.get_target()

        telem: Dict[str, Any] = {
            "t":   1,
            "ts":  int(time.time() * 1000) & 0xFFFFFFFF,
            "st":  sm.state.value,
            "v":   [round(v_cur[0], 2), round(v_cur[1], 2)],
            "vt":  [round(v_tgt[0], 2), round(v_tgt[1], 2)],
            "bat": self._extract_battery(),
            "f":   list(sm.active_faults),
            "e":   1 if sm.estop_active else 0,
            "mot": 1 if sm.is_motion_enabled() else 0,
        }

        # Actuator state from the live intent.
        intent = ctrl.intent_gen._intent
        telem["aug"] = 1 if intent.auger_enabled else 0
        telem["sal"] = 1 if intent.salt_enabled else 0
        telem["ch"]  = round(intent.chute_angle, 1) if intent.chute_angle is not None else 0

        # Odometry.
        pose = ctrl.get_pose()
        telem["odo"] = (
            [round(pose.x, 2), round(pose.y, 2), round(pose.theta, 2)]
            if pose else None
        )

        # Perimeter recorder.
        if self._rec is not None:
            telem["rec"] = self._rec.get_state().value
            telem["rwp"] = self._rec.get_waypoint_count()
        else:
            telem["rec"] = "n/a"
            telem["rwp"] = 0

        return telem

    # ------------------------------------------------------------------
    # Response helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _ok(msg: Dict, data: Optional[Dict] = None) -> Dict[str, Any]:
        resp: Dict[str, Any] = {"ack": msg["cmd"], "ok": True}
        seq = msg.get("seq")
        if seq is not None:
            resp["seq"] = seq
        if data is not None:
            resp["data"] = data
        return resp

    @staticmethod
    def _err(msg: Dict, err: str) -> Dict[str, Any]:
        resp: Dict[str, Any] = {"ack": msg.get("cmd", "?"), "ok": False, "err": err}
        seq = msg.get("seq")
        if seq is not None:
            resp["seq"] = seq
        return resp

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------

    def _h_vel(self, msg):
        v = msg.get("v")
        w = msg.get("w", 0.0)
        if v is None:
            return self._err(msg, "'v' is required")
        if not self._ctrl.is_motion_enabled():
            return self._err(msg, "motion not enabled")
        self._ctrl.set_velocity(float(v), float(w))
        return self._ok(msg)

    def _h_stop(self, msg):
        self._ctrl.stop_motion()
        return self._ok(msg)

    def _h_estop(self, msg):
        self._ctrl.estop()
        return self._ok(msg)

    def _h_resume(self, msg):
        self._ctrl.resume()
        return self._ok(msg)

    def _h_mode(self, msg):
        mode = msg.get("mode")
        if mode is None:
            return self._err(msg, "'mode' is required")
        if mode not in ("IDLE", "MANUAL", "AUTO"):
            return self._err(msg, f"invalid mode: {mode}")
        self._ctrl.set_mode(mode)
        return self._ok(msg)

    def _h_auger(self, msg):
        on = msg.get("on")
        if on is None:
            return self._err(msg, "'on' is required")
        self._ctrl.set_auger(bool(on))
        return self._ok(msg)

    def _h_salt(self, msg):
        on = msg.get("on")
        if on is None:
            return self._err(msg, "'on' is required")
        self._ctrl.set_salt(bool(on))
        return self._ok(msg)

    def _h_chute(self, msg):
        angle = msg.get("angle")
        if angle is None:
            return self._err(msg, "'angle' is required")
        angle = max(-90.0, min(90.0, float(angle)))
        self._ctrl.set_chute(angle)
        return self._ok(msg)

    def _h_light(self, msg):
        on = msg.get("on")
        if on is None:
            return self._err(msg, "'on' is required")
        self._ctrl.set_headlight(bool(on))
        return self._ok(msg)

    def _h_slight(self, msg):
        state = msg.get("state")
        if state is None:
            return self._err(msg, "'state' is required")
        if state not in ("ON", "OFF", "BLINK"):
            return self._err(msg, f"invalid state: {state}")
        self._ctrl.set_status_light(state)
        return self._ok(msg)

    def _h_reset_faults(self, msg):
        codes = msg.get("codes")
        self._ctrl.reset_faults(codes)
        return self._ok(msg)

    def _h_status(self, msg):
        # Keep STATUS ACK compact and strictly JSON-safe for small embedded
        # parsers; rich state is continuously available via telemetry push.
        sm = self._ctrl.state_machine
        data = {
            "st": sm.state.value,
            "mot": 1 if sm.is_motion_enabled() else 0,
            "e": 1 if sm.estop_active else 0,
            "f": list(sm.active_faults),
        }
        if self._rec is not None:
            data["rec"] = self._rec.get_state().value
            data["rwp"] = self._rec.get_waypoint_count()
        return self._ok(msg, data=data)

    # -- Perimeter recording --

    def _h_rec_start(self, msg):
        if self._rec is None:
            return self._err(msg, "recorder not available")
        reset = msg.get("reset", True)
        if not self._rec.start_recording(reset_odometry=bool(reset)):
            return self._err(msg, f"cannot start in state {self._rec.get_state().value}")
        return self._ok(msg)

    def _h_rec_stop(self, msg):
        if self._rec is None:
            return self._err(msg, "recorder not available")
        if not self._rec.stop_recording():
            return self._err(msg, f"cannot stop in state {self._rec.get_state().value}")
        return self._ok(msg)

    def _h_rec_reset(self, msg):
        if self._rec is None:
            return self._err(msg, "recorder not available")
        if not self._rec.reset():
            return self._err(msg, f"cannot reset in state {self._rec.get_state().value}")
        return self._ok(msg)

    def _h_rec_stats(self, msg):
        if self._rec is None:
            return self._err(msg, "recorder not available")
        return self._ok(msg, data=self._rec.get_stats())

    # -- System --

    def _h_ping(self, msg):
        return self._ok(msg)

    def _h_hb(self, _msg):
        # Heartbeat — no response, just resets the watchdog timer
        # in the server layer.
        return None

    # ------------------------------------------------------------------
    # Telemetry helpers
    # ------------------------------------------------------------------

    def _extract_battery(self) -> Optional[float]:
        """Pull battery voltage from latest telemetry."""
        raw = self._ctrl.subscriber.get_latest()
        if not raw:
            return None
        try:
            _, payload, _ = parse_envelope(raw)
            b = payload.get("b")
            if b is not None:
                return round(b / 100.0, 2)
            bv = payload.get("battery_v")
            if bv is not None:
                return round(float(bv), 2)
        except (ValueError, KeyError):
            pass
        return None
