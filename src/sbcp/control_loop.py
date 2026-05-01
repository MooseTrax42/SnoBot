"""
SBCP (SnoBot Communication Protocol) v0.3.0
Non-blocking control loop with async/await capabililties, as well as smooth velocity management.
"""

import asyncio
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple
from sbcp.intent import IntentGenerator
from sbcp.state_machine import StateMachine, RobotState, RobotMode
from sbcp.commands import *
from sbcp.transport import AsyncSerialTransport
from sbcp.async_modules import AsyncCommManager, AsyncPublisher, AsyncSubscriber
from sbcp.types import PROTOCOL_VERSION, robot_state_from_id
from sbcp.schema import make_envelope, parse_envelope
from sbcp.errors import COMM_TIMEOUT_CODE, ACK_TIMEOUT_CODE
from sban.localization.odometry import Pose, OdometryEstimator

@dataclass
class VelocityLimits:
    """Physical limits for velocity ramping."""
    max_linear_vel: float = 1.5      # m/s
    max_angular_vel: float = 3.14    # rad/s
    max_linear_accel: float = 2.0    # m/s2
    max_angular_accel: float = 4.0   # rad/s2

class VelocityRamper:
    """
    Smoothly ramps velocity commands to avoid jerky motion.

    Implements acceleration limiting between target and current velocity. 
    Call update() at control rate to get teh ramped velocity.
    """
    def __init__(self, limits: Optional[VelocityLimits] = None, rate_hz: float = 20.0):
        self.limits = limits or VelocityLimits()
        self.dt = 1 / rate_hz # Time step.

        # Current state.
        self._current_v: float = 0.0
        self._current_w: float = 0.0

        # Target state.
        self._target_v: float = 0.0
        self._target_w: float = 0.0

        # Timing.
        self._last_update_time = asyncio.get_event_loop().time()

    def set_target(self, v: float, w: float):
        self._target_v = self._clamp(v, -self.limits.max_linear_vel, self.limits.max_linear_vel)
        self._target_w = self._clamp(w, -self.limits.max_angular_vel, self.limits.max_angular_vel)

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range."""
        return max(min_val, min(max_val, value))
    
    def get_current(self) -> Tuple[float, float]:
        return (self._current_v, self._current_w)
    
    def get_target(self) -> Tuple[float, float]:
        return (self._target_v, self._target_w)
    
    @staticmethod
    def _ramp_velocity(current: float, target: float, max_accel: float, dt: float) -> float:
        """Ramp velocity toward target with acceleration limit."""
        # Calculate error.
        error = target - current
        
        # Calculate maximum change this timestep.
        max_delta = max_accel * dt
        
        # Limit change to max acceleration.
        if abs(error) <= max_delta:
            # Within one step, just go to target.
            return target
        else:
            # Ramp at max acceleration.
            direction = 1.0 if error > 0 else -1.0
            return current + direction * max_delta
    
    def update(self) -> Tuple[float, float]:
        # Calculate actual dt in case update rate varies.
        now = asyncio.get_event_loop().time()

        if self._last_update_time is None:
            self._last_update_time = now
            return (self._current_v, self._current_w)

        dt = max(now - self._last_update_time, 1e-4)
        self._last_update_time = now

        # Ramp linear.
        self._current_v = self._ramp_velocity(self._current_v, self._target_v, self.limits.max_linear_accel, dt)
        self._current_w = self._ramp_velocity(self._current_w, self._target_w, self.limits.max_angular_accel, dt)

        return (self._current_v, self._current_w)
    
    def reset(self):
        """Reset to zero velocity (emergency stop)."""
        self._current_v = 0.0
        self._current_w = 0.0
        self._target_v = 0.0
        self._target_w = 0.0

    def is_at_target(self, tolerance: float = 0.01) -> bool:
        """Check if current velocity is at target."""
        v_diff = abs(self._current_v - self._target_v)
        w_diff = abs(self._current_w - self._target_w)
        return v_diff < tolerance and w_diff < tolerance
    
    def is_stopped(self, tolerance: float = 0.01) -> bool:
        return abs(self._current_v) < tolerance and abs(self._current_w) < tolerance

class ControlLoop:
    """
    Main robot controller.

    Combines:
    - AsyncPublisher: Sends intents at 20Hz
    - AsycSubscriber: Receives telemetry at 50Hz
    - IntentGenerator: Maintains desired state
    - StateMachine: Tracks robot state
    - Command handling: For ACK-required commands.
    """
    def __init__(self,
        transport: AsyncSerialTransport,
        publish_rate_hz: float = 15.0,
        subscribe_rate_hz: float = 20.0,
        ack_timeout_s: float = 0.5,
        auto_reconnect: bool = True,
        diagnostic_reconnect: bool = True,
        min_send_interval_ms: float = 5.0,
        enable_odometry: bool = True,
        odometry_config_path: str = "data/sban/robot_params.yaml",
        telemetry_poll_rate_hz: float = 20.0,
    ):
        self.transport = transport
        self._auto_reconnect = auto_reconnect
        self._diagnostic_reconnect_enabled = diagnostic_reconnect
        
        # Core components.
        self.intent_gen = IntentGenerator()
        self.state_machine = StateMachine(auto_clear_non_latched=True)
        self.vel_ramp = VelocityRamper(rate_hz=publish_rate_hz)

        # Adding odometry instance.
        self.odometry: Optional[OdometryEstimator] = None
        if enable_odometry:
            try:
                self.odometry = OdometryEstimator(odometry_config_path)
                print(f"[ControlLoop] Odometry enabled (meters_per_tick={self.odometry.meters_per_tick:.6f})")
            except FileNotFoundError:
                print(f"[ControlLoop] Warning: Config not found at {odometry_config_path}")
                print("[ControlLoop] Odometry disabled")
            except Exception as e:
                print(f"[ControlLoop] Odometry init failed: {e}")

        # Async components.
        self.publisher = AsyncPublisher(transport, publish_rate_hz)
        self.subscriber = AsyncSubscriber(transport, subscribe_rate_hz)
        self.comm_manager = AsyncCommManager(min_send_interval_ms)
        self.comm_manager.wrap_transport(transport)

        # Add modules to the manager.
        self.comm_manager.add_module(self.publisher)
        self.comm_manager.add_module(self.subscriber)

        # Command tracking.
        self._seq = 0
        self._pending_acks: Dict[int, Tuple[float, str]] = {}
        self._ack_timeout = ack_timeout_s
        self._last_intent_sent: Optional[Dict[str, Any]] = None
        self._last_intent_sent_comp: Optional[Dict[str, Any]] = None
        self._last_intent_sent_time: float = 0.0
        self._intent_heartbeat_s: float = 0.080  # 12.5 Hz heartbeat (faster than 10 Hz publish to ensure delivery)

        # Set up callbacks.
        self._set_up_callbacks()

        # Watchdog task.
        self._control_tick_task: Optional[asyncio.Task] = None
        self._watchdog_task: Optional[asyncio.Task] = None
        self._telemetry_poll_task: Optional[asyncio.Task] = None
        self._running = False
        self._last_status_request_time: float = 0.0
        self._status_probe_pending: bool = False
        self._last_telem_log_time: float = 0.0
        self._last_ack_log_time: float = 0.0
        self._watchdog_log_interval_s: float = 0.5
        self._reconnect_in_progress: bool = False
        self._last_reconnect_time: float = 0.0
        self._reconnect_cooldown_s: float = 5.0
        self._reconnect_task: Optional[asyncio.Task] = None
        self._telemetry_stalled: bool = False
        self._diagnostic_reconnect_attempted: bool = False
        self._diagnostic_reconnect_task: Optional[asyncio.Task] = None
        self._telemetry_poll_rate_hz: float = max(1.0, float(telemetry_poll_rate_hz))

    def _set_up_callbacks(self):
        """Set up callbacks for publisher and subscriber."""

        # Publisher callback to update intent before each send.
        def on_publish(data: Dict[str, Any]):
            # Get ramped velocity and update intent.
            v, w = self.vel_ramp.get_current()
            
            # Safety gate: zero velocity if motion not enabled.
            if not self.state_machine.is_motion_enabled():
                v, w = 0.0, 0.0
                self.intent_gen.clear_actuators()
            
            # Stopped modes force motion zero and disable actuators.
            if self.state_machine.state in (RobotState.STOPPED, RobotState.FAULT, RobotState.ESTOPPED):
                v, w = 0.0, 0.0
                self.intent_gen.clear_actuators()
            
            # Update motion intent.
            self.intent_gen.set_motion(v, w)
            
            # Get complete intent dict and add sequence number.
            intent_dict = self.intent_gen.get_intent_dict()
            now = time.time()
            if intent_dict:
                compare_dict = {k: v for k, v in intent_dict.items() if k != "ts"}
                changed = compare_dict != (self._last_intent_sent_comp or {})
                if changed or (now - self._last_intent_sent_time) >= self._intent_heartbeat_s:
                    seq = self._get_next_seq()
                    intent_msg = make_envelope("INTENT", intent_dict, seq=seq, ts_ms=intent_dict.get("ts"))
                    self.publisher.update_data(intent_msg)
                    self._last_intent_sent = dict(intent_dict)
                    self._last_intent_sent_comp = dict(compare_dict)
                    self._last_intent_sent_time = now

                if "mode" in intent_dict:
                    print(f"[ControlLoop] Publishing intent with mode: {intent_dict['mode']}")
            else:
                # Send heartbeat to keep MCU watchdog fed even with no active intent.
                if (now - self._last_intent_sent_time) >= self._intent_heartbeat_s:
                    ts_ms = int(time.time() * 1000)
                    heartbeat = {"ts": ts_ms}
                    seq = self._get_next_seq()
                    intent_msg = make_envelope("INTENT", heartbeat, seq=seq, ts_ms=ts_ms)
                    self.publisher.update_data(intent_msg)
                    self._last_intent_sent = dict(heartbeat)
                    self._last_intent_sent_comp = {}
                    self._last_intent_sent_time = now

        self.publisher.register_callback(on_publish)

        # Subscriber callback to handle telemetry.
        def on_telemetry(data: Dict[str, Any]):
            try:
                msg_type, payload, meta = parse_envelope(data)
            except ValueError as e:
                print(f"[ControlLoop] Dropping invalid message: {e}")
                return
            def try_fault_recover(trigger: str):
                """Attempt to recover from FAULT once comms have resumed."""
                if self.state_machine.estop_active:
                    return
                if self.state_machine.state == RobotState.FAULT and not self.state_machine._has_blocking_fault():
                    try:
                        self.state_machine.transition_to(RobotState.IDLE, trigger=trigger)
                    except Exception as e:
                        print(f"Failed to transition to IDLE: {e}")

            if msg_type in ("STAT", "S"):
                had_comm_fault = COMM_TIMEOUT_CODE in self.state_machine.active_faults
                if self._telemetry_stalled and not self._auto_reconnect:
                    self._telemetry_stalled = False
                    print("[ControlLoop] Telemetry resumed")
                # Store previous state for transition detection
                prev_state = self.state_machine.state
                
                if msg_type == "S":
                    state_id = payload.get("st")
                    state_enum = robot_state_from_id(state_id)
                    telemetry_state = state_enum.value if state_enum else "IDLE"
                    telemetry_faults = payload.get("f", []) or []
                    telemetry_estop = bool(payload.get("e", 0))
                else:
                    telemetry_state = payload.get("state", "IDLE")
                    telemetry_faults = payload.get("faults", []) or []
                    telemetry_estop = payload.get("estop", False)

                # Update state machine from telemetry
                self.state_machine.update_from_telemetry(
                    state=telemetry_state,
                    faults=telemetry_faults,
                    estop=telemetry_estop
                )
                
                # Detect and handle state transitions
                if prev_state != self.state_machine.state:
                    print(f"[ControlLoop] State transition: {prev_state.value} -> {self.state_machine.state.value}")
                    
                    # Clear mode intent on any state change
                    self.intent_gen.clear_mode()
                    
                    # Clear control intents based on transition type
                    if self.state_machine.state == RobotState.STOPPED:
                        # Successfully stopped - clear stop intent
                        self.intent_gen.clear_control()
                        print("[ControlLoop] Stop applied, clearing control intent")
                    elif self.state_machine.state == RobotState.ESTOPPED:
                        # E-stop latched on MCU; clear estop intent to avoid re-triggering
                        self.intent_gen.clear_control()
                        print("[ControlLoop] E-stop latched, clearing control intent")
                    
                    elif prev_state == RobotState.STOPPED:
                        # Successfully resumed - clear resume intent
                        self.intent_gen.clear_control()
                        print("[ControlLoop] Resume applied, clearing control intent")
                    
                    # Any stopped/fault state forces motion and actuators off.
                    if self.state_machine.state in (RobotState.STOPPED, RobotState.FAULT, RobotState.ESTOPPED):
                        self.intent_gen.clear_motion()
                        self.intent_gen.clear_actuators()
                        self.vel_ramp.reset()
                
                # Fast error response - immediately clear motion on blocking faults
                if self.state_machine._has_blocking_fault():
                    self.intent_gen.clear_motion()
                    self.intent_gen.clear_actuators()
                    self.vel_ramp.reset()
                
                # Auto-recover comm timeout when telemetry resumes.
                if COMM_TIMEOUT_CODE in self.state_machine.active_faults:
                    if self.state_machine.remove_fault(COMM_TIMEOUT_CODE):
                        print("[ControlLoop] Telemetry resumed, cleared COMM_TIMEOUT fault")
                        try_fault_recover("comm_recovered")
                elif had_comm_fault:
                    try_fault_recover("comm_recovered")

                # Update odometry from encoder/IMU data.
                if self.odometry is not None:
                    enc = self.get_encoder_data()
                    if enc:
                        self.odometry.update(enc['left'], enc['right'])
                    
                    imu = self.get_imu_data()
                    if imu and imu['valid']:
                        self.odometry.update_imu(imu['yaw_deg'])
            
            elif msg_type == "ACK":
                # Handle command acknowledgment
                had_ack_fault = ACK_TIMEOUT_CODE in self.state_machine.active_faults
                ack_seq = meta.get("seq", payload.get("seq"))
                if ack_seq in self._pending_acks:
                    _sent_at, cmd_name = self._pending_acks.pop(ack_seq)
                    if cmd_name == "STATUS":
                        self._status_probe_pending = False
                
                # Auto-recover ACK timeout fault on successful ACK.
                if ACK_TIMEOUT_CODE in self.state_machine.active_faults:
                    if self.state_machine.remove_fault(ACK_TIMEOUT_CODE):
                        print("[ControlLoop] ACK received, cleared ACK_TIMEOUT fault")
                        try_fault_recover("ack_recovered")
                elif had_ack_fault:
                    try_fault_recover("ack_recovered")
            
            elif msg_type == "FAULT":
                # Handle fault event notification
                fault_code = payload.get("code")
                if fault_code:
                    self.state_machine.add_fault(fault_code)

        self.subscriber.register_callback(on_telemetry)

    async def _send_command_direct(self, cmd: Command):
        """Send command directly. NOT FOR OPERATIONAL USE."""
        seq = self._get_next_seq()
        msg = cmd.to_message(seq)
        self.transport.send(msg)

        if cmd.REQUIRE_ACK:
            self._pending_acks[seq] = (time.time(), cmd.NAME)
            
            # Wait for ACK.
            timeout = 0.5
            start = time.time()
            while seq in self._pending_acks:
                if time.time() - start > timeout:
                    raise TimeoutError(f"ACK timeout for {cmd.NAME}")
                await asyncio.sleep(0.01)

    async def _wait_for_stat(self, timeout_s: float, max_age_s: float = 1.0, label: str = "") -> bool:
        """Wait for a fresh STAT message (telemetry-driven sync)."""
        start = time.time()
        while time.time() - start < timeout_s:
            latest = self.subscriber.get_latest()
            if latest:
                try:
                    msg_type, _payload, _meta = parse_envelope(latest)
                except ValueError:
                    await asyncio.sleep(0.02)
                    continue
                if msg_type in ("STAT", "S") and self.subscriber.get_age() <= max_age_s:
                    return True
            await asyncio.sleep(0.02)
        if label:
            print(f"[ControlLoop] Timed out waiting for STAT ({label})")
        return False

    async def initialize(self) -> bool:
        """
        Initialize robot from BOOT to IDLE.
        
        Only case where commands are sent directly over transport.
        """
        # Send HELLO handshake.
        hello_cmd = Hello(version=PROTOCOL_VERSION)
        time.sleep(1.0)
        await self._send_command_direct(hello_cmd)
        await asyncio.sleep(0.1)

        # Wait for telemetry-driven STAT instead of polling STATUS.
        if not await self._wait_for_stat(timeout_s=2.0, label="init"):
            print("[Initialize] No STAT received after HELLO")
            return False

        # Transition from BOOT to IDLE.
        print(f"[Initialize] Current state: {self.state_machine.state}")
        mode_cmd = Mode(mode=RobotMode.IDLE)
        await self._send_command_direct(mode_cmd)
        if not await self.wait_for_state(RobotState.IDLE, timeout=2.0):
            print(f"[Initialize] State after MODE command: {self.state_machine.state}")
            return False
        print(f"[Initialize] State after MODE command: {self.state_machine.state}")

        # Send a heartbeat intent immediately to keep MCU watchdog fed.
        self._send_heartbeat_intent()

        return self.state_machine.state == RobotState.IDLE

    def _send_heartbeat_intent(self):
        ts_ms = int(time.time() * 1000)
        heartbeat = {"ts": ts_ms}
        seq = self._get_next_seq()
        intent_msg = make_envelope("INTENT", heartbeat, seq=seq, ts_ms=ts_ms)
        self.publisher.update_data(intent_msg)
        self._last_intent_sent = dict(heartbeat)
        self._last_intent_sent_comp = {}
        self._last_intent_sent_time = time.time()

    async def start(self):
        """Start the controller."""
        if self._running:
            return
        self._running = True
        
        if not self.transport.connected:
            self.transport.connect()

        # Start publisher and subscriber.
        await self.comm_manager.start_all()

        # Clear any stale serial input after reconnecting.
        try:
            self.transport.flush_input()
        except Exception:
            pass

        # Start control tick loop.
        self._control_tick_task = asyncio.create_task(self._control_tick_loop())

        # Start telemetry polling loop (poll-based instead of push-based).
        self._telemetry_poll_task = asyncio.create_task(self._telemetry_poll_loop())

        # Start watchdog.
        self._watchdog_task = asyncio.create_task(self._watchdog_loop())

    async def stop(self):
        """Stop the controller."""
        if not self._running:
            return
        self._running = False

        # Stop control tick loop.
        if self._control_tick_task:
            self._control_tick_task.cancel()
            try:
                await self._control_tick_task
            except asyncio.CancelledError:
                pass

        # Stop telemetry poll loop.
        if self._telemetry_poll_task:
            self._telemetry_poll_task.cancel()
            try:
                await self._telemetry_poll_task
            except asyncio.CancelledError:
                pass

        # Stop the watchdog.
        if self._watchdog_task:
            self._watchdog_task.cancel()

            try:
                await self._watchdog_task
            except asyncio.CancelledError:
                pass

        # Stop publisher and subscriber.
        await self.comm_manager.stop_all()

        # Stop any reconnect attempt.
        if self._reconnect_task and not self._reconnect_task.done():
            self._reconnect_task.cancel()
            try:
                await self._reconnect_task
            except asyncio.CancelledError:
                pass

        if self.transport.connected:
            self.transport.disconnect()

    async def _attempt_reconnect(self):
        if self._reconnect_in_progress:
            return
        now = time.time()
        if now - self._last_reconnect_time < self._reconnect_cooldown_s:
            return
        self._reconnect_in_progress = True
        self._last_reconnect_time = now
        try:
            print("[ControlLoop] Telemetry stalled, attempting reconnect...")
            
            # Stop comm loops
            await self.comm_manager.stop_all()
            
            # HARD RESET: Close and wait longer
            if self.transport.connected:
                self.transport.disconnect()
            await asyncio.sleep(2.0)  # Increased from 0.5s - give Arduino time to fully reset
            
            # Reconnect
            self.transport.connect()
            await asyncio.sleep(2.0)  # Increased from 1.0s - wait for Arduino boot
            
            # Flush stale data
            try:
                self.transport.flush_input()
                await asyncio.sleep(0.5)  # Let flush complete
            except Exception:
                pass
            
            # Restart comm loops
            self.publisher.pause()
            await self.comm_manager.start_all()
            
            # Clear state
            self._pending_acks.clear()
            self._status_probe_pending = False
            
            # MULTIPLE HANDSHAKE ATTEMPTS
            handshake_ok = False
            for attempt in range(3):
                try:
                    print(f"[ControlLoop] Handshake attempt {attempt + 1}/3")
                    await self._send_command_direct(Hello(version=PROTOCOL_VERSION))
                    if not await self._wait_for_stat(timeout_s=2.0, label="reconnect"):
                        raise TimeoutError("No STAT received after HELLO")
                    print("[ControlLoop] Handshake successful")
                    handshake_ok = True
                    break
                except Exception as e:
                    print(f"[ControlLoop] Handshake attempt {attempt + 1} failed: {e}")
                    if attempt == 2:
                        print(f"[ControlLoop] All handshake attempts failed")
                    await asyncio.sleep(1.0)
            
            if handshake_ok:
                self.publisher.resume()
            self._send_heartbeat_intent()
            
        finally:
            self._reconnect_in_progress = False

    async def _diagnostic_reconnect_once(self):
        """One-shot reconnect attempt for diagnostics (no retries)."""
        if self._reconnect_in_progress:
            return
        self._reconnect_in_progress = True
        try:
            print("[ControlLoop] Diagnostic reconnect (one-shot)")
            await self.comm_manager.stop_all()
            if self.transport.connected:
                self.transport.disconnect()
            await asyncio.sleep(1.0)
            self.transport.connect()
            await asyncio.sleep(1.0)
            try:
                self.transport.flush_input()
            except Exception:
                pass
            await self.comm_manager.start_all()
            try:
                await self._send_command_direct(Hello(version=PROTOCOL_VERSION))
                if await self._wait_for_stat(timeout_s=2.0, label="diagnostic"):
                    print("[ControlLoop] Diagnostic HELLO ACK received")
                else:
                    raise TimeoutError("No STAT received after HELLO")
            except Exception as e:
                print(f"[ControlLoop] Diagnostic HELLO failed: {e}")
        finally:
            self._reconnect_in_progress = False
            
    async def _watchdog_loop(self):
        """Monitor communication health."""
        while self._running:
            try:
                # Check ACK timeouts.
                now = time.time()
                timed_out = []

                for seq, (timestamp, cmd_name) in list(self._pending_acks.items()):
                    if now - timestamp > self._ack_timeout:
                        timed_out.append((seq, cmd_name))

                if timed_out:
                    for seq, cmd_name in timed_out:
                        now = time.time()
                        if now - self._last_ack_log_time >= self._watchdog_log_interval_s:
                            print(f"Watchdog ACK timeout: {cmd_name} (seq={seq})")
                            self._last_ack_log_time = now
                        del self._pending_acks[seq]
                        if cmd_name == "STATUS":
                            self._status_probe_pending = False

                    # Trigger ACK timeout fault.
                    self.state_machine.add_fault(ACK_TIMEOUT_CODE, auto_transition=False)
                    if self.state_machine.state not in (RobotState.FAULT, RobotState.ESTOPPED):
                        try:
                            self.state_machine.transition_to(RobotState.FAULT, trigger="ack_timeout")
                        except Exception as e:
                            print(f"Failed to transition to FAULT: {e}")
                    self.stop_motion()
                    self.intent_gen.clear_actuators()

                # Check telemetry age but skip if system is just started.
                telem_age = self.subscriber.get_age()

                if telem_age != float('inf') and telem_age > 3.0:
                    now = time.time()
                    if now - self._last_telem_log_time >= self._watchdog_log_interval_s:
                        print(f"Watchdog  telemetry timeout: {telem_age:.2f}s")
                        self._last_telem_log_time = now

                    # Check if RX thread is dead but port still open - cheaper recovery than full reconnect
                    # Only applies when using RX thread mode (not sync mode)
                    tstats = self.transport.get_stats()
                    if tstats.get('use_rx_thread', False):  # Only check if using thread mode
                        if not tstats['rx_thread_alive'] and tstats['serial_open']:
                            print("[ControlLoop] RX thread dead, attempting restart")
                            if hasattr(self.transport, '_restart_rx_thread'):
                                if self.transport._restart_rx_thread():
                                    print("[ControlLoop] RX thread restarted successfully")
                                    continue  # Skip reconnect, give thread a chance

                    self.state_machine.add_fault(70, auto_transition=False)
                    if self.state_machine.state not in (RobotState.FAULT, RobotState.ESTOPPED):
                        try:
                            self.state_machine.transition_to(RobotState.FAULT, trigger="telemetry_timeout")
                        except Exception as e:
                            print(f"Failed to transition to FAULT: {e}")
                    self.stop_motion()
                    self.intent_gen.clear_actuators()
                    if self._auto_reconnect:
                        if not self._reconnect_in_progress:
                            self._reconnect_task = asyncio.create_task(self._attempt_reconnect())
                    else:
                        if not self._telemetry_stalled:
                            print("[ControlLoop] Telemetry stalled; auto_reconnect disabled (diagnostic mode)")
                            self._telemetry_stalled = True
                            if self._diagnostic_reconnect_enabled and not self._diagnostic_reconnect_attempted:
                                self._diagnostic_reconnect_attempted = True
                                self._diagnostic_reconnect_task = asyncio.create_task(self._diagnostic_reconnect_once())
                            elif not self._diagnostic_reconnect_enabled:
                                print("[ControlLoop] Diagnostic reconnect disabled; leaving port open for observation")

                # Check degraded comm timeout.
                self.state_machine.check_degraded_timeout()

            except Exception as e:
                print(f"Control watchdog error: {e}")

            # Sleep for 100 ms.
            await asyncio.sleep(0.1)

    async def _control_tick_loop(self):
        period = 1.0 / self.publisher.rate_hz

        while self._running:
            try:
                # Update ramped velocity,
                v, w = self.vel_ramp.update()

                # Apply safety gating.
                if not self.state_machine.is_motion_enabled():
                    if not self.vel_ramp.is_stopped():
                        self.vel_ramp.reset()

            except Exception as e:
                print(f"Control loop tick error: {e}")

            await asyncio.sleep(period)

    async def _telemetry_poll_loop(self):
        """
        Poll for telemetry by sending STATUS requests.
        This prevents Arduino from autonomously pushing data and overwhelming serial.
        """
        period = 1.0 / self._telemetry_poll_rate_hz

        while self._running:
            try:
                # Send STATUS command (no ACK required, just triggers STAT response)
                # Start polling immediately to support initialization
                seq = self._get_next_seq()
                status_cmd = Status()
                msg = status_cmd.to_message(seq)
                self.transport.send(msg)
            except Exception as e:
                print(f"Telemetry poll error: {e}")

            await asyncio.sleep(period)

    async def wait_for_state(self, target_state: RobotState, timeout: float = 2.0) -> bool:
        """
        Blocks until the telemetry confirms the robot has reached the target state.
        """
        start_time = time.time()
        while self.state_machine.state != target_state:
            if time.time() - start_time > timeout:
                return False
            # Yield to allow the subscriber task to process incoming telemetry
            await asyncio.sleep(0.02) 
        return True

    async def send_command(self, cmd: Command):
        """Send a command that requires ACK."""
        # Validate that command is allowed.
        if not self.state_machine.can_accept_command(cmd):
            raise ValueError(f"Command {cmd.NAME} not allowed in state {self.state_machine.state.value}")
        
        # Build message.
        seq = self._get_next_seq()
        msg = cmd.to_message(seq)

        # Send the message.
        self.transport.send(msg)

        # Track ACK if required.
        if cmd.REQUIRE_ACK:
            self._pending_acks[seq] = (time.time(), cmd.NAME)

            # BLOCK until ACK is received or timeout occurs.
            start = time.time()
            while seq in self._pending_acks:
                if time.time() - start > self._ack_timeout:
                    # Align with watchdog: raise ACK_TIMEOUT fault + FAULT state.
                    self.state_machine.add_fault(ACK_TIMEOUT_CODE, auto_transition=False)
                    if self.state_machine.state not in (RobotState.FAULT, RobotState.ESTOPPED):
                        try:
                            self.state_machine.transition_to(RobotState.FAULT, trigger="ack_timeout")
                        except Exception as e:
                            print(f"Failed to transition to FAULT: {e}")
                    raise TimeoutError(f"ACK timeout for {cmd.NAME}")
                await asyncio.sleep(0.01) # Yield to the subscriber.

    def _get_next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) % 65536
        return seq
    
    # Convenience methods for setting intents.
    def set_velocity(self, v: float, w: float):
        self.vel_ramp.set_target(v, w)

    def stop_motion(self):
        self.vel_ramp.set_target(0.0, 0.0)

    def set_auger(self, enabled: bool):
        self.intent_gen.set_auger(enabled)

    def set_salt(self, enabled: bool):
        self.intent_gen.set_salt(enabled)

    def set_chute(self, angle: float):
        self.intent_gen.set_chute(angle)

    def set_headlight(self, enabled: bool):
        self.intent_gen.set_headlight(enabled)

    def set_status_light(self, state: str):
        self.intent_gen.set_status_light(state)

    def set_mode(self, mode: str):
        if isinstance(mode, RobotMode):
            mode_str = mode.value
        else:
            mode_str = mode
        self.intent_gen.set_mode(mode_str)

    def estop(self):
        # E-stop is an immediate hard stop.
        self.intent_gen.request_estop()
        self.intent_gen.clear_actuators()
        self.vel_ramp.reset()

    def resume(self):
        self.intent_gen.request_resume()

    def reset_faults(self, fault_codes: Optional[List[int]] = None):
        self.intent_gen.request_reset_faults(fault_codes)

    def get_status(self) -> Dict[str, Any]:
        status = {
            "running": self._running,
            "state_machine": self.state_machine.get_status(),
            "telemetry_age_ms": self.subscriber.get_age() * 1000,
            "has_error": self.subscriber.has_error(),
            "pending_acks": len(self._pending_acks),
            "velocity": {
                "current": self.vel_ramp.get_current(),
                "target": self.vel_ramp.get_target(),
                "at_target": self.vel_ramp.is_at_target(),
                "stopped": self.vel_ramp.is_stopped()
            },
            "latest_intent": self.publisher.get_latest(),
            "latest_telemetry": self.subscriber.get_latest(),
        }

        if self.odometry is not None:
            pose = self.odometry.get_pose()
            status["odometry"] = {
                "enabled": True,
                "pose": {
                    "x": pose.x,
                    "y": pose.y, 
                    "theta": pose.theta,
                    "heading_deg": self.odometry.get_heading_degrees()
                },
                "stats": self.odometry.get_stats()
            }
        else:
            status["odometry"] = {"enabled": False}

        return status
    
    def is_motion_enabled(self) -> bool:
        return self.state_machine.is_motion_enabled()

    def get_encoder_data(self) -> Optional[Dict[str, Any]]:
        """
        Get latest encoder data from telemetry.

        Returns:
            Dictionary with 'left' and 'right' encoder counts, or None if not available
        """
        telemetry = self.subscriber.get_latest()
        if not telemetry:
            return None

        try:
            _, payload, meta = parse_envelope(telemetry)
            if "enc" in payload:
                enc = payload.get("enc") or []
                if isinstance(enc, list) and len(enc) >= 2:
                    return {
                        "left": enc[0],
                        "right": enc[1],
                        "timestamp": meta.get("ts", 0)
                    }

            encoder_left = payload.get("encoder_left")
            encoder_right = payload.get("encoder_right")

            if encoder_left is not None and encoder_right is not None:
                return {
                    "left": encoder_left,
                    "right": encoder_right,
                    "timestamp": meta.get("ts", 0)
                }
        except (ValueError, KeyError):
            pass

        return None

    def get_imu_data(self) -> Optional[Dict[str, Any]]:
        """
        Get latest IMU data from telemetry.

        Returns:
            Dictionary with 'yaw' (degrees) and 'valid' flag, or None if not available
        """
        telemetry = self.subscriber.get_latest()
        if not telemetry:
            return None

        try:
            _, payload, meta = parse_envelope(telemetry)
            if "imu" in payload:
                imu = payload.get("imu") or []
                if isinstance(imu, list) and len(imu) >= 3:
                    yaw_cdeg = imu[2]
                    return {
                        "yaw_deg": yaw_cdeg / 100.0,
                        "valid": bool(payload.get("v", 0)),
                        "timestamp": meta.get("ts", 0)
                    }

            imu_yaw = payload.get("imu_yaw")
            imu_valid = payload.get("imu_valid", False)

            if imu_yaw is not None:
                return {
                    "yaw_deg": imu_yaw,
                    "valid": imu_valid,
                    "timestamp": meta.get("ts", 0)
                }
        except (ValueError, KeyError):
            pass

        return None
    
    def get_pose(self) -> Optional[Pose]:
        """Get current robot pose from odometry."""
        if self.odometry is None:
            return None
        return self.odometry.get_pose()
    
    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset odometry to specified position."""
        if self.odometry is not None:
            self.odometry.reset(x, y, theta)

    def get_odometry_stats(self) -> Optional[Dict[str, Any]]:
        if self.odometry is None:
            return None
        return self.odometry.get_stats()
    
    def is_at_velocity(self, tolerance: float = 0.01) -> bool:
        return self.vel_ramp.is_at_target(tolerance)
    
    def is_stopped(self, tolerance: float = 0.01) -> bool:
        return self.vel_ramp.is_stopped(tolerance)
    
