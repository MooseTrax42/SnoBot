import asyncio
from sbcp.control_loop import ControlLoop
from sbcp.transport import DummyTransport
from sbcp.commands import RobotMode
from sbcp.types import RobotState
from sbcp.commands import *
from sbcp.errors import COMM_TIMEOUT_CODE, ACK_TIMEOUT_CODE

import sys
from pathlib import Path

# Add project paths.
sys.path.insert(0, str(Path(__file__).parent.parent))

async def test_rate_decoupling():
    transport = DummyTransport()
    controller = ControlLoop(transport, subscribe_rate_hz=10.0)

    await controller.start()
    await asyncio.sleep(2.0)

    age = controller.subscriber.get_age()
    latest = controller.subscriber.get_latest()

    print(f"Telemetry age: {age:.3f}s")
    print(f"Latest telemetry: {latest}")

    assert age < 0.15, "Telemetry should remain fresh despite slow polling"
    assert latest is not None
    assert latest.get("type") in ("STAT", "S")
    assert "data" in latest

    await controller.stop()

async def test_intent_latest_wins():
    transport = DummyTransport()
    controller = ControlLoop(transport)

    await controller.start()
    await controller.initialize()

    controller.set_mode(RobotMode.MANUAL)
    await asyncio.sleep(0.1)
    # Rapid updates faster than publish rate
    for i in range(10):
        controller.set_velocity(i * 0.1, 0.0)
        await asyncio.sleep(0.02)  # 50 Hz updates

    await asyncio.sleep(0.2)  # Let publisher run

    # Check the target velocity, not the ramped/current velocity
    target_v, _ = controller.vel_ramp.get_target()
    print(f"Target velocity: {target_v}")
    assert abs(target_v - 0.9) < 0.05, f"Expected target ~0.9, got {target_v}"

    await controller.stop()

async def test_velocity_ramping():
    transport = DummyTransport()
    controller = ControlLoop(transport, publish_rate_hz=20.0)

    await controller.start()
    await controller.initialize()

    controller.set_velocity(1.5, 0.0)

    samples = []
    for _ in range(10):
        samples.append(controller.vel_ramp.get_current()[0])
        await asyncio.sleep(0.05)

    print("Velocity samples:", samples)

    assert all(samples[i+1] >= samples[i] for i in range(len(samples)-1))
    assert samples[-1] < 1.5, "Should not jump to target instantly"

    await controller.stop()

async def test_ack_timeout_triggers_fault():
    """Debug version of the test with extra logging."""
    print("\n=== Starting ACK Timeout Debug Test ===\n")
    
    transport = DummyTransport()
    controller = ControlLoop(transport, ack_timeout_s=0.3)

    print("1. Starting controller...")
    await controller.start()
    
    print("2. Initializing (robot should go to IDLE)...")
    await controller.initialize()
    
    print(f"3. Current state: {controller.state_machine.state}")
    print(f"   Pending ACKs: {controller._pending_acks}")
    
    # Drop ACKs to force timeout without monkey-patching.
    transport.drop_acks = True

    print("\n4. Sending MODE command (should timeout)...")
    try:
        await controller.send_command(Mode(mode=RobotMode.MANUAL))
        print("   ERROR Command unexpectedly returned without timeout")
    except TimeoutError as e:
        print(f"   OK Command timed out: {e}")
    except Exception as e:
        print(f"   ERROR sending command: {e}")
        return

    # Check status at intervals
    print("\n5. Waiting for timeout (checking every 0.15s)...")
    for i in range(4):
        await asyncio.sleep(0.15)
        elapsed = i * 0.15 + 0.15
        print(f"\n   [{elapsed:.2f}s] Status check:")
        print(f"      State: {controller.state_machine.state.value}")
        print(f"      Active faults: {controller.state_machine.active_faults}")
        print(f"      Pending ACKs: {len(controller._pending_acks)}")
        print(f"      Motion enabled: {controller.state_machine.is_motion_enabled()}")

    print("\n6. Final status:")
    status = controller.get_status()
    print(f"   State: {status['state_machine']['state']}")
    print(f"   Faults: {status['state_machine']['active_faults']}")
    print(f"   Motion enabled: {status['state_machine']['motion_enabled']}")
    print(f"   Pending ACKs: {status['pending_acks']}")

    print("\n7. Checking test assertions...")
    expected_states = [RobotState.FAULT.value]
    actual_state = status["state_machine"]["state"]
    
    if actual_state in expected_states:
        print(f"   OK State is correct: {actual_state}")
    else:
        print(f"   ERROR State is WRONG: expected {expected_states}, got {actual_state}")
    
    if controller.vel_ramp.is_stopped():
        print("   OK Velocity is stopped")
    else:
        print("   ERROR Velocity is NOT stopped")

    assert actual_state == RobotState.FAULT.value
    assert ACK_TIMEOUT_CODE in controller.state_machine.active_faults
    assert controller.vel_ramp.is_stopped()

    await controller.stop()
    print("\n=== Test Complete ===\n")

async def test_telemetry_timeout():
    transport = DummyTransport()
    controller = ControlLoop(transport)

    await controller.start()
    await controller.initialize()

    # Kill telemetry
    transport.pause_telemetry()

    await asyncio.sleep(1.2)

    state = controller.state_machine.state
    print("State after telemetry loss:", state)

    assert state == RobotState.FAULT
    assert controller.vel_ramp.is_stopped()

    await controller.stop()

async def test_telemetry_recovery_to_idle():
    transport = DummyTransport()
    controller = ControlLoop(transport)

    await controller.start()
    await controller.initialize()

    # Kill telemetry to trigger COMM_TIMEOUT.
    transport.pause_telemetry()
    await asyncio.sleep(1.2)
    assert controller.state_machine.state == RobotState.FAULT
    assert COMM_TIMEOUT_CODE in controller.state_machine.active_faults

    # Resume telemetry and wait for recovery.
    transport.resume_telemetry()
    await asyncio.sleep(0.3)
    assert controller.state_machine.state == RobotState.IDLE
    assert COMM_TIMEOUT_CODE not in controller.state_machine.active_faults

    await controller.stop()

async def test_motion_blocked_by_state():
    transport = DummyTransport()
    controller = ControlLoop(transport)

    await controller.start()
    await controller.initialize()

    # Force STOPPED state
    controller.state_machine.transition_to(RobotState.STOPPED)

    controller.set_velocity(1.0, 0.0)
    await asyncio.sleep(0.2)

    v, w = controller.vel_ramp.get_current()
    print("Velocity in STOPPED:", v, w)

    assert abs(v) < 1e-3
    await controller.stop()

async def demo_full_system():
    transport = DummyTransport()
    controller = ControlLoop(transport)

    await controller.start()
    await controller.initialize()

    controller.set_mode(RobotMode.MANUAL)

    for v in [0.2, 0.4, 0.6, 0.0]:
        controller.set_velocity(v, 0.0)
        await asyncio.sleep(0.5)
        print("Intent:", controller.publisher.get_latest())
        print("Telemetry:", controller.subscriber.get_latest())

    controller.estop()
    await asyncio.sleep(1.0)

    controller.resume()
    await asyncio.sleep(1.0)

    await controller.stop()

async def run_all():
    # await test_rate_decoupling()
    # await test_intent_latest_wins()
    # await test_velocity_ramping()
    await test_ack_timeout_triggers_fault()
    # await test_telemetry_timeout()
    # await test_telemetry_recovery_to_idle()
    # await test_motion_blocked_by_state()
    # await demo_full_system()

if __name__ == "__main__":
    asyncio.run(run_all())
