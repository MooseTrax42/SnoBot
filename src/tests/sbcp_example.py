import asyncio

import sys
from pathlib import Path

# Add project paths.
sys.path.insert(0, str(Path(__file__).parent.parent))

from sbcp.transport import DummyTransport
from sbcp.control_loop import ControlLoop
from sbcp.commands import RobotMode, Mode
from sbcp.types import RobotState

async def main():
    # 1. Setup
    transport = DummyTransport()
    controller = ControlLoop(transport)

    try:
        # 2. Start the background loops (publisher, subscriber, watchdog)
        await controller.start()

        # 3. Handshake and sync state (BOOT -> IDLE)
        if not await controller.initialize():
            print("Failed to initialize robot!")
            return

        # 4. Enable Motion (IDLE -> MANUAL)
        # This sends the command and waits for the state transition confirmation
        await controller.send_command(Mode(mode=RobotMode.MANUAL))

        if await controller.wait_for_state(RobotState.MANUAL):
            print("Telemetry confirmed: Robot is in MANUAL mode.")
            
            print("Moving forward...")
            controller.set_velocity(1.0, 0.0)
        else:
            print("Error: Command was ACK'd but telemetry never reported MANUAL state!")
        
        await asyncio.sleep(2.0) 

        print("Turning...")
        controller.set_velocity(v=0.5, w=0.5)

        await asyncio.sleep(2.0)

        # 6. Stop
        print("Stopping...")
        controller.stop_motion() # Helper to set v=0, w=0
        
        # Wait for robot to actually stop (optional check)
        while not controller.is_stopped():
            await asyncio.sleep(0.1)

    finally:
        # 7. Cleanup
        await controller.stop()
        print("Completed!")

if __name__ == "__main__":
    asyncio.run(main())