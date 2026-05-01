"""
Simple IMU + Odometry Test Script

Tests:
1. IMU data is being received
2. Odometry updates from encoders
3. IMU fusion affects heading
4. All systems integrated correctly

Usage:
    python test_imu_simple.py [--port /dev/ttyACM0]
"""

import asyncio
import argparse
import sys
import math
from pathlib import Path

# Add project paths.
sys.path.insert(0, str(Path(__file__).parent.parent))

from sbcp.control_loop import ControlLoop
from sbcp.transport import AsyncSerialTransport


async def main(port: str = "/dev/ttyACM0"):
    print("=" * 60)
    print("SnoBot IMU + Odometry Test")
    print("=" * 60)
    print()
    
    # Initialize transport with correct settings
    # Match baud rate from config.h: SERIAL_BAUD 230400
    # Use RX thread on non-Windows platforms
    import platform
    transport = AsyncSerialTransport(
        port=port, 
        baud=230400,  # Must match Arduino config.h
        timeout=0.01,
        use_rx_thread=(platform.system() != "Windows")  # Thread mode for Linux/Mac
    )
    control = ControlLoop(
        transport, 
        enable_odometry=True,
        publish_rate_hz=10.0,
        subscribe_rate_hz=12.0
    )
    
    print("Connecting...")
    await control.start()
    
    print("Initializing...")
    if not await control.initialize():
        print("ERROR: Failed to initialize!")
        await control.stop()
        return
    
    print("✓ Connected")
    print()
    print("Test Plan:")
    print("1. Watch encoder counts change as robot moves")
    print("2. Watch IMU heading update")
    print("3. Watch odometry pose integrate both")
    print()
    print("Drive the robot in MANUAL mode and observe:")
    print("- Forward motion → X increases")
    print("- Left turn → Theta increases (CCW positive)")
    print("- IMU fusion → Heading stays accurate")
    print()
    print("Press Ctrl+C to stop")
    print("-" * 60)
    
    try:
        last_enc_left = None
        last_enc_right = None
        last_imu_yaw = None
        
        while True:
            # Get raw data
            enc_data = control.get_encoder_data()
            imu_data = control.get_imu_data()
            pose = control.get_pose()
            
            # Display
            print("\r" + " " * 80, end="")  # Clear line
            
            output = []
            
            # Encoder info
            if enc_data:
                left = enc_data['left']
                right = enc_data['right']
                
                # Show deltas
                if last_enc_left is not None:
                    dl = left - last_enc_left
                    dr = right - last_enc_right
                    enc_str = f"Enc: L={left:6d}({dl:+4d}) R={right:6d}({dr:+4d})"
                else:
                    enc_str = f"Enc: L={left:6d} R={right:6d}"
                
                output.append(enc_str)
                last_enc_left = left
                last_enc_right = right
            else:
                output.append("Enc: NO DATA")
            
            # IMU info
            if imu_data:
                yaw = imu_data['yaw_deg']
                valid = "✓" if imu_data['valid'] else "✗"
                
                # Show delta
                if last_imu_yaw is not None:
                    dyaw = yaw - last_imu_yaw
                    # Handle wraparound
                    if dyaw > 180:
                        dyaw -= 360
                    elif dyaw < -180:
                        dyaw += 360
                    imu_str = f"IMU: {yaw:6.1f}°({dyaw:+5.1f}°) {valid}"
                else:
                    imu_str = f"IMU: {yaw:6.1f}° {valid}"
                
                output.append(imu_str)
                last_imu_yaw = yaw
            else:
                output.append("IMU: NO DATA")
            
            # Odometry pose
            if pose:
                output.append(f"Pose: ({pose.x:5.2f}, {pose.y:5.2f}) @ {math.degrees(pose.theta):6.1f}°")
            else:
                output.append("Pose: NO ODOM")
            
            print("\r" + " | ".join(output), end="", flush=True)
            
            try:
                await asyncio.sleep(0.1)  # 10 Hz update
            except asyncio.CancelledError:
                # Clean break on Ctrl+C
                break
            
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("Final State")
        print("=" * 60)
        
        # Final data
        enc_data = control.get_encoder_data()
        imu_data = control.get_imu_data()
        pose = control.get_pose()
        stats = control.get_odometry_stats()
        
        print()
        if enc_data:
            print(f"Encoders:")
            print(f"  Left:  {enc_data['left']:8d} ticks")
            print(f"  Right: {enc_data['right']:8d} ticks")
        
        print()
        if imu_data:
            print(f"IMU:")
            print(f"  Yaw:   {imu_data['yaw_deg']:6.1f}°")
            print(f"  Valid: {imu_data['valid']}")
        
        print()
        if pose:
            print(f"Final Pose:")
            print(f"  X:     {pose.x:6.3f} m")
            print(f"  Y:     {pose.y:6.3f} m")
            print(f"  Theta: {pose.theta:6.3f} rad ({math.degrees(pose.theta):6.1f}°)")
        
        print()
        if stats:
            print(f"Odometry Stats:")
            print(f"  Distance: {stats['total_distance_m']:.3f} m")
            print(f"  Updates:  {stats['update_count']}")
            print(f"  m/tick:   {stats['meters_per_tick']:.6f}")
        
        print()
    
    finally:
        print("\nStopping...")
        await control.stop()
        print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple IMU + Odometry Test")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    args = parser.parse_args()
    
    try:
        asyncio.run(main(port=args.port))
    except KeyboardInterrupt:
        # Clean exit on Ctrl+C
        print("\n\nExiting...")
        sys.exit(0)
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)