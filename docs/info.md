
# SnoBot: Autonomous Snow Removal System - Project Goals & Architecture
**Last Updated**: 2026-01-10\
**Project Status**: Planning & architecture phase\
**SBCP Version Target**: 0.3.0
___
## Executive Summary
SnoBot is an autonomous snow-clearing robot featuring a differential drive, auger, salt dispenser, and chute control. The system uses stereo vision based on OpenCV and VPI, plus YOLO for perception, with all intelligence on an Nvidia Jetson Orin Nano Super and safety-critical control on an Arduino Uno R4. The robot operates by having a user trace a perimeter, then autonomously fills the polygon using scanline patterns while avoiding obstacles.
___
## System Architecture
### Hardware Topology
```
Jetson Orin Nano Super (Brain)
├─ Stereo cameras (CSI) - perception
├─ All vision processing (depth + YOLO)
├─ All decision making & path planning
└─ USB connection to Arduino

Arduino R4 WiFi (Muscles + Sensors)
├─ Drive motors (2x brushed DC with encoders)
├─ Auger motor (BLDC,  <4500 RPM)
├─ Salt dispenser (solenoid + motor)
├─ Sensors:
│   ├─ Encoders (1200 ticks/rev quadrature)
│   ├─ IMU (BNO055, 9-DOF)
│   ├─ Battery monitor (40-52V)
│   └─ E-stop button (hardware interrupt)
└─ Safety enforcement (overcurrent, E-stop, watchdog)

Communication: USB Serial @ 115200 baud
Power: 48V Li-ion battery (40Ah, 1920Wh)
```
### Design Philosophy
- **Jetson = intelligence**: All perception, decision-making, path planning, obstacle avoidance
- **Arduino = Execution**: Sensor sampling, motor control, safety enforcement, telemetry
- **Event-Driven**: No busy-waiting, components react to state changes
- **Safety First**: Arduino enforces hard limits regardless of Jetson state
- **Fail-Safe**: Communication losses create graceful stops, while E-stops cause immediate halts
___
## High-Level Goals
### Operational Capabilities
- [_] Fully autonomous operation after initial perimeter setup
- [_] Per-instance machine calibration (cameras, motors, sensors)
- [_] Real-time depth perception + object detection for obstacle avoidance
- [_] Multiple operation modes (manual, autonomous, service/diagnostic)
- [_] Comprehensive safety system with E-stop as absolute authority
- [_] Continuous metrics collection for prototype analysis

### Mission Profile
1. **Setup Phase**: Operator manually drives perimeter of area to clear
2. **Initialization**: Robot uses vision to match starting position within perimeter
3. **Autonomous Clearing**: Scanline pattern fills polygon, dispensing salt as needed
4. **Obstacle Handling**: Detects and avoids obstacles, replans as needed
5. **Completion**: Returns to start or designated position when area cleared

## Module Architecture & Implementation Status
