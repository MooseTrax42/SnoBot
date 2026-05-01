# Vision-Communication Integration Plan
## Autonomous Perimeter Tracing and Area Filling with Obstacle Avoidance

**Date**: 2026-01-25
**Status**: Phase 1 Complete (Encoder Integration & Odometry)
**Project**: SnoBot Autonomous Snow Removal Robot

---

## Table of Contents
1. [Overview](#overview)
2. [User Requirements](#user-requirements)
3. [System Architecture](#system-architecture)
4. [Implementation Phases](#implementation-phases)
5. [Completed Work](#completed-work)
6. [Next Steps](#next-steps)
7. [Testing Plan](#testing-plan)

---

## Overview

This document outlines the integration of SnoBot's vision system (SBVS) and communication system (SBCP) to enable:

1. **Manual perimeter tracing** - Drive robot around boundary while recording path via odometry
2. **Persistent perimeter storage** - Save traced boundaries as YAML files
3. **Automatic path generation** - Create scanline coverage patterns within perimeter
4. **Real-time obstacle avoidance** - Use stereo depth + object detection to navigate safely
5. **Autonomous execution** - Follow paths with behavior control (auger, salt, chute)

### Design Decisions Made

Based on user input, the following approach was selected:

- **Teaching Method**: Manual drive - physically drive robot around boundary
- **Localization**: Wheel encoders + IMU fusion (encoders exist but needed software integration)
- **Behavior Zones**: Simple - single behavior for entire perimeter
- **Coverage Pattern**: Scanline/boustrophedon - parallel back-and-forth passes

---

## User Requirements

### Original Request
> "I'd like to be able to start drawing shapes and defining behaviors of the machine. I want to be able to save traced perimeters and then fill in automatically, being sure to navigate around obstacles as they appear."

### Key Capabilities Needed
1. **Shape Drawing**: Define work areas by driving perimeter
2. **Behavior Definition**: Map actuator settings (auger, salt, chute) to perimeters
3. **Perimeter Persistence**: Save/load traced boundaries
4. **Autonomous Filling**: Generate and execute coverage paths
5. **Obstacle Avoidance**: Real-time navigation around detected obstacles

---

## System Architecture

### Existing Systems (Discovered)

#### SBVS (SnoBot Vision System) v0.2.0
Located in `src/sbvs/`

**Capabilities**:
- Stereo camera system (1920×1080 downscaled to 640×360)
- VPI-accelerated depth estimation with safety cone mask (±35°)
- YOLOv12 nano object detection with IoU-based tracking
- Distance measurement methods: `distance_roi()`, `distance_at()`, `distance_forward_cone()`

**Key Files**:
- `camera/stereo_camera.py` - Dual camera synchronization
- `stereo/stereo_processor.py` - Depth computation
- `object/object_processor.py` - YOLO detection + tracking

#### SBCP (SnoBot Communication Protocol) v0.3.0
Located in `src/sbcp_new/`

**Capabilities**:
- JSON-over-serial at 115,200 baud (Arduino ↔ Jetson)
- Velocity control: `SET_VELOCITY(v, w)` with ramping
- State machine: 8 states (BOOT, IDLE, MANUAL, AUTO, STOPPED, etc.)
- Actuator commands: auger, salt, chute, lights
- Telemetry at 12Hz

**Key Files**:
- `transport.py` - AsyncSerialTransport
- `control_loop.py` - Main orchestrator with IntentGenerator
- `commands.py` - Command definitions

### New System: SBAN (SnoBot Autonomy) v0.1.0
Located in `src/sban/`

**Module Structure**:
```
src/sban/
├── localization/
│   └── odometry.py              # Encoder + IMU fusion for pose tracking
├── perimeter/
│   ├── perimeter_recorder.py    # Records path during manual drive
│   ├── perimeter_storage.py     # YAML persistence
│   └── perimeter_data.py        # Data structures
├── planning/
│   ├── coverage_planner.py      # Scanline path generation
│   └── path_data.py             # Path data structures
├── navigation/
│   ├── collision_checker.py     # SBVS depth + detection integration
│   └── pure_pursuit.py          # Path following controller
├── state/
│   └── mission_manager.py       # Mission state machine
└── integration/
    └── autonomy_controller.py   # Main orchestrator

data/sban/
├── perimeters/*.yaml            # Saved perimeter polygons
├── missions/*.yaml              # Mission configurations
└── robot_params.yaml            # Robot physical parameters
```

---

## Implementation Phases

### Phase 1: Encoder Integration & Odometry ✅ COMPLETED

**Goal**: Enable pose tracking (x, y, theta) during robot operation

#### Arduino Side
- ✅ **encoder_reader.h/cpp** - Quadrature encoder reader with interrupt-based counting
  - Pins: 2 (left A), 4 (left B), 7 (right A), 8 (right B)
  - Direction detection via B channel state
  - Thread-safe counter access

- ✅ **system_test.ino** - Modified to:
  - Initialize `encoders.begin()` in setup
  - Send encoder data in telemetry: `encoder_left`, `encoder_right`
  - Send IMU data: `imu_yaw`, `imu_valid`

#### Python Side
- ✅ **control_loop.py** - Added accessor methods:
  - `get_encoder_data()` → {left, right, timestamp}
  - `get_imu_data()` → {yaw_deg, valid, timestamp}

- ✅ **robot_params.yaml** - Robot physical parameters:
  ```yaml
  wheelbase_m: 0.588625
  wheel_radius_m: 0.127
  encoder_ticks_per_rev: 600
  gear_ratio: 18.0
  # Calculated: ~13,750 ticks/meter
  ```

- ✅ **odometry.py** - Differential drive odometry estimator:
  - Dead reckoning from encoder deltas
  - Midpoint integration for curve accuracy
  - IMU fusion with 30% weight for heading correction
  - Pose tracking: `Pose(x, y, theta)`

**Key Equations**:
```python
# Meters per encoder tick
meters_per_tick = (2π × wheel_radius) / (ticks_per_rev × gear_ratio)
                = (2π × 0.127) / (600 × 18.0)
                = 0.7980 / 10800 = 0.0000739 m/tick

# Differential drive kinematics
delta_s = (delta_left_m + delta_right_m) / 2.0         # Distance traveled
delta_theta = (delta_right_m - delta_left_m) / wheelbase_m  # Heading change

# Pose update (midpoint method)
theta_mid = theta + delta_theta / 2.0
x += delta_s × cos(theta_mid)
y += delta_s × sin(theta_mid)
theta += delta_theta
```

---

### Phase 2: Perimeter Teaching Mode (NEXT)

**Goal**: Record perimeter by manually driving robot around boundary

#### Components to Implement

**1. Perimeter Data Structures** (`perimeter_data.py`)
```python
@dataclass
class PerimeterData:
    name: str
    waypoints: List[Pose]        # Raw recorded poses
    polygon: List[Point]         # Simplified polygon (Douglas-Peucker)
    behaviors: BehaviorConfig
    created_at: datetime
```

**2. Perimeter Recorder** (`perimeter_recorder.py`)
```python
class PerimeterRecorder:
    async def start_teaching()      # Transitions to MANUAL mode, resets odometry
    async def record_loop()         # Records poses at 5Hz
    def finish_teaching()           # Validates closure (start/end within 1m)
    def get_perimeter_data()        # Returns recorded data
```

**Workflow**:
1. User calls `start_teaching()`
2. Robot enters MANUAL mode
3. Odometry resets to (0, 0, 0)
4. Recording loop samples pose at 5Hz
5. User drives robot around perimeter
6. User calls `finish_teaching()`
7. System validates closure and simplifies polygon

**3. Perimeter Storage** (`perimeter_storage.py`)
```python
class PerimeterStorage:
    def save(perimeter, filename)           # Writes YAML
    def load(filename)                      # Reads YAML
    def validate_polygon(polygon)           # Checks closure, self-intersection
    def simplify_polygon(waypoints, tol)    # Douglas-Peucker algorithm
```

**YAML Format**:
```yaml
perimeter:
  version: "1.0"
  name: "Front Driveway"
  created_at: "2026-01-25T14:30:00Z"

  waypoints:  # Raw recorded poses
    - {x: 0.0, y: 0.0, theta: 0.0, timestamp: 1706192400.123}
    - {x: 1.5, y: 0.1, theta: 0.05, timestamp: 1706192402.456}
    # ...

  polygon:  # Simplified for planning
    - {x: 0.0, y: 0.0}
    - {x: 10.0, y: 0.0}
    - {x: 10.0, y: 5.0}
    - {x: 0.0, y: 5.0}

  behaviors:
    auger_enabled: true
    salt_enabled: true
    chute_angle_deg: 45
    velocity_scale: 1.0
```

---

### Phase 3: Path Planning

**Goal**: Generate scanline coverage path from perimeter polygon

#### Scanline Coverage Algorithm

**Dependencies**: Add `shapely>=2.0` to requirements.txt

**Implementation** (`coverage_planner.py`):
```python
class ScanlinePlanner:
    def plan(polygon, line_spacing_m=0.6) -> CoveragePath:
        # 1. Compute polygon bounding box
        # 2. Generate horizontal scanlines at line_spacing_m intervals
        # 3. Clip scanlines to polygon interior using Shapely
        # 4. Connect segments in boustrophedon pattern
        # 5. Generate waypoints at 0.5m intervals
```

**Algorithm Steps**:
1. Get bounding box of polygon
2. Create parallel lines spaced by robot width (0.6m)
3. Intersect lines with polygon using Shapely
4. Connect line segments with turning paths
5. Optimize to minimize number of turns

**Output** (`path_data.py`):
```python
@dataclass
class CoveragePath:
    waypoints: List[Waypoint]
    total_length_m: float
    estimated_time_s: float

@dataclass
class Waypoint:
    x: float
    y: float
    theta: float
    target_velocity_ms: float
```

---

### Phase 4: Collision Detection

**Goal**: Integrate SBVS depth + object detection for obstacle awareness

#### Vision Bridge (`collision_checker.py`)

**Integration with SBVS**:
```python
class CollisionChecker:
    def __init__(self, stereo_processor, object_processor):
        self.stereo = stereo_processor
        self.objects = object_processor

    def check_forward_clear(self, horizon_m=2.0):
        # 1. Check stereo depth in forward cone
        distance = stereo_result.distance_forward_cone(
            cone_mask, percentile=5
        )

        # 2. Check confirmed object detections
        confirmed = [d for d in detections if d.get("confirmed")]

        # 3. Return (is_clear, distance_to_obstacle)
        return (distance > horizon_m, distance)

    def get_confirmed_objects(self, max_distance_m=3.0):
        # Filter ObjectProcessor detections for confirmed=True
        # Return only objects within range
        pass
```

**Obstacle Response Strategy**:
1. **Detection**: Obstacle within 3.0m
2. **Classification**:
   - Confirmed tracked object: Stop, mark location
   - Depth-only detection: Validate over 3-5 frames
3. **Action**:
   - If small (<0.5m): Local path adjustment
   - If large (>0.5m): Stop and request replan
4. **Recovery**: Return to global path when clear

---

### Phase 5: Path Following & Autonomous Control

**Goal**: Execute coverage path while avoiding obstacles

#### Pure Pursuit Controller (`pure_pursuit.py`)

**Classic path following algorithm**:
```python
class PurePursuitController:
    def __init__(self, lookahead_distance_m=1.5):
        self.lookahead = lookahead_distance_m

    def compute_velocity(self, current_pose, path):
        # 1. Find lookahead point on path
        # 2. Compute curvature to reach lookahead
        # 3. Convert to (v, w) velocity commands
        # 4. Respect max velocity limits
        return (v, w)
```

**Lookahead Distance**: 1.5m (tunable parameter)

#### Mission State Machine (`mission_manager.py`)

**States**:
- IDLE: No active mission
- TEACHING: Recording perimeter
- VALIDATING: Checking polygon validity
- PLANNING: Generating coverage path
- READY: Waiting for autonomous start
- EXECUTING: Autonomous operation
- PAUSED: User paused mission
- REPLANNING: Generating new path around obstacle
- COMPLETED: Mission finished
- ABORTED: Mission stopped with error

**State Transitions**:
```
IDLE → TEACHING → VALIDATING → PLANNING → READY → EXECUTING → COMPLETED
                                                      ↓
                                                   PAUSED
                                                      ↓
                                                 REPLANNING
```

#### Main Autonomy Controller (`autonomy_controller.py`)

**Main Execution Loop** (20 Hz):
```python
class AutonomyController:
    async def execute_mission(self, mission_config):
        while not mission_complete:
            # 1. Get current pose from odometry
            current_pose = self.odometry.get_pose()

            # 2. Check for obstacles
            is_clear, distance = self.collision_checker.check_forward_clear()

            if not is_clear and distance < 1.0:
                # Emergency stop
                self.control_loop.set_velocity(0.0, 0.0)
                await self.handle_obstacle()
                continue

            # 3. Compute velocity commands (pure pursuit)
            v, w = self.pure_pursuit.compute_velocity(current_pose, self.path)

            # 4. Apply behaviors for current zone
            self.control_loop.set_auger(mission_config.auger_enabled)
            self.control_loop.set_salt(mission_config.salt_enabled)
            self.control_loop.set_chute(mission_config.chute_angle)

            # 5. Send velocity command
            self.control_loop.set_velocity(v, w)

            # 6. Update progress
            self.mission_manager.update_progress(current_pose)

            await asyncio.sleep(0.05)  # 20 Hz
```

---

### Phase 6: Configuration & Integration

**Goal**: Tie all systems together with configuration and example scripts

#### Configuration Extension (`common_config.py`)

```python
@dataclass
class SBANConfig:
    # Planning parameters
    line_spacing_m: float = 0.6
    waypoint_spacing_m: float = 0.5

    # Navigation parameters
    lookahead_distance_m: float = 1.5
    obstacle_detection_range_m: float = 3.0
    obstacle_stop_distance_m: float = 1.0

    # Autonomous velocities (conservative)
    max_velocity_auto_ms: float = 1.0
    max_angular_velocity_auto_rads: float = 1.5

    # Odometry parameters
    wheelbase_m: float = 0.588625
    wheel_radius_m: float = 0.127
    encoder_ticks_per_rev: int = 1024

# Add to SnoBotConfig
@dataclass
class SnoBotConfig:
    sbcp: SBCPConfig
    sbvs: SBVSConfig
    sban: SBANConfig  # NEW
    logging: LoggingConfig
    metrics: MetricsConfig
```

#### Main Example Script (`main_autonomous.py`)

```python
async def main():
    # Initialize subsystems
    config = SnoBotConfig.load("config.yaml")
    control_loop = ControlLoop(config.sbcp)
    stereo = StereoProcessor(config.sbvs)
    objects = ObjectProcessor(config.sbvs)

    # Initialize autonomy
    odometry = OdometryEstimator(config.sban)
    mission_mgr = MissionManager()
    autonomy = AutonomyController(
        control_loop, stereo, objects, odometry, mission_mgr
    )

    # Teaching mode
    print("Starting teaching mode...")
    await autonomy.start_teaching()
    # ... user drives perimeter ...
    await autonomy.finish_teaching("driveway_001")

    # Planning
    print("Generating coverage path...")
    await autonomy.generate_plan()

    # Execution
    print("Starting autonomous execution...")
    await autonomy.execute_mission()
```

---

## Completed Work

### ✅ Phase 1: Encoder Integration & Odometry (COMPLETE)

**Files Created**:
1. ✅ [src/arduino/encoder_reader.h](../src/arduino/encoder_reader.h) - Encoder header
2. ✅ [src/arduino/encoder_reader.cpp](../src/arduino/encoder_reader.cpp) - Encoder implementation
3. ✅ [data/sban/robot_params.yaml](../data/sban/robot_params.yaml) - Robot parameters
4. ✅ [src/sban/localization/odometry.py](../src/sban/localization/odometry.py) - Odometry estimator
5. ✅ [src/sban/__init__.py](../src/sban/__init__.py) - SBAN module init
6. ✅ [src/sban/localization/__init__.py](../src/sban/localization/__init__.py) - Localization module init

**Files Modified**:
1. ✅ [src/arduino/system_test.ino](../src/arduino/system_test.ino)
   - Added encoder reader initialization
   - Added encoder/IMU telemetry fields
   - Removed old encoder ISRs

2. ✅ [src/sbcp_new/control_loop.py](../src/sbcp_new/control_loop.py)
   - Added `get_encoder_data()` method
   - Added `get_imu_data()` method

**Directories Created**:
```
src/sban/
  localization/ ✓
  perimeter/
  planning/
  navigation/
  state/
  integration/

data/sban/
  perimeters/
  missions/
  calibration/
  robot_params.yaml ✓
```

**Testing Status**: 🔶 Ready for hardware testing
- Arduino code compiles (needs verification)
- Python odometry class unit-testable
- Integration test needed with real encoders

---

## Next Steps

### Immediate (Phase 2)
1. Create perimeter data structures
2. Implement perimeter recorder
3. Implement perimeter storage with YAML
4. Test teaching mode with manual drive

### Medium-term (Phases 3-4)
5. Add `shapely` to requirements.txt
6. Implement scanline coverage planner
7. Create collision checker integrating SBVS
8. Test path generation and visualization

### Long-term (Phases 5-6)
9. Implement pure pursuit controller
10. Create mission state machine
11. Build main autonomy controller
12. Create example scripts and documentation

---

## Testing Plan

### Unit Testing

**Odometry Accuracy**:
```python
# Test straight line (10m)
# Expected drift: <10% (< 1m error)
encoder.reset()
drive_straight(10.0)
final_pose = odometry.get_pose()
assert abs(final_pose.x - 10.0) < 1.0
```

**Perimeter Recording**:
```python
# Test rectangle closure
# Drive 5m × 3m rectangle
# Expected closure error: < 1.0m
recorder.start_teaching()
# ... drive perimeter ...
perimeter = recorder.finish_teaching()
assert perimeter.is_closed(tolerance=1.0)
```

**Coverage Planning**:
```python
# Test scanline generation
polygon = [(0,0), (10,0), (10,5), (0,5)]
path = planner.plan(polygon, line_spacing=0.6)
assert path.total_length_m > 0
assert len(path.waypoints) > 0
```

### Integration Testing

1. **Teaching Mode**: Record simple rectangular perimeter (5m × 3m)
2. **Path Generation**: Generate scanline path, visualize in matplotlib
3. **Open Area Execution**: Run autonomous path following without obstacles
4. **Obstacle Avoidance**: Place static obstacle in path, verify stop behavior

### Full System Test

1. Mark perimeter in controlled area (parking lot)
2. Record perimeter via manual drive
3. Generate coverage plan
4. Execute autonomous mission with monitoring
5. Verify: path coverage, obstacle stops, behavior activation

**Success Criteria**:
- [ ] Odometry drift <10% over 50m path
- [ ] Perimeter closure detected within 1.0m
- [ ] Coverage path includes >90% of polygon area
- [ ] Obstacle stops triggered at 1.0m distance
- [ ] Pure pursuit tracking error <0.5m
- [ ] No collisions during 100m autonomous operation
- [ ] Mission state transitions work correctly

---

## Safety Considerations

1. **Emergency Stop**: Always respect SBCP E-stop and fault states
2. **Conservative Speeds**: Limit autonomous velocity to 1.0 m/s (vs 1.5 m/s manual max)
3. **Obstacle Response**: Hard stop if obstacle within 1.0m
4. **Manual Override**: User can always transition back to MANUAL mode
5. **Watchdog**: Existing 500ms motion watchdog continues to operate
6. **State Validation**: Verify SBCP state matches expected before sending commands

---

## Dependencies

**Python Libraries to Add**:
```
shapely>=2.0.0          # Polygon operations for path planning
scipy>=1.11.0           # Numerical operations (for future smoothing)
```

**Hardware Requirements**:
- ✅ Wheel encoders wired to Arduino interrupt pins (2, 4, 7, 8)
- ⚠️ IMU (BNO055) calibrated and reporting yaw (currently returns dummy data)
- ✅ Stereo cameras calibrated (calibration files exist)

---

## Future Enhancements (Out of Scope)

- Visual odometry for encoder-free operation
- Multi-zone behavior support within single perimeter
- Dynamic replanning around obstacles (currently stops)
- GPS integration for absolute positioning
- Mission progress UI/dashboard
- Spiral coverage pattern option
- Advanced Kalman filtering for odometry
- SLAM (Simultaneous Localization and Mapping)

---

## References

**Existing Systems Documentation**:
- SBVS: `src/sbvs/README.md` (if exists)
- SBCP: `data/sbcp/sbcp_0.3.0.yaml`

**Key Configuration Files**:
- Arduino: `src/arduino/config.h`
- Python: `src/common/common_config.py`
- Robot: `data/sban/robot_params.yaml`

**Algorithm References**:
- Pure Pursuit: Coulter, R. C. (1992). "Implementation of the Pure Pursuit Path Tracking Algorithm"
- Douglas-Peucker: Polygon simplification for reducing waypoint count
- Boustrophedon Decomposition: Coverage path planning for polygonal regions

---

## Contact & Collaboration

This planning session: 2026-01-25
Implementation status: Phase 1 of 6 complete
Next session: Continue with Phase 2 (Perimeter Recording)

To resume this work:
```bash
claude --continue
# or
claude --resume vision-integration
```
