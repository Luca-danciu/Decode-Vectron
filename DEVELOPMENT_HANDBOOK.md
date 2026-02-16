# Vectron Robotics – Development Handbook

**Team:** Vectron Robotics (17873)  
**Competition:** FIRST Tech Challenge (FTC) – DECODE 2025–2026  
**School:** Colegiul Național „Andrei Mureșanu”, Dej

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Architecture & Project Structure](#2-architecture--project-structure)
3. [Hardware Subsystems](#3-hardware-subsystems)
4. [Path Following (Pedro Pathing)](#4-path-following-pedro-pathing)
5. [OpModes](#5-opmodes)
6. [Hardware Tests](#6-hardware-tests)
7. [Hardware Configuration Reference](#7-hardware-configuration-reference)
8. [Development Workflow](#8-development-workflow)

---

## 1. Project Overview

This project is an FTC robot controller for the **DECODE** game. The robot:

- **Collects** artifacts (purple and green balls) from the field
- **Scores** artifacts into towers (Blue Tower / Red Tower) based on AprilTag-detected patterns (GPP, PGP, PPG)
- **Climbs** using an ascent mechanism
- Uses **path following** for autonomous and teleop parking

### Tech Stack

| Component | Technology |
|-----------|------------|
| SDK | FTC SDK 11.0 |
| Path Following | Pedro Pathing v2.0.2 |
| Vision | Limelight 3A, Vision Portal (AprilTag) |
| Localization | GoBilda Pinpoint (odometry pods) |
| Dashboard | Bylazar Full Panels v1.0.6 |

---

## 2. Architecture & Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Hardware/           # Robot subsystems (motors, servos, sensors)
├── OpModes/            # TeleOp & Autonomous programs
├── HardwareTests/      # Diagnostic and tuning OpModes
└── pedroPathing/       # Path following config & tuning
```

### Data Flow

```
OpMode (TeleOp/Auto)
    │
    ├─► Hardware (Drivetrain, Indexer, Outtake, etc.)
    │       └─► hardwareMap (motors, servos, sensors)
    │
    └─► Pedro Pathing Follower
            ├─► Constants (MecanumConstants, PinpointConstants)
            ├─► PathBuilder → PathChain → Path
            └─► Localization (GoBilda Pinpoint)
```

---

## 3. Hardware Subsystems

### 3.1 `Drivetrain`

**File:** `Hardware/Drivetrain.java`  
**Purpose:** Mecanum drive with 4 wheels.

| Motor | Config Name | Direction |
|-------|-------------|-----------|
| Front Left | FL | FORWARD |
| Front Right | FR | REVERSE |
| Rear Left | RL | FORWARD |
| Rear Right | RR | REVERSE |

**Methods:**
- `drivetraininit(HardwareMap)` – Initialize motors, brake mode, no encoders
- `drive(forward, strafe, turn)` – Mecanum drive with power normalization
- `activeBrake()` – Stop and hold position

**Notes:** Used for manual drive only. Pedro Pathing drives the same motors directly via its Follower.

---

### 3.2 `Indexer`

**File:** `Hardware/Indexer.java`  
**Purpose:** Intake, elevator, and pusher for balls.

| Device | Config Name | Type |
|--------|-------------|------|
| IndexerL | IndexerL | Servo |
| IndexerR | IndexerR | Servo |
| Banana | Banana | Servo |
| Intake | Intake | DcMotor |

**Methods:**
- **Intake:** `Colect()`, `KeepInside()`, `TakeOut()`, `TakeOutBit()`, `Stop()`
- **Pick poses (collect):** `PickPose1()` (0.025), `PickPose2()` (0.395), `PickPose3()` (0.785)
- **Outtake poses:** `OuttakePose1()` (0.21), `OuttakePose2()` (0.58), `OuttakePose3()` (0.97)
- **Pusher:** `Push()`, `PushBlue()`, `Down()`
- **Query:** `IndexerGetPos()` – Right indexer position

**Ball flow:** Ball enters at PickPose1 → PickPose2 → PickPose3 → stored. Outtake moves to scoring height (1–3), then `Push()` releases into launcher.

---

### 3.3 `Outtake`

**File:** `Hardware/Outtake.java`  
**Purpose:** Launcher motor and launch angle servo.

| Device | Config Name | Type |
|--------|-------------|------|
| Launcher | Launcher | DcMotor |
| Angle | Angle | Servo |

**Methods:**
- `Charge()` – Launcher at -0.8 power
- `StopLauncher()` – Launcher off
- `AngleMax()` – 0.83 (far shots)
- `AngleMin()` – 0.46 (close shots)

**Notes:** Launcher speed is controlled by PIDF in OpModes (target RPM). Angle is often set by distance to AprilTag.

---

### 3.4 `Ascent`

**File:** `Hardware/Ascent.java`  
**Purpose:** Climbing mechanism and hook.

| Device | Config Name | Type |
|--------|-------------|------|
| RidicareDJ | RidicareDJ | CRServo |
| RidicareDS | RidicareDS | CRServo |
| RidicareSJ | RidicareSJ | CRServo |
| RidicareSS | RidicareSS | CRServo |
| Prindere | Prindere | Servo |

**Methods:**
- `Ridicare(D, S)` – Set power on DJ/DS and SJ/SS
- `Prins()` – Hook closed (0)
- `Eliberat()` – Hook open (0.4)

---

### 3.5 `TurretSubsystem`

**File:** `Hardware/TurretSubsystem.java`  
**Purpose:** Turret that tracks a fixed field point using odometry.

**Config:** Motor `"Tureta"` (DcMotorEx)

**Key parameters:**
- GoBilda 435 RPM, gear ratio 6.533
- ~6.07 ticks/degree
- Software limit ±180°

**Constructor:** `TurretSubsystem(hwMap, follower, targetX, targetY, startingPose)`

**Methods:**
- `update()` – Call every loop; computes angle to target and runs turret
- `setTarget(x, y)` – Change target point
- `stop()` – Stop motor
- `getCurrentTurretAngle()` – For debugging

**Notes:** Uses Pedro Follower pose for robot heading. Most OpModes implement turret control manually (PIDF, Limelight) instead of this class.

---

### 3.6 `Limelight`

**File:** `Hardware/Limelight.java`  
**Purpose:** Limelight 3A for AprilTag detection and distance.

**Config:** `"limelight"` (Limelight3A), `"imu"` (IMU)

**Methods:**
- `getAprilTag()` – String: `"BT"`, `"GPP"`, `"PGP"`, `"PPG"`, `"RT"`, `"NONE"`
- `getAprilTagDistance()` – Approximate distance (inches) using `ta` and IMU

**AprilTag IDs:** 20=BT, 21=GPP, 22=PGP, 23=PPG, 24=RT

---

### 3.7 `WebcamTureta`

**File:** `Hardware/WebcamTureta.java`  
**Purpose:** Alternate vision using Vision Portal and AprilTag processor.

**Config:** Webcam `"WebcamTureta"`

**Methods:**
- `initwebcam(HardwareMap)` – Initialize vision portal
- `detectTag()` – Last detected tag name (PGP, PPG, GPP, Blue Tower, Red Tower)
- `getTagBearing()` – Bearing to Blue Tower (ID 20)
- `isTagFound()` – Whether Blue Tower tag is visible
- `getDistanceMeters()` – Last distance in meters
- `close()`, `startDetection()`, `stopDetection()`

---

### 3.8 `ColorSensorIndexer`

**File:** `Hardware/ColorSensorIndexer.java`  
**Purpose:** Color detection for purple vs green balls (DECODE).

**Config:** `"SenzorIntakeCH"` (NormalizedColorSensor) – often paired with a distance sensor.

**Enum:** `DetectedColor` – PURPLE, GREEN, UNKNOWN

**Methods:**
- `getDetectedColor()` – From normalized RGB thresholds
- `sendRGBTelemetry(Telemetry)` – Debug RGB values

**Logic:** Uses thresholds in normalized R/G/B to classify purple vs green for selective collection.

---

### 3.9 `IMUIndexer`

**File:** `Hardware/IMUIndexer.java`  
**Purpose:** IMU for robot heading.

**Config:** `"imu"` (IMU)

**Methods:**
- `getHeading(AngleUnit)` – Current yaw

**Notes:** Pedro Pathing uses its own localization. This IMU is mainly for Limelight and other sensor fusion.

---

### 3.10 `PIDController`

**File:** `Hardware/PIDController.java`  
**Purpose:** Simple PID controller.

**Constructor:** `PIDController(kP, kI, kD)`

**Methods:**
- `calculate(target, current)` – Returns control output

**Notes:** OpModes often implement custom PIDF (with feedforward) for launcher and turret instead of using this class.

---

## 4. Path Following (Pedro Pathing)

### 4.1 `Constants`

**File:** `pedroPathing/Constants.java`

Defines Pedro Pathing configuration:

| Constant | Role |
|----------|------|
| `driveConstants` | Mecanum motor names (FL, FR, RL, RR), directions, velocities |
| `localizerConstants` | Pinpoint pod positions, encoder config |
| `followerConstants` | Mass, zero-power acceleration, PIDF for translation/heading/drive |
| `pathConstraints` | Max velocity, acceleration |
| `createFollower(HardwareMap)` | Factory for `Follower` instance |

**Motor mapping:** FR, RR, RL, FL (right-front reversed, right-rear reversed).

---

### 4.2 `Tuning`

**File:** `pedroPathing/Tuning.java`

Menu of tuning OpModes via Bylazar SelectableOpMode:

- **Localization:** LocalizationTest, ForwardTuner, LateralTuner, TurnTuner
- **Automatic:** Velocity and zero-power acceleration tuners
- **Manual:** Translational, Heading, Drive, Centripetal tuners
- **Tests:** Line, Triangle, Circle paths

**Usage:** Run "Tuning" → pick sub-OpMode from menu → tune constants in Panels Dashboard.

---

## 5. OpModes

### 5.1 TeleOp OpModes

| OpMode | Purpose |
|--------|---------|
| `TeleOpBlue` | Main blue-side teleop |
| `TeleOpRed` | Main red-side teleop |
| `TeleOpBNGBlue` | Blue-side variant |

**Shared logic:**

- **Drive:** `follower.setTeleOpDrive()` or `drivetrain.drive()`
- **Turret:** Two modes – pose-based (tower coordinates) or Limelight tracking
- **Collect:** State machine (PickPose1→2→3) with distance sensor
- **Throw:** State machine (OuttakePose1→2→3 + Push + Down) with RPM wait
- **AprilTag case:** GPP, PGP, PPG used to choose indexer pose for scoring
- **Selective collect:** Color sensor + `greenBallPickedAt` for green ball placement
- **Park:** One-touch path to park pose

---

### 5.2 Autonomous OpModes

| OpMode | Side | Strategy |
|--------|------|----------|
| `AutoBlueSideTower` | Blue | 9-ball tower: score preload → 3 pickups → score each set |
| `AutoRedSideTower` | Red | Same pattern |
| `AutoBlueSideSmallTriangle` | Blue | Triangle path |
| `AutoRedSideSmallTriangle` | Red | Triangle path |
| `AutoRedSideOpenAndTake` | Red | Simplified open-and-take |
| `AutoBNGcomboBlue` | Blue | Blue-side combo |
| `BlueSide9sorted` | Blue | 9-ball with sorting |
| `RedSide9sorted` | Red | 9-ball with sorting |

**Shared pattern:**

1. `pathState` switch for high-level sequence
2. `stateThrow` for scoring sequence
3. `stateCollect` for intake
4. `buildPaths()` creates `PathChain`s via `follower.pathBuilder()`
5. `autonomousPathUpdate()` advances pathState based on `follower.isBusy()` and timers

---

### 5.3 State Machines

**Throw (stateThrow):** 0→1 (set pose, wait RPM)→2→3 (next pose)… →9 (done).

**Collect (stateCollect):** 0 (PickPose1) → 1 (PickPose2) → 2 (PickPose3) → 3/99 (done or selective).

**Selective collect (states 96–99):** Uses color sensor and `greenBallPickedAt` to choose correct scoring pose for each ball.

---

## 6. Hardware Tests

Located in `HardwareTests/` – used for debugging and tuning:

| OpMode | Purpose |
|--------|---------|
| `TuretaTestUltim` | Turret test |
| `TuretaTeleOp` | Manual turret control |
| `TestTeleOP` | Basic TeleOp |
| `TeleOpTestAruncarePidRPM` | Launcher RPM PID test |
| `RidicareTest` | Ascent test |
| `OuttakeTest` | Outtake test |
| `LimeLightAprilTagTest` | Limelight AprilTag |
| `LauncherPidTeleOp` | Launcher PID tuning |
| `IndexerPosition` | Indexer servo positions |
| `HybridAimTest` | Hybrid aiming |
| `AutoAimTurret` | Auto turret aiming |
| `ColorSensorTest` | Color sensor |
| `CameraTest` | Vision/camera |
| `BallColorPipeline` | Ball color pipeline |

---

## 7. Hardware Configuration Reference

Configure these in the Robot Configuration (Driver Hub / FTC app):

| Config Name | Type | Subsystem |
|-------------|------|-----------|
| FL | DcMotor | Drivetrain |
| FR | DcMotor | Drivetrain |
| RL | DcMotor | Drivetrain |
| RR | DcMotor | Drivetrain |
| Tureta | DcMotorEx | Turret |
| Launcher | DcMotor | Outtake |
| Intake | DcMotor | Indexer |
| IndexerL | Servo | Indexer |
| IndexerR | Servo | Indexer |
| Banana | Servo | Indexer |
| Angle | Servo | Outtake |
| Prindere | Servo | Ascent |
| RidicareDJ | CRServo | Ascent |
| RidicareDS | CRServo | Ascent |
| RidicareSJ | CRServo | Ascent |
| RidicareSS | CRServo | Ascent |
| SenzorIntakeCH | NormalizedColorSensor + DistanceSensor | Indexer |
| limelight | Limelight3A | Vision |
| imu | IMU | Localization |
| WebcamTureta | WebcamName | Vision (optional) |
| pinpoint | GoBilda Pinpoint | Localization |
| ledRedD, ledGreenD, ledRedS, ledGreenS | DigitalChannel | LEDs |

---

## 8. Development Workflow

### Building

1. Android Studio Ladybug (2024.2+) recommended
2. Sync Gradle, build APK
3. Deploy to Robot Controller phone

### Tuning Path Following

1. Run **Tuning** → choose Forward/Lateral/Turn tuners
2. Record multipliers and update `Constants.java`
3. Run **Drive Tuner**, **Heading Tuner** to tune PIDF
4. Use **Line**, **Triangle**, **Circle** to validate

### Adding a New OpMode

1. Create class in `OpModes/` or `HardwareTests/`
2. Annotate with `@TeleOp` or `@Autonomous`
3. In `init()`, create `Follower` via `Constants.createFollower(hardwareMap)`
4. In `loop()`, call `follower.update()` each cycle
5. Use `follower.followPath()`, `follower.setTeleOpDrive()`, etc.

### Adding Hardware

1. Add config in Robot Configuration
2. Create or extend a Hardware class
3. `hardwareMap.get(Type.class, "ConfigName")` in `init`
4. Use in OpMode logic

### Common Pitfalls

- **Turret ticks:** Wrap at ±180° (MAX_TICKS)
- **Launcher RPM:** Use measured RPM in a feedback loop; avoid open-loop only
- **Path timing:** Wait for `!follower.isBusy()` before starting next action
- **AprilTag detection:** Check `result.isValid()` and non-empty fiducials
- **State machines:** Use timers to prevent too-fast transitions

---

## Appendix: Game-Specific Terminology

| Term | Meaning |
|------|---------|
| GPP, PGP, PPG | AprilTag patterns on obelisk (green/purple order) |
| BT, RT | Blue Tower, Red Tower (AprilTag IDs 20, 24) |
| PickPose1/2/3 | Indexer heights for collecting |
| OuttakePose1/2/3 | Indexer heights for scoring |
| Selective collection | Collect by color and score in correct tower slot |
| Tower X, Y | Fixed coordinates used for turret aiming |

---

*Last updated: February 2026*
