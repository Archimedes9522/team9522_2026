# CLAUDE.md — FRC Team 9522 (2026)

## Project Overview

Java command-based robot code for **FRC Team 9522 (Archimedes)**, 2026 season. The robot is a **4-module swerve drive with a turreted shooter superstructure** designed for a pick-and-shoot game.

- **Language:** Java 17
- **Build system:** GradleRIO
- **Framework:** WPILib Command-Based

Build: `.\gradlew.bat build` | Simulate: `.\gradlew.bat simulateJava` | Deploy: `.\gradlew.bat deploy`

---

## Key Library Stack

| Library | Purpose |
|---|---|
| [WPILib](https://docs.wpilib.org/) | Command-based robot framework |
| [YAGSL](https://github.com/BroncBotz3481/YAGSL) | JSON-configured swerve drivetrain |
| [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) | Deterministic logging & log replay |
| [PathPlanner](https://pathplanner.dev/) | Autonomous path planning & execution |
| [PhotonVision](https://photonvision.org/) | AprilTag detection & vision pose fusion |
| [YAMS](https://github.com/IronMaple/yams) | Mechanism motor control & simulation abstraction |
| [MapleSim](https://github.com/IronMaple/maple-sim) | High-fidelity physics simulation (field, projectiles) |
| [REVLib](https://docs.revrobotics.com/) | SparkMax / SparkFlex motor controllers |

---

## Architecture

### Critical files

- **`Constants.java`** — Single source of truth for all CAN IDs, PID gains, gear ratios, field positions (`AimPoints`), and the `kChassisOnly` flag. **Never inline constants — always modify here.**
- **`RobotContainer.java`** — Creates subsystems and wires bindings. Mechanism subsystems are `null` when `kChassisOnly = true`; all downstream code must be null-safe.
- **`Superstructure.java`** — Coordinates all shooting mechanisms. High-level actions (aim, shoot, feed) go through Superstructure, not individual subsystems.
- **`AutoCommands.java`** — Centralized registration of all PathPlanner named commands via `AutoCommands.registerAll()`. Do not scatter named command registration elsewhere.
- **`ShootOnTheMoveCommand.java`** — Reads aim point + drivetrain pose/velocity, calculates turret angle, shooter RPM, and lead compensation. Uses `InterpolatingDoubleTreeMap` for distance→RPM and distance→TOF.

### Data flow

```
PhotonVision → VisionIO → Vision → SwerveSubsystem (pose fusion)
RobotContainer (zone triggers) → Superstructure.setAimPoint()
ShootOnTheMoveCommand reads aim point + drivetrain pose/velocity
  → calculates turret angle, shooter RPM, lead compensation
  → calls subsystem.setTargetAngle() / setTargetSpeed() directly
```

### Controls architecture

Controls are **static utility classes** (not subsystems): `DriverControls.configure()`, `OperatorControls.configure()`, `PoseControls`. They live in the `controls/` package.

### AdvantageKit IO pattern

Subsystems that touch hardware use a `IO interface + implementations` pattern:
- `VisionIO` (interface) → `VisionIOPhotonVision` (real) / `VisionIOPhotonVisionSim` (sim)
- This enables deterministic log replay. When editing a vision subsystem, check all three layers.

---

## Motor Controllers & Mechanisms

| Mechanism | Motor | Controller |
|---|---|---|
| Turret | NEO | SparkMax + REV Through Bore Encoder (absolute, no separate CAN ID) |
| Shooter (×2) | NEO | SparkMax |
| Kicker | NEO | SparkMax |
| Intake Pivot | NEO Vortex | SparkFlex |
| Intake Roller | NEO Vortex | SparkFlex |
| Hopper Indexer | NEO Vortex | SparkFlex |

All mechanisms use **YAMS** `Pivot` (positional) or `Flywheel` (velocity) wrappers with `SmartMotorController → SparkWrapper`. See `TurretSubsystem` for the canonical pattern.

**Hood no-op mode:** `HoodConstants.kHasMotor = false` skips all hardware init. `getAngle()` returns the fixed 55° constant and all commands return `Commands.none()`.

---

## Critical Gotchas

### CAN timeout risk
Every unconnected SparkMax/SparkFlex causes ~30s blocking timeout at startup. The `kChassisOnly` flag skips all mechanism init. **When adding a new motor, place it inside the `if (!Constants.kChassisOnly)` guard in RobotContainer.**

### Turret is mounted backwards
**Turret 0° = robot rear.** `ShootOnTheMoveCommand` adds 180° to convert robot-relative to turret-relative angles. `getRobotAdjustedAngle()` also adds 180°. Do not forget this offset.

### Command ownership
`ShootOnTheMoveCommand` calls `setTargetAngle()` / `setTargetSpeed()` directly on subsystems (not scheduling sub-commands) because it `addRequirements()` those subsystems. This prevents command scheduling conflicts.

### `.asProxy()` in Superstructure
When composing commands from different subsystems inside Superstructure, use `.asProxy()` to properly handle subsystem requirements inside parallel groups.

### Alliance-aware aim points
`AimPoints.getAllianceHubPosition()` returns the correct target based on `DriverStation.getAlliance()`. Zone triggers in RobotContainer auto-switch between hub/outpost/far-side targets.

### Lead compensation hardening
`ShootOnTheMoveCommand` uses distance clamping (`[2m, 12m]`), a low-speed lead deadband (0.1 m/s), and turret slew limiting (5 deg/cycle) to prevent jitter.

### Swerve config is JSON-driven
`src/main/deploy/swerve/` contains all YAGSL module configs. **Do not hardcode swerve parameters in Java.**

---

## Robot Physical Properties

- **Dimensions:** 28 in long × 26 in wide
- **Wheelbase/trackwidth:** 24.5 in / 22.5 in
- **Mass:** 112.39 lbs (50.98 kg) — update `kRobotMassKg` and `physicalproperties.json` after weighing
- **Wheel COF:** 1.19 (black rubber on carpet), synced in swerve and PathPlanner configs
- **Turret mounting:** 0° = robot rear (backwards mount)
- **IMU:** `invertedIMU: true` in `swervedrive.json` — gyro is mounted facing backwards

---

## Chassis-Only Mode

When testing with no mechanism motors connected, set `Constants.kChassisOnly = true` to skip mechanism init and avoid 5+ minute startup timeouts.

When `true`: all mechanism subsystems, Superstructure, operator controls, and zone triggers are disabled. Swerve, vision, and autos work normally.

**Set back to `false` before competing.**

---

## Project Structure

```
src/main/
├── deploy/
│   ├── swerve/                 # YAGSL JSON config (edit here, not in Java)
│   └── pathplanner/            # Auto & path files
└── java/frc/robot/
    ├── Constants.java          # All constants — edit here first
    ├── RobotContainer.java     # Subsystem wiring & bindings
    ├── commands/
    │   ├── ShootOnTheMoveCommand.java
    │   └── AutoCommands.java   # Named command registration
    ├── controls/               # Static controller binding classes
    ├── subsystems/
    │   ├── drive/SwerveSubsystem.java
    │   ├── mechanisms/         # Superstructure + individual mechanism subsystems
    │   └── vision/             # VisionIO + implementations
    └── util/maplesim/          # MapleSim integration
```

---

## Pending Work (as of 2026-03-18)

- [ ] Verify robot mass and update `kRobotMassKg` + `physicalproperties.json`
- [ ] Test and tune PathPlanner auto PID (translation P=5.0, rotation P=5.0)
- [ ] Tune swerve PIDF values on real robot (current values are YAGSL defaults)
- [ ] Run SysId characterization for shooter and swerve feedforward
- [ ] Tune mechanism PIDs on real hardware (Turret P=10 D=0.3, Shooter P=0.00936 from sim)
- [ ] Tune shooter RPM interpolation table with real-robot shot data
- [ ] Fine-tune auto paths in PathPlanner GUI — current waypoints are geometry-calculated

---

## Build Commands

```shell
.\gradlew.bat compileJava    # Fast compile check
.\gradlew.bat build          # Full build with checks
.\gradlew.bat simulateJava   # Launch simulation
.\gradlew.bat deploy         # Deploy to roboRIO
```

Vendordeps are in `vendordeps/`. Install new ones via WPILib VS Code extension → Manage Vendor Libraries.

---

## Logging & Debugging

- `Robot.java` extends `LoggedRobot`. Use `Logger.recordOutput("Subsystem/Key", value)` for telemetry.
- `.wpilog` files save to the roboRIO's USB drive.
- Open in [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) for full-match replay with complete data visibility.
- For vision debugging, use the IO-layer sim implementation to reproduce issues without hardware.
