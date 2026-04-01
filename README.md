# FRC Team 9522 — 2026 Robot Code

[![WPILib](https://img.shields.io/badge/WPILib-2026-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://openjdk.org/)
[![YAGSL](https://img.shields.io/badge/Swerve-YAGSL-green)](https://github.com/BroncBotz3481/YAGSL)

> **FRC Team 9522 — Archimedes**  
> 2026 Season: *REEFSCAPE / Rebuilt*

Java command-based robot code for a **swerve-drive robot with a turreted shooter**, built on the WPILib Command-Based framework and augmented with a modern stack of FRC libraries for advanced autonomous navigation, vision-based pose estimation, full-state logging, and physics simulation.

---

## Table of Contents

- [Technologies](#technologies)
- [Key Features](#key-features)
- [Project Structure](#project-structure)
- [Building & Deploying](#building--deploying)
- [Chassis-Only Mode](#chassis-only-mode)
- [Configuration](#configuration)
- [Swerve Drive (YAGSL)](#swerve-drive-yagsl)
- [Vision (PhotonVision)](#vision-photonvision)
- [Autonomous (PathPlanner)](#autonomous-pathplanner)
- [Logging & Replay (AdvantageKit)](#logging--replay-advantagekit)
- [Simulation](#simulation)
- [Development Conventions](#development-conventions)

---

## Technologies

| Library | Purpose |
|---|---|
| [WPILib](https://docs.wpilib.org/) | Command-based robot framework |
| [YAGSL](https://github.com/BroncBotz3481/YAGSL) | JSON-configured swerve drive |
| [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) | Deterministic logging & replay |
| [PathPlanner](https://pathplanner.dev/) | Autonomous path planning & execution |
| [PhotonVision](https://photonvision.org/) | AprilTag detection & vision pose estimation |
| [YAMS](https://github.com/IronMaple/yams) | Motor control & mechanism simulation |
| [MapleSim](https://github.com/IronMaple/maple-sim) | High-fidelity physics simulation |
| [REVLib](https://docs.revrobotics.com/) | REV SparkMax / SparkFlex motor controllers |
| [Redux (Canandcoder)](https://docs.reduxrobotics.com/) | Absolute encoders |
| [Studica (NavX)](https://www.studica.com/navx) | NavX-MXP gyroscope (via SPI) |

---

## Key Features

- **Vernier Absolute Encoders (Turret):** Utilizes dual absolute encoders (19t and 21t gears) and the Chinese Remainder Theorem (CRT) via `EasyCRT` to provide infinite-resolution absolute tracking of the turret without relying on hard stops or manual rezeroing mid-match.
- **Addressable LED Feedback:** A dedicated `LEDSubsystem` drives WS2812B LEDs to provide the drive team with instant visual feedback (e.g., solid orange for game piece acquired, fast blinking green for target locked).
- **Odometry-Based Pre-Spin:** Automatically idles the shooter flywheel to 3000 RPM as soon as the robot crosses into its offensive zone, dropping time-to-shoot to near-zero.
- **Zone-Based Auto-Aim:** Dynamically selects the best target (Hub, Outpost, or Far Side) depending on the robot's real-time field odometry.
- **Seamless Hardware/Simulation Parity:** Run the exact same code on the physical robot and within MapleSim's high-fidelity rigid-body physics engine.

---

## Project Structure

```
src/main/
├── deploy/
│   ├── swerve/                 # YAGSL JSON configuration
│   │   ├── swervedrive.json    #   Drive-wide settings & gyro config
│   │   ├── controllerproperties.json
│   │   └── modules/            #   Per-module motor & encoder config
│   └── pathplanner/            # PathPlanner autos & paths
└── java/frc/robot/
    ├── Main.java               # Entry point
    ├── Robot.java              # Robot lifecycle (AdvantageKit LoggedRobot)
    ├── RobotContainer.java     # Subsystem creation, command bindings
    ├── Constants.java          # All robot-wide constants & CAN IDs
    ├── FieldConstants.java     # Named AimPoints & field geometry
    ├── BuildConstants.java     # Auto-generated build metadata
    ├── commands/
    │   └── mechanisms/ShootOnTheMoveCommand.java
    ├── controls/
    │   ├── DriverControls.java     # Driver Xbox controller bindings
    │   ├── OperatorControls.java   # Operator Xbox controller bindings
    │   └── PoseControls.java       # Virtual target pose controller
    ├── subsystems/
    │   ├── drive/
    │   │   └── SwerveSubsystem.java    # YAGSL swerve wrapper
    │   ├── mechanisms/
    │   │   ├── Superstructure.java     # Coordinated mechanism manager
    │   │   ├── ShooterSubsystem.java   # Flywheel shooter
    │   │   ├── TurretSubsystem.java    # Vernier CRT Turret rotation
    │   │   ├── HoodSubsystem.java      # Launch angle adjustment
    │   │   ├── IntakeSubsystem.java    # Ground intake
    │   │   ├── HopperSubsystem.java    # Game piece storage/transfer
    │   │   ├── KickerSubsystem.java    # Feeds game pieces to shooter
    │   │   └── LEDSubsystem.java       # Addressable WS2812B feedback
    │   └── vision/
    │       ├── Vision.java
    │       ├── VisionConstants.java
    │       ├── VisionIO.java
    │       ├── VisionIOPhotonVision.java
    │       └── VisionIOPhotonVisionSim.java
    └── util/
        ├── CommandsLogging.java
        ├── LocalADStarAK.java
        └── maplesim/              # MapleSim integration utilities
```

---

## Building & Deploying

This is a standard [GradleRIO](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/deploying-robot-code.html) project. Use the included Gradle wrapper — no separate Gradle installation is required.

### Build
```shell
./gradlew build        # Linux / macOS
.\gradlew.bat build    # Windows
```

### Deploy to Robot
Connect to the robot's network, then:
```shell
./gradlew deploy
```

### Simulate
```shell
./gradlew simulateJava
```
The simulation launches the WPILib Driver Station and supports full AdvantageScope visualization, including 3D field rendering via MapleSim.

---

## Chassis-Only Mode

When testing with **only the swerve chassis** (no mechanism motors connected), robot startup can take 5+ minutes as each missing SparkMax times out on the CAN bus.

To skip mechanism initialization and get a fast startup:
1. Open `src/main/java/frc/robot/Constants.java`
2. Set the flag at the top of the file: `public static final boolean kChassisOnly = true;`
3. Build and deploy

When `kChassisOnly` is `true`:
- All mechanism subsystems, the Superstructure, and LEDs are **not created**.
- Operator controls for mechanisms are **disabled**.
- Zone-based auto-aim triggers are **disabled**.
- The swerve drive, vision, and autonomous paths work normally.

> **Remember to set `kChassisOnly = false` before competing with the full robot!**

---

## Configuration

### Constants
All numerical values, CAN IDs, PID tuning parameters, and field positions live in [`Constants.java`](src/main/java/frc/robot/Constants.java). Key sections include `ControllerConstants`, `ShooterConstants`, `TurretConstants`, etc. `FieldConstants.java` holds named `AimPoints`.

### Swerve Drive (YAGSL)
The swerve drive is configured entirely through JSON files in `src/main/deploy/swerve/`:

- **`swervedrive.json`** — Drive-wide settings: max speed, gyro type (NavX via MXP SPI), IMU orientation, module locations
- **`controllerproperties.json`** — Heading correction PID, drive PID
- **`modules/`** — Per-module configuration (drive motor, angle motor, absolute encoder, gear ratios, encoder offsets)

Module offsets are set in degrees in each module's JSON file (`absoluteEncoderOffset`). To calibrate:
1. Point all wheels straight forward
2. Read the absolute encoder values
3. Set the offsets so each module reads 0° when wheels face forward

### Vision (PhotonVision)
Camera configuration is in `VisionConstants.java`. The robot uses a PhotonVision coprocessor for AprilTag detection, providing field-relative pose estimates that are fused with wheel odometry in the swerve subsystem.

### Autonomous (PathPlanner)
Autonomous routines are created in the PathPlanner GUI and stored in `src/main/deploy/pathplanner/`. Named commands are registered in `RobotContainer` and executed during autonomous via PathPlanner's auto builder.

---

## Logging & Replay (AdvantageKit)

The robot uses [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) for deterministic logging:
- `Robot.java` extends `LoggedRobot` instead of `TimedRobot`
- All subsystem inputs/outputs are logged every cycle
- `.wpilog` files are saved to the roboRIO's USB drive (Format USB as FAT32, logs go to `/U/logs/`)
- Open log files in [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) for full-match replay with complete data visibility. Look for values like `Superstructure/ShooterPose` or `Turret/AbsoluteAngleDeg`.

Vision subsystems follow the **IO Layer pattern**: a `VisionIO` interface with separate real (`VisionIOPhotonVision`) and sim (`VisionIOPhotonVisionSim`) implementations, enabling seamless switching between hardware and simulation.

---

## Simulation

Extensive simulation support powered by **MapleSim**:
- Full swerve drive rigid-body physics simulation
- Mechanism motor simulation via YAMS
- 2026 game field model (`Arena2026Rebuilt`)
- PhotonVision camera simulation (`VisionIOPhotonVisionSim`)
- AdvantageScope 3D visualization

---

## Development Conventions

- **Command-Based Architecture** — All robot actions are encapsulated as `Command` objects, composed declaratively
- **AdvantageKit IO Pattern** — Subsystems separate logic from hardware via IO interfaces
- **Configuration over Code** — Swerve, autos, and field data use config files rather than hardcoded values
- **Constants-First** — All tunable values live in `Constants.java`
- **Controller Separation** — Driver, operator, and pose controls are in dedicated classes in the `controls` package
- **Null-Safe Mechanisms** — When `kChassisOnly` is enabled, all mechanism references are safely null-checked throughout the codebase

---

## License

This project is released under the [WPILib BSD License](WPILib-License.md).
