# Gemini Project Context: FRC Team 9522 Robot (2026)

This document provides context for the Gemini AI assistant about the structure and conventions of this FIRST Robotics Competition (FRC) robot codebase.

## Project Overview

This is the Java codebase for FRC Team 9522's 2026 robot. The project is built using the WPILib Command-Based framework and is heavily augmented with modern libraries for advanced functionality, logging, and simulation.

- **Language:** Java 17
- **Build System:** GradleRIO
- **Robot Type:** Swerve Drive with a turreted shooter.

### Core Technologies & Libraries

- **[WPILib](https://docs.wpilib.org/):** The standard FRC robot framework. This project uses the Command-Based model.
- **[AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit):** A logging and simulation framework. All subsystems are built using the "IO Layer" pattern to support deterministic replay and analysis. Log files are the source of truth for debugging.
- **[YAGSL (Yet Another Generic Swerve Library)](https://github.com/BroncBotz3481/YAGSL):** The library used for the swerve drivetrain. Configuration is handled entirely through JSON files located in `src/main/deploy/swerve/`.
- **[PathPlanner](https://pathplanner.dev/):** Used for creating and executing complex autonomous routines. Path and auto files are located in `src/main/deploy/pathplanner/`.
- **[PhotonVision](https://photonvision.org/):** The vision system used for AprilTag detection and pose estimation, crucial for odometry correction.
- **[YAMS (Yet Another Motor-control and Simulation)](https://github.com/IronMaple/yams):** A library for abstracting motor control and simulation, used here for the shooter mechanism.
- **[MapleSim](https://github.com/IronMaple/maple-sim):** Provides a high-fidelity physics simulation environment, including a model of the 2026 game field (`Arena2026Rebuilt.java`).

### Robot Architecture

The robot is designed for a "pick-and-shoot" style game.

- **Drivetrain:** A 4-module swerve drive controlled by YAGSL.
- **Vision:** Utilizes PhotonVision on a coprocessor for AprilTag tracking to provide constant pose updates to the `SwerveSubsystem`.
- **Superstructure:** A coordinated set of mechanisms for game piece manipulation:
    - `IntakeSubsystem`: A ground-level intake to acquire game pieces (FUEL).
    - `HopperSubsystem` / `KickerSubsystem`: Internal mechanisms to store and transfer game pieces to the shooter.
    - `TurretSubsystem`: Rotates the shooter mechanism to aim.
    - `ShooterSubsystem`: A flywheel-based shooter.
    - `HoodSubsystem`: Adjusts the shooter's launch angle (currently configured as a fixed angle).
- **Auto-Aim:** The robot has sophisticated logic to aim automatically based on its position on the field, switching between different targets (`AimPoints` in `Constants.java`).

## Building and Running

This is a standard GradleRIO project. The primary build and deployment tasks are managed by the `gradlew` wrapper.

- **Build Code:**
  ```shell
  ./gradlew build
  ```
- **Run Simulation:** The project has extensive simulation support.
  ```shell
  ./gradlew simulateJava
  ```
  When the simulation starts, AdvantageScope and the WPILib Driver Station will typically launch, allowing for full visualization of the robot's state and 3D field visualization.
- **Deploy to Robot:**
  ```shell
  ./gradlew deploy
  ```

## Development Conventions

- **Constants:** All numerical values, CAN IDs, and tuning parameters are stored in `Constants.java`. This is the first place to look when modifying hardware configuration or tuning performance.
- **Subsystem Design (AdvantageKit IO Pattern):** Subsystems (`src/main/java/frc/robot/subsystems`) are split into an `IO` interface and one or more implementations (e.g., `VisionIO`, `VisionIOPhotonVision`, `VisionIOPhotonVisionSim`). This separates the subsystem logic from the hardware specifics, enabling robust simulation and replay. When modifying a subsystem, check if changes are needed in the logic class, the IO interface, and the hardware implementation class.
- **Configuration over Code:** Many parts of the robot are configured with files in the `src/main/deploy/` directory rather than hard-coded in Java.
    - **Swerve Drive:** `src/main/deploy/swerve/` (JSON files for modules, physical properties, PIDF, etc.)
    - **Autonomous:** `src/main/deploy/pathplanner/` (`.auto` and `.path` files)
- **Command-Based:** Actions are encapsulated in `Command` objects. Complex actions are composed of smaller commands. Controller bindings are declaratively defined in the `controls` package.
- **Data Logging:** AdvantageKit logs all inputs and outputs. To debug an issue, obtain a `.wpilog` file from the robot or a simulation run and open it in AdvantageScope. You can replay the entire match with full data visibility.
