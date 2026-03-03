# Copilot Instructions - FRC Team 9522 (2026)

## Architecture

This is a **WPILib Command-Based** Java 17 robot with a YAGSL swerve drive and a turreted shooter superstructure. Build with `.\gradlew.bat build`, simulate with `.\gradlew.bat simulateJava`, deploy with `.\gradlew.bat deploy`.

### Key structural decisions

- **Constants.java** is the single source of truth for CAN IDs, PID gains, gear ratios, field positions (AimPoints), and the kChassisOnly flag. Always modify constants there, never inline.
- **Superstructure** (subsystems/mechanisms/Superstructure.java) coordinates all shooting mechanisms. Individual subsystems expose commands, but high-level actions (aim, shoot, feed) go through Superstructure.
- **RobotContainer** creates subsystems and wires bindings. Mechanism subsystems are null when kChassisOnly = true - all downstream code must be null-safe for those references.
- **Controls are static utility classes** in the controls/ package (DriverControls.configure(), OperatorControls.configure()). They are not subsystems.
- **Named commands for PathPlanner** are registered centrally in commands/AutoCommands.java via AutoCommands.registerAll(), not scattered in RobotContainer.

### Data flow

PhotonVision -> VisionIO -> Vision -> SwerveSubsystem (pose fusion)
RobotContainer (zone triggers) -> Superstructure.setAimPoint()
ShootOnTheMoveCommand reads aim point + drivetrain pose/velocity
  -> calculates turret angle, shooter RPM, lead compensation
  -> directly calls subsystem.setTargetAngle() / setTargetSpeed()

## Conventions and patterns

- **Motor controllers**: REV SparkFlex for NEO Vortex, SparkMax for NEO/NEO550. The turret uses SparkFlex + REV Through Bore Encoder (absolute, plugged into the SparkFlex data port, no separate CAN ID). Use DCMotor.getNeoVortex(1) or DCMotor.getNEO(1) in YAMS configs accordingly.
- **YAMS for mechanisms**: All mechanisms use YAMS Pivot (positional) or Flywheel (velocity) wrappers with SmartMotorController -> SparkWrapper. See TurretSubsystem for the canonical example.
- **Hood no-op mode**: HoodConstants.kHasMotor = false makes HoodSubsystem skip all hardware init. getAngle() returns the fixed 55 degree constant, all commands return Commands.none().
- **AdvantageKit logging**: Robot.java extends LoggedRobot. Use Logger.recordOutput("Subsystem/Key", value) for telemetry. Vision uses the IO-layer pattern (VisionIO interface -> VisionIOPhotonVision / VisionIOPhotonVisionSim).
- **Command ownership**: ShootOnTheMoveCommand directly calls setTargetAngle()/setTargetSpeed() on subsystems (not scheduling sub-commands) because it addRequirements() those subsystems. This avoids command-scheduling conflicts.
- **Swerve configuration is JSON-driven**: src/main/deploy/swerve/ contains all YAGSL module configs. Do not hardcode swerve parameters in Java.
- **Simulation**: MapleSim arena classes live in util/maplesim/. Arena2026Rebuilt models the field; RebuiltFuelOnFly handles projectile simulation.

## Robot physical properties

- Dimensions: 28 in long x 26 in wide (wheelbase 24.5 in, trackwidth 22.5 in)
- Mass: 120 lbs (54.43 kg), updated in Constants.kRobotMassKg and physicalproperties.json
- Wheel COF: 1.19 on carpet (black rubber), synced in swerve and PathPlanner configs
- Turret mounting: 0 degrees = robot rear (backwards mount). ShootOnTheMoveCommand adds 180 degrees to robot-relative angles to get turret-relative angles.

## What to watch for

- **CAN timeout risk**: Every unconnected SparkMax/SparkFlex causes ~30s blocking timeout at startup. The kChassisOnly flag exists specifically to skip mechanism init. If adding a new motor, add it inside the if (!Constants.kChassisOnly) guard in RobotContainer.
- **Turret is mounted backwards**: Turret 0 degrees = robot rear. ShootOnTheMoveCommand adds 180 degrees to convert robot-relative to turret-relative. getRobotAdjustedAngle() also adds 180 degrees.
- **Interpolating lookup tables** in ShootOnTheMoveCommand: distance->RPM and distance->TOF tables use InterpolatingDoubleTreeMap. Values beyond 5m are estimated. Distance is clamped to [2m, 12m].
- **.asProxy() on subsystem commands**: When composing commands from different subsystems inside Superstructure, .asProxy() is used to properly handle subsystem requirements in parallel groups.
- **Alliance-aware aim points**: AimPoints.getAllianceHubPosition() returns the correct target based on DriverStation.getAlliance(). Zone triggers in RobotContainer auto-switch between hub/outpost/far-side targets.
- **Lead compensation hardening**: ShootOnTheMoveCommand uses distance clamping, low-speed lead deadband (0.1 m/s), and turret slew limiting (5 deg/cycle) to prevent jitter.

## Build and test

.\gradlew.bat compileJava    # Fast compile check
.\gradlew.bat build           # Full build with checks
.\gradlew.bat simulateJava    # Launch simulation
.\gradlew.bat deploy          # Deploy to roboRIO

Vendordeps are in vendordeps/ (YAGSL, YAMS, AdvantageKit, PathPlanner, PhotonVision, REVLib, MapleSim, etc.). Install new ones via WPILib VS Code extension Manage Vendor Libraries.
