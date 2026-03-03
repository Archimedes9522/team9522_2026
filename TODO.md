# TODO — FRC Team 9522 (2026)

> **Last updated:** 2026-03-03  
> **Branch:** `main`  
> **Target:** First Qualifier (fixed launch angle, no adjustable hood)

---

## 🔴 Critical — Before First Match

- [x] **Calibrate swerve module encoder offsets**  
  ~~Current offsets (FL=90°, FR=180°, BL=0°, BR=-90°) are placeholder values.~~  
  Offsets calibrated, driving confirmed working.

- [x] **Set `kChassisOnly = false` when mechanisms are connected**  
  Set to `false` — all mechanism subsystems will initialize on startup.  
  If a specific motor is missing, add per-mechanism skip flags as needed.

- [ ] **Verify robot mass**  
  `physicalproperties.json` and `Constants.java` both say 120 lbs (from CA26).  
  Weigh the actual 2026 robot and update `kRobotMassLbs` and `physicalproperties.json` `robotMass`.

- [ ] **Test PathPlanner auto paths on real robot**  
  AutoBuilder PID values (P=5.0 translation, P=5.0 rotation) are from CA26.  
  May need tuning for the new chassis — watch for oscillation or sluggish response.

- [x] **Handle fixed launch angle (no hood motor)**  
  `HoodSubsystem` now checks `HoodConstants.kHasMotor`. When `false`, all commands  
  are no-ops, `getAngle()` returns the fixed 55°, and no CAN initialization occurs.  
  Set `kHasMotor = true` later when a motorized hood is added.

- [x] **Confirm motor types for all mechanisms**  
  All motor types updated:
  - Turret: NEO Vortex (SparkFlex, `DCMotor.getNeoVortex(1)`)
  - Shooter: 2× NEO (SparkMax, `DCMotor.getNEO(1)`)
  - Kicker: NEO (SparkMax)
  - Intake pivot: NEO Vortex recommended (SparkFlex)
  - Intake rollers: NEO (SparkMax)
  - Hopper: NEO Vortex recommended (SparkFlex)

---

## 🟡 Important — Before Competition

- [ ] **Tune swerve PIDF values**  
  `pidfproperties.json` has YAGSL default starting points for SparkMax (drive P=0.002, angle P=0.01).  
  These should be tuned on the actual robot using SmartDashboard/Shuffleboard PID tuning.  
  See: https://docs.yagsl.com/configuring-yagsl/how-to-tune-pidf

- [ ] **Run SysId characterization**  
  Feedforward values (kS, kV, kA) in `ShooterConstants` and the swerve drive have not been  
  characterized on this robot. Use WPILib SysId to get real values.  
  CA26 commented out: `replaceSwerveModuleFeedforward(0.268, 2.67, 0.23)` — this was their FF.

- [ ] **Tune mechanism PID values on real hardware**  
  - Turret PID (P=10, D=0.3) — tuned for sim, will need real-robot tuning
  - Shooter PID (P=0.00936) — tuned for sim
  - Intake pivot PID (P=0.1) — placeholder

- [x] **Verify `invertedIMU` setting**  
  `swervedrive.json` has `invertedIMU: true`. Confirmed correct — gyro is mounted  
  facing backwards on the robot, so `invertedIMU: true` compensates for this.

- [x] **Verify motor inversions**  
  All module JSONs have `drive: false, angle: true`.  
  If a module drives backward or turns the wrong way, adjust the `inverted` fields.

- [x] **Set up PathPlanner GUI settings**  
  Updated `deploy/pathplanner/settings.json` and YAGSL module JSONs with correct dimensions:  
  - Robot: 28" long × 26" wide (0.7112m × 0.6604m)  
  - Module positions: front=±12.25", left=±11.25" (wheelbase=24.5", trackwidth=22.5")  
  - PathPlanner modules: FL=0.31115,0.28575 / FR=0.31115,-0.28575 / BL=-0.31115,0.28575 / BR=-0.31115,-0.28575  
  - Robot mass: 54.43 kg (120 lbs), wheelCOF: 1.19, wheel radius: 0.0381m

- [ ] **Tune shooter speed interpolation table**  
  `ShootOnTheMoveCommand.java` has CA26-based RPM values (2m=2700, 3m=3000, 4m=3300, 4.86m=3750).  
  Extended estimates (6m=4200, 8m=4800, 10m=5200, 12m=5500) need real-robot tuning.  
  Distance clamped to [2m, 12m] range. Lead compensation has low-speed deadband (0.1 m/s).

- [x] **Create PathPlanner auto routines**  
  `AutoCommands.java` has all named commands registered. Created 13 `.auto` files  
  organized into 4 folders, plus 11 `.path` files in 3 folders:
  - **Shoot Only** (Top/Center/Bottom): Stand still, auto-aim, dump all 8 preloaded FUEL
  - **Taxi + Shoot** (Top/Center/Bottom): Dump 8 preloaded, then taxi past auto line
  - **Pickup** (Center, Bottom): Dump 8, drive to center FUEL, intake, return, shoot again
  - **Taxi Only** (Top/Center/Bottom): Simple taxi for mobility points, no shooting
  - New named commands: `dumpPreload` (4s feed), `autoDumpAll` (auto-aim + dump all 8)
  - **Field-aware paths**: All routes avoid hub ramp zones (47"×217" no-drive areas),
    pass through safe corridors (y>7.0 top, y<1.1 bottom), center taxi routes around.
  - **Turret heading**: Robot faces 0° (right) when shooting at blue hub so the
    rear-mounted turret (0°=rear) points toward the hub. Auto-aim handles fine adjustment.
  - **8 FUEL preload**: Game manual allows 8 preloaded FUEL; `autoDumpAll` empties all 8.

- [ ] **Fine-tune auto paths in PathPlanner GUI**  
  All paths have approximate waypoints based on field geometry calculations.  
  Open each path in PathPlanner GUI to verify obstacle clearance visually and  
  adjust control points for smooth curves. Key field measurements:
  - Blue hub: (4.60, 4.03), ramp zone: x=[4.00, 5.19], y=[1.28, 6.79]
  - Trench walls: (4.63, 1.43), (4.63, 6.64), (11.92, 1.43), (11.92, 6.64)
  - Center FUEL grid: x=[7.36, 9.19], y=[1.72, 6.10]
  - Tower pole (blue): (1.07, 4.04) — watch for collision near center start

---

## 🟢 Nice to Have — Improvements

- [x] **Add PathPlanner feedforward to AutoBuilder drive callback**  
  AutoBuilder now passes `feedforwards.linearForces()` and module states to  
  `swerveDrive.drive(speeds, moduleStates, forces)` for smoother autonomous paths.

- [x] **Evaluate heading correction for aim modes**  
  Added `setHeadingCorrection(boolean)` toggle to `SwerveSubsystem`.  
  Disabled by default for normal driving. Can be enabled for aim modes where  
  the robot should hold a specific heading while strafing.

- [x] **Remove unused old constants classes**  
  Removed `DriveConstants`, `ModuleConstants`, `NeoMotorConstants` from `Constants.java`.  
  Migrated `kMaxAngularSpeed` usage to `swerveDrive.getMaximumChassisAngularVelocity()`.  
  `AutoConstants` kept (actively used for PathPlanner PID and config).

- [x] **Add `wheelGripCoefficientOfFriction` tuning**  
  YAGSL `physicalproperties.json` has 1.19, PathPlanner `settings.json` now synced to 1.19.  
  Standard value for black rubber on carpet. Only update if using different wheel material.

- [x] **Clean up YAMS `withMOI()` deprecation warnings**  
  Updated to `withMOI(KilogramSquareMeters.of(value))` in `TurretSubsystem.java`  
  and `HoodSubsystem.java`. YAMS vendordep updated to 2026.2.23. Zero build warnings.

- [x] **Consider PathPlanner setpoint generator**  
  Added `SwerveSetpointGenerator` to `SwerveSubsystem` for smoother teleop driving.  
  New methods: `driveWithSetpoints(ChassisSpeeds)` and `driveFieldOrientedWithSetpoints(SwerveInputStream)`.  
  Generates kinematically feasible setpoints that prevent wheel scrub.

- [x] **Standardize swerve encoder config**  
  Module JSONs and PathPlanner settings synced. Unused constants classes removed.

---

## ✅ Completed

- [x] Switch from MAXSwerve template to YAGSL
- [x] Add AdvantageKit logging (LoggedRobot)
- [x] Add PhotonVision for AprilTag vision
- [x] Add MapleSim physics simulation
- [x] Calibrate swerve encoder offsets — driving confirmed working
- [x] Switch turret from CANcoder to REV Through Bore Encoder
- [x] Fix turret D-pad presets to ±90° (was ±45°)
- [x] Add PortForwarder for PhotonVision (port 5800)
- [x] Add YAMS for mechanism motor control
- [x] Add `kChassisOnly` mode for chassis-only testing
- [x] Add `setModuleEncoderAutoSynchronize` (YAGSL advanced feature)
- [x] Add `setAngularVelocityCompensation` (YAGSL advanced feature)
- [x] Add `setCosineCompensator` (YAGSL advanced feature)
- [x] Add `robotMass` to `physicalproperties.json`
- [x] Sync `AutoConstants` PID with actual PathPlanner config (P=5.0)
- [x] Cross-reference `ModuleConstants` with `physicalproperties.json`
- [x] Fix drive gear ratio to match 13T pinion (5.08 instead of 5.50)
- [x] Remove redundant `OIConstants` class
- [x] Fix `Command.schedule()` deprecation warnings
- [x] Add README.md
- [x] Verify and correct CAN IDs in module JSONs (FR=1,2 / FL=3,4 / BL=5,6 / BR=7,8)
- [x] Fix YAMS `withMOI()` deprecation — use `KilogramSquareMeters.of()` units API
- [x] Add PathPlanner feedforward to AutoBuilder drive callback
- [x] Add `setHeadingCorrection()` toggle for aim modes
- [x] Add `SwerveSetpointGenerator` for smoother teleop driving
- [x] Remove unused old constants classes (`DriveConstants`, `ModuleConstants`, `NeoMotorConstants`)
- [x] Fix PathPlanner GUI settings (module positions, mass, trackwidth, wheel COF/radius)
- [x] Update robot/module dimensions to actual measurements (28"×26" robot, 24.5"×22.5" module spacing)
- [x] Set `kChassisOnly = false` — mechanisms enabled
- [x] Handle fixed hood (no motor) — `HoodConstants.kHasMotor = false`, no-op mode
- [x] Confirm all motor types (Turret=Vortex/SparkFlex, Shooter=2×NEO, Kicker=NEO)
- [x] Update gear ratios (Turret 40:1, Intake pivot 53.33:1, Intake roller 4:1, Hopper 4:1, Kicker 3.2:1)
- [x] Update shooter RPM table from CA26 (2m=2700, 3m=3000, 4m=3300, 4.86m=3750)
- [x] Harden `ShootOnTheMoveCommand` (distance clamping, lead deadband, turret slew, error logging)
- [x] Fix turret javadoc (SparkFlex, not Spark MAX)
- [x] Create `AutoCommands.java` — centralized named command bank for PathPlanner autos
- [x] D-pad turret presets ±45° (matching CA26)
