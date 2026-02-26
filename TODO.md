# TODO — FRC Team 9522 (2026)

> **Last updated:** 2026-02-26  
> **Branch:** `main`

---

## 🔴 Critical — Before First Match

- [ ] **Calibrate swerve module encoder offsets**  
  Current offsets (FL=90°, FR=180°, BL=0°, BR=-90°) are placeholder values.  
  To calibrate: point all wheels straight forward, read absolute encoder values,  
  set `absoluteEncoderOffset` in each module JSON so they all read 0° when forward.

- [ ] **Set `kChassisOnly = false` when mechanisms are connected**  
  `Constants.java` currently has `kChassisOnly = true` to skip mechanism motor initialization.  
  This **must** be set to `false` before running with the full robot.

- [ ] **Verify robot mass**  
  `physicalproperties.json` and `Constants.java` both say 120 lbs (from CA26).  
  Weigh the actual 2026 robot and update `kRobotMassLbs` and `physicalproperties.json` `robotMass`.

- [ ] **Test PathPlanner auto paths on real robot**  
  AutoBuilder PID values (P=5.0 translation, P=5.0 rotation) are from CA26.  
  May need tuning for the new chassis — watch for oscillation or sluggish response.

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
  - Hood PID (P=0.1) — placeholder
  - Intake pivot PID (P=0.1) — placeholder

- [x] **Verify `invertedIMU` setting**  
  `swervedrive.json` has `invertedIMU: true`. Confirmed correct — gyro is mounted  
  facing backwards on the robot, so `invertedIMU: true` compensates for this.

- [ ] **Verify motor inversions**  
  All module JSONs have `drive: false, angle: true`.  
  If a module drives backward or turns the wrong way, adjust the `inverted` fields.

- [x] **Set up PathPlanner GUI settings**  
  Updated `deploy/pathplanner/settings.json` and YAGSL module JSONs with correct dimensions:  
  - Robot: 28" long × 26" wide (0.7112m × 0.6604m)  
  - Module positions: front=±12.25", left=±11.25" (wheelbase=24.5", trackwidth=22.5")  
  - PathPlanner modules: FL=0.31115,0.28575 / FR=0.31115,-0.28575 / BL=-0.31115,0.28575 / BR=-0.31115,-0.28575  
  - Robot mass: 54.43 kg (120 lbs), wheelCOF: 1.19, wheel radius: 0.0381m

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

---

## ✅ Completed

- [x] Switch from MAXSwerve template to YAGSL
- [x] Add AdvantageKit logging (LoggedRobot)
- [x] Add PhotonVision for AprilTag vision
- [x] Add MapleSim physics simulation
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
