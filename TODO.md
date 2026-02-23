# TODO — FRC Team 9522 (2026)

> **Last updated:** 2026-02-22  
> **Branch:** `test`

---

## 🔴 Critical — Before First Match

- [ ] **Verify pinion gear tooth count (12T vs 13T)**  
  `Constants.java` says `kDrivingMotorPinionTeeth = 13` and `physicalproperties.json` now uses `gearRatio: 5.08` (13T).  
  **Physically inspect** the MAXSwerve modules to confirm this is correct.  
  If the robot uses **12T**, change `physicalproperties.json` drive `gearRatio` to `5.50` and update `ModuleConstants.kDrivingMotorPinionTeeth` to `12`.  
  YAGSL standard factors: 12T=5.50, 13T=5.08, 14T=4.71

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

- [ ] **Verify `invertedIMU` setting**  
  `swervedrive.json` has `invertedIMU: true`. CA26 uses `false`.  
  If the robot spins the wrong way in teleop, toggle this.

- [ ] **Verify motor inversions**  
  All module JSONs have `drive: false, angle: true`.  
  If a module drives backward or turns the wrong way, adjust the `inverted` fields.

- [ ] **Set up PathPlanner GUI settings**  
  Verify `deploy/pathplanner/settings.json` has correct robot dimensions, mass, and  
  module positions matching the YAGSL config.

---

## 🟢 Nice to Have — Improvements

- [ ] **Add PathPlanner feedforward to AutoBuilder drive callback**  
  CA26 uses `swerveDrive.drive(speeds, moduleStates, moduleFeedForwards.linearForces())`  
  instead of just `swerveDrive.drive(speeds)`. This gives smoother autonomous paths.  
  Requires testing to ensure it works with our hardware.

- [ ] **Evaluate heading correction for aim modes**  
  `setHeadingCorrection(false)` is correct for normal driving but could be beneficial  
  for auto-aim modes where the robot should maintain a specific heading.

- [ ] **Remove unused old constants classes**  
  `DriveConstants`, `ModuleConstants`, `AutoConstants`, `NeoMotorConstants` are not used by YAGSL.  
  Once the team is fully comfortable with YAGSL config, these could be removed to  
  reduce confusion. Currently kept as reference/cross-check.

- [ ] **Add `wheelGripCoefficientOfFriction` tuning**  
  Currently 1.19 (standard black rubber on carpet). If using different wheels, update  
  `physicalproperties.json` and re-test autonomous accuracy.

- [ ] **Clean up YAMS `withMOI()` deprecation warnings**  
  `TurretSubsystem.java` and `HoodSubsystem.java` use deprecated `withMOI()`.  
  Check YAMS library for the replacement API when updating the dependency.

- [ ] **Consider PathPlanner setpoint generator**  
  CA26 imports `SwerveSetpointGenerator` for smoother path following.  
  Could be added later for competition-level autonomous smoothness.

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
