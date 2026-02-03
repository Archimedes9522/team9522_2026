// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Superstructure coordinates all shooting mechanisms for unified control.
 * 
 * <p>This class manages the shooter, turret, hood, intake, hopper, and kicker
 * subsystems together for coordinated shooting operations.
 * 
 * <p>Key features:
 * <ul>
 *   <li>Readiness triggers for automated shooting</li>
 *   <li>Coordinated aim commands</li>
 *   <li>Feed-all commands for ball management</li>
 *   <li>Pose output for AdvantageScope visualization</li>
 * </ul>
 */
public class Superstructure extends SubsystemBase {

  // === SUBSYSTEM REFERENCES ===
  public final ShooterSubsystem shooter;
  public final TurretSubsystem turret;
  public final HoodSubsystem hood;
  public final IntakeSubsystem intake;
  public final HopperSubsystem hopper;
  public final KickerSubsystem kicker;

  // === TOLERANCE VALUES ===
  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle TURRET_TOLERANCE = Degrees.of(1);
  private static final Angle HOOD_TOLERANCE = Degrees.of(2);

  // === READINESS TRIGGERS ===
  private final Trigger isShooterAtSpeed;
  private final Trigger isTurretOnTarget;
  private final Trigger isHoodOnTarget;
  private final Trigger isReadyToShoot;

  // === TARGET SETPOINTS ===
  private AngularVelocity targetShooterSpeed = RPM.of(0);
  private Angle targetTurretAngle = Degrees.of(0);
  private Angle targetHoodAngle = Degrees.of(0);

  // === AIM POINT ===
  // Default to alliance-aware hub position (will be BLUE if on blue alliance)
  private Translation3d aimPoint = Constants.AimPoints.getAllianceHubPosition();

  /**
   * Creates a new Superstructure with all mechanism subsystems.
   * 
   * @param shooter The shooter subsystem
   * @param turret The turret subsystem
   * @param hood The hood subsystem
   * @param intake The intake subsystem
   * @param hopper The hopper subsystem
   * @param kicker The kicker subsystem
   */
  public Superstructure(
      ShooterSubsystem shooter,
      TurretSubsystem turret,
      HoodSubsystem hood,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      KickerSubsystem kicker) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.intake = intake;
    this.hopper = hopper;
    this.kicker = kicker;

    // Create triggers for readiness checks
    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) 
            < SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) 
            < TURRET_TOLERANCE.in(Degrees));

    this.isHoodOnTarget = new Trigger(
        () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) 
            < HOOD_TOLERANCE.in(Degrees));

    this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget).and(isHoodOnTarget);
  }

  // ==================== AIM COMMANDS ====================

  /**
   * Aims the superstructure to specific targets.
   * 
   * @param shooterSpeed Target shooter speed
   * @param turretAngle Target turret angle
   * @param hoodAngle Target hood angle
   * @return Command that aims all mechanisms
   */
  public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return Commands.runOnce(() -> {
      targetShooterSpeed = shooterSpeed;
      targetTurretAngle = turretAngle;
      targetHoodAngle = hoodAngle;
    }).andThen(
        Commands.parallel(
            shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy(),
            hood.setAngle(hoodAngle).asProxy()))
        .withName("Superstructure.aim");
  }

  /**
   * Aims the superstructure using dynamic suppliers.
   * Useful for motion-compensated shooting.
   * 
   * @param shooterSpeedSupplier Supplier for target shooter speed
   * @param turretAngleSupplier Supplier for target turret angle
   * @param hoodAngleSupplier Supplier for target hood angle
   * @return Command that continuously updates aim
   */
  public Command aimDynamicCommand(
      Supplier<AngularVelocity> shooterSpeedSupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier) {
    // CA26 pattern: Use .asProxy() to properly handle subsystem requirements
    // This is critical when the command is scheduled from ShootOnTheMoveCommand
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy(),
        hood.setAngleDynamic(hoodAngleSupplier).asProxy())
        .withName("Superstructure.aimDynamic");
  }

  /**
   * Sets target setpoints without running commands.
   * Useful for triggers that check readiness.
   */
  public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    targetShooterSpeed = shooterSpeed;
    targetTurretAngle = turretAngle;
    targetHoodAngle = hoodAngle;
  }

  /**
   * Command that continuously moves mechanisms to follow the target setpoints.
   * Use this alongside ShootOnTheMoveCommand which updates the setpoints.
   * 
   * @return Command that follows target setpoints for turret and hood
   */
  public Command followTargetSetpointsCommand() {
    return aimDynamicCommand(
        () -> targetShooterSpeed,
        () -> targetTurretAngle,
        () -> targetHoodAngle)
        .withName("Superstructure.followTargetSetpoints");
  }

  /**
   * Waits until all mechanisms are ready to shoot.
   * 
   * @return Command that waits for readiness
   */
  public Command waitUntilReadyCommand() {
    return Commands.waitUntil(isReadyToShoot).withName("Superstructure.waitUntilReady");
  }

  /**
   * Aims and waits until ready to shoot.
   */
  public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return aimCommand(shooterSpeed, turretAngle, hoodAngle)
        .andThen(waitUntilReadyCommand())
        .withName("Superstructure.aimAndWait");
  }

  // ==================== TURRET PRESETS ====================

  public Command setTurretForward() {
    return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
  }

  public Command setTurretLeft() {
    return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
  }

  public Command setTurretRight() {
    return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
  }

  // ==================== SHOOTING COMMANDS ====================

  /**
   * Spins up the shooter to default speed.
   */
  public Command shootCommand() {
    return shooter.spinUp().withName("Superstructure.shoot");
  }

  /**
   * Stops the shooter.
   */
  public Command stopShootingCommand() {
    return shooter.stop().withName("Superstructure.stopShooting");
  }

  /**
   * Stops all mechanisms.
   */
  public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stop().asProxy(),
        turret.set(0).asProxy(),
        hood.set(0).asProxy())
        .withName("Superstructure.stopAll");
  }

  // ==================== FEED COMMANDS ====================

  /**
   * Runs hopper and kicker to feed balls to shooter.
   */
  public Command feedAllCommand() {
    return Commands.parallel(
        hopper.feedCommand().asProxy(),
        kicker.feedCommand().asProxy())
        .withName("Superstructure.feedAll");
  }

  /**
   * Runs hopper and intake in reverse to unjam.
   */
  public Command backFeedAllCommand() {
    return Commands.parallel(
        hopper.reverseCommand().asProxy(),
        intake.deployAndEjectCommand().asProxy())
        .withName("Superstructure.backFeedAll");
  }

  /**
   * Stops all feeding mechanisms.
   */
  public Command stopFeedingAllCommand() {
    return Commands.parallel(
        hopper.stopCommand().asProxy(),
        kicker.stopCommand().asProxy())
        .withName("Superstructure.stopFeedingAll");
  }

  // ==================== INTAKE COMMANDS ====================

  /**
   * Runs the intake rollers.
   */
  public Command intakeCommand() {
    return intake.intakeCommand().withName("Superstructure.intake");
  }

  /**
   * Ejects from the intake.
   */
  public Command ejectCommand() {
    return intake.ejectCommand().withName("Superstructure.eject");
  }

  /**
   * Deploys intake and runs rollers.
   */
  public Command deployAndIntakeCommand() {
    return intake.deployAndRollCommand().withName("Superstructure.deployAndIntake");
  }

  /**
   * Deploys intake and runs rollers.
   * Alias for deployAndIntakeCommand() for compatibility.
   */
  public Command setIntakeDeployAndRoll() {
    return intake.deployAndRollCommand().withName("Superstructure.setIntakeDeployAndRoll");
  }

  // ==================== RE-ZERO COMMANDS ====================

  /**
   * Re-zeros the turret and intake pivot encoders.
   * Use when mechanisms are at their known zero positions.
   */
  public Command rezeroCommand() {
    return Commands.parallel(
        turret.rezero(),
        intake.rezero(),
        hood.rezero())
        .withName("Superstructure.rezero");
  }

  /**
   * Re-zeros the intake pivot and turret encoders.
   * Alias for rezeroCommand() for compatibility with OperatorControls.
   */
  public Command rezeroIntakePivotAndTurretCommand() {
    return rezeroCommand().withName("Superstructure.rezeroIntakePivotAndTurret");
  }

  // ==================== GETTERS ====================

  public AngularVelocity getShooterSpeed() {
    return shooter.getSpeed();
  }

  public Angle getTurretAngle() {
    return turret.getRawAngle();
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }

  public Angle getTargetTurretAngle() {
    return targetTurretAngle;
  }

  public Angle getTargetHoodAngle() {
    return targetHoodAngle;
  }

  public Translation3d getAimPoint() {
    return aimPoint;
  }

  public void setAimPoint(Translation3d newAimPoint) {
    this.aimPoint = newAimPoint;
  }

  /**
   * Updates the aim point to the current alliance's hub.
   * Call this when alliance changes or at the start of a match.
   */
  public void updateAimPointForAlliance() {
    this.aimPoint = Constants.AimPoints.getAllianceHubPosition();
  }

  /**
   * Gets the combined rotation of the turret and hood.
   * 
   * @return Rotation3d representing shooter orientation
   */
  public Rotation3d getAimRotation3d() {
    return new Rotation3d(
        Degrees.of(0),                      // no roll
        hood.getAngle().unaryMinus(),       // pitch is negative hood angle
        turret.getRobotAdjustedAngle());    // yaw from turret
  }

  /**
   * Gets the shooter pose for AdvantageScope visualization.
   * 
   * @return Pose3d of the shooter mechanism
   */
  public Pose3d getShooterPose() {
    return new Pose3d(
        new Translation3d(Meter.of(-0.3), Meter.of(0), Meter.of(0.6)),
        getAimRotation3d());
  }

  /**
   * Gets the tangential velocity of the shooter wheels.
   * 
   * @return Ball exit velocity
   */
  public LinearVelocity getTangentialVelocity() {
    return shooter.getTangentialVelocity();
  }

  /**
   * Gets the trigger for "ready to shoot" state.
   * 
   * @return Trigger that fires when all mechanisms are on target
   */
  public Trigger getReadyToShootTrigger() {
    return isReadyToShoot;
  }

  /**
   * Checks if the superstructure is ready to shoot.
   * 
   * @return True if shooter, turret, and hood are all on target
   */
  public boolean isReadyToShoot() {
    return isReadyToShoot.getAsBoolean();
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    // Log readiness state
    Logger.recordOutput("Superstructure/ShooterAtSpeed", isShooterAtSpeed.getAsBoolean());
    Logger.recordOutput("Superstructure/TurretOnTarget", isTurretOnTarget.getAsBoolean());
    Logger.recordOutput("Superstructure/HoodOnTarget", isHoodOnTarget.getAsBoolean());
    Logger.recordOutput("Superstructure/ReadyToShoot", isReadyToShoot.getAsBoolean());
    
    // Log targets - these are set by ShootOnTheMoveCommand
    Logger.recordOutput("Superstructure/TargetShooterRPM", targetShooterSpeed.in(RPM));
    Logger.recordOutput("Superstructure/TargetTurretAngle", targetTurretAngle.in(Degrees));
    Logger.recordOutput("Superstructure/TargetHoodDegrees", targetHoodAngle.in(Degrees));
    
    // Log actual turret position for comparison
    Logger.recordOutput("Superstructure/ActualTurretAngle", turret.getRawAngle().in(Degrees));
    
    // Log shooter pose for 3D visualization
    Logger.recordOutput("Superstructure/ShooterPose", getShooterPose());
  }
}
