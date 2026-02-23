// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Shooter subsystem using dual NEO motors and YAMS FlyWheel.
 * 
 * <p>The shooter launches FUEL balls at the hub for scoring.
 * Uses closed-loop velocity control for consistent shot power.
 * 
 * <p>Hardware:
 * <ul>
 *   <li>2x NEO motors (leader + follower, inverted)</li>
 *   <li>4" diameter wheels</li>
 *   <li>1:1 gear ratio (direct drive)</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

  // === MOTORS ===
  private final SparkMax leaderSpark;
  private final SparkMax followerSpark;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final FlyWheel shooter;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    // Initialize motors
    leaderSpark = new SparkMax(ShooterConstants.kLeaderMotorId, MotorType.kBrushless);
    followerSpark = new SparkMax(ShooterConstants.kFollowerMotorId, MotorType.kBrushless);

    // Configure YAMS SmartMotorController
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withFollowers(Pair.of(followerSpark, true))  // Follower inverted
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            ShooterConstants.kP,
            ShooterConstants.kI,
            ShooterConstants.kD)
        .withFeedforward(new SimpleMotorFeedforward(
            ShooterConstants.kS,
            ShooterConstants.kV,
            ShooterConstants.kA))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(ShooterConstants.kGearRatio)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)  // Coast for flywheel momentum
        .withStatorCurrentLimit(Amps.of(ShooterConstants.kCurrentLimitAmps));

    motorController = new SparkWrapper(leaderSpark, DCMotor.getNEO(2), smcConfig);

    // Configure YAMS FlyWheel
    FlyWheelConfig shooterConfig = new FlyWheelConfig(motorController)
        .withDiameter(Inches.of(ShooterConstants.kWheelDiameterInches))
        .withMass(Pounds.of(1))
        .withUpperSoftLimit(RPM.of(ShooterConstants.kMaxSpeedRpm))
        .withLowerSoftLimit(RPM.of(0))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    shooter = new FlyWheel(shooterConfig);
  }

  // ==================== COMMANDS ====================

  /**
   * Directly sets the shooter target speed. Call this every loop when doing
   * dynamic shooting from a command that already requires this subsystem.
   * 
   * <p>This method directly controls the motor without creating a command,
   * so it can be called from within another command's execute() method.
   * 
   * @param speed Target angular velocity
   */
  public void setTargetSpeed(AngularVelocity speed) {
    // Log the commanded speed
    Logger.recordOutput("Shooter/CommandedSpeedRPM", speed.in(RPM));
    // Use direct motor control instead of scheduling a command to avoid conflicts
    motorController.setVelocity(speed);
  }

  /**
   * Sets the shooter to a specific speed.
   * 
   * @param speed Target angular velocity
   * @return Command that sets and maintains the speed
   */
  public Command setSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  /**
   * Sets the shooter to a dynamic speed from a supplier.
   * Useful for range-based shooting where speed changes with distance.
   * 
   * @param speedSupplier Supplier that provides target speed
   * @return Command that continuously updates speed
   */
  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return shooter.setSpeed(speedSupplier);
  }

  /**
   * Spins up the shooter to default shooting speed.
   * 
   * @return Command that spins up the flywheel
   */
  public Command spinUp() {
    return setSpeed(RPM.of(ShooterConstants.kShootingSpeedRpm));
  }

  /**
   * Stops the shooter.
   * 
   * @return Command that stops the flywheel
   */
  public Command stop() {
    return setSpeed(RPM.of(0));
  }

  /**
   * Runs system identification for feedforward tuning.
   * 
   * @return SysId command sequence
   */
  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }

  // ==================== GETTERS ====================

  /**
   * Gets the current shooter speed.
   * 
   * @return Current angular velocity
   */
  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  /**
   * Calculates the tangential velocity at the wheel edge.
   * Used for physics calculations (ball exit velocity).
   * 
   * @return Linear velocity at wheel surface
   */
  public LinearVelocity getTangentialVelocity() {
    Distance wheelRadius = Inches.of(ShooterConstants.kWheelDiameterInches / 2);
    return MetersPerSecond.of(
        getSpeed().in(RadiansPerSecond) * wheelRadius.in(Meters));
  }

  /**
   * Checks if the shooter is at the target speed (ready to shoot).
   * 
   * @param targetRpm Target speed in RPM
   * @return True if within tolerance of target
   */
  public boolean isAtSpeed(double targetRpm) {
    double currentRpm = getSpeed().in(RPM);
    return Math.abs(currentRpm - targetRpm) < ShooterConstants.kSpeedToleranceRpm;
  }

  /**
   * Checks if the shooter is spun up and ready to shoot.
   * Uses the default shooting speed.
   * 
   * @return True if ready to shoot
   */
  public boolean isReadyToShoot() {
    return isAtSpeed(ShooterConstants.kShootingSpeedRpm);
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    // Log motor velocities for debugging
    Logger.recordOutput("Shooter/LeaderVelocityRPM", leaderSpark.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/FollowerVelocityRPM", followerSpark.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/SpeedRPM", getSpeed().in(RPM));
    Logger.recordOutput("Shooter/ReadyToShoot", isReadyToShoot());
  }

  @Override
  public void simulationPeriodic() {
    // Update YAMS simulation
    shooter.simIterate();
  }
}
