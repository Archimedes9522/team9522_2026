// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Hood subsystem for adjusting shot trajectory.
 * 
 * <p>Supports two modes based on {@link HoodConstants#kHasMotor}:
 * <ul>
 *   <li><b>Motorized mode</b> ({@code kHasMotor = true}): Uses YAMS Pivot with a NEO 550
 *       motor and 50:1 reduction for active angle control (0° to 60°).</li>
 *   <li><b>Fixed mode</b> ({@code kHasMotor = false}): No physical motor — hood is a fixed
 *       polycarb piece. All commands are no-ops and {@link #getAngle()} always returns
 *       the fixed shooting angle. This avoids CAN bus timeouts from missing motors.</li>
 * </ul>
 * 
 * <p>Set {@link HoodConstants#kHasMotor} to {@code true} when a motorized hood is installed.
 */
public class HoodSubsystem extends SubsystemBase {

  // === HARDWARE (null when kHasMotor = false) ===
  private final SparkMax spark;

  // === YAMS CONTROLLER (null when kHasMotor = false) ===
  private final SmartMotorController motorController;
  private final Pivot hood;

  /**
   * Creates a new HoodSubsystem.
   * 
   * <p>If {@link HoodConstants#kHasMotor} is false, no hardware is initialized
   * and the hood reports a fixed angle.
   */
  public HoodSubsystem() {
    if (HoodConstants.kHasMotor) {
      // === MOTORIZED MODE ===
      spark = new SparkMax(HoodConstants.kMotorId, MotorType.kBrushless);

      SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              100, 0, 0,
              DegreesPerSecond.of(90),
              DegreesPerSecondPerSecond.of(90))
          .withFeedforward(new ArmFeedforward(0, 0.3, 0.1))
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(HoodConstants.kGearRatio)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withSoftLimit(
              Degrees.of(HoodConstants.kMinAngleDegrees),
              Degrees.of(HoodConstants.kMaxAngleDegrees))
          .withStatorCurrentLimit(Amps.of(HoodConstants.kCurrentLimitAmps))
          .withClosedLoopRampRate(Seconds.of(0.1))
          .withOpenLoopRampRate(Seconds.of(0.1));

      motorController = new SparkWrapper(spark, DCMotor.getNeo550(1), smcConfig);

      PivotConfig hoodConfig = new PivotConfig(motorController)
          .withHardLimit(
              Degrees.of(HoodConstants.kMinAngleDegrees - 5),
              Degrees.of(HoodConstants.kMaxAngleDegrees + 5))
          .withStartingPosition(Degrees.of(HoodConstants.kStowedAngleDegrees))
          .withMOI(KilogramSquareMeters.of(0.001))
          .withTelemetry("Hood", TelemetryVerbosity.HIGH);

      hood = new Pivot(hoodConfig);
    } else {
      // === FIXED MODE (no motor) ===
      System.out.println("[Hood] No motor configured (kHasMotor=false). Running in fixed-angle mode at "
          + HoodConstants.kFixedShootingAngle + "°");
      spark = null;
      motorController = null;
      hood = null;
    }
  }

  // ==================== COMMANDS ====================

  /**
   * Directly sets the hood target angle.
   * No-op in fixed mode.
   */
  public void setTargetAngle(Angle angle) {
    if (hood == null) return;
    double angleDeg = angle.in(Degrees);
    double clampedDeg = Math.max(HoodConstants.kMinAngleDegrees,
                                  Math.min(HoodConstants.kMaxAngleDegrees, angleDeg));
    motorController.setPosition(Degrees.of(clampedDeg));
  }

  /**
   * Sets the hood to a specific angle.
   * Returns an instant no-op command in fixed mode.
   */
  public Command setAngle(Angle angle) {
    if (hood == null) return Commands.none().withName("Hood.fixedAngle");
    return hood.setAngle(angle);
  }

  /**
   * Sets the hood to a dynamic angle from a supplier.
   * Returns an instant no-op command in fixed mode.
   */
  public Command setAngleDynamic(Supplier<Angle> hoodAngleSupplier) {
    if (hood == null) return Commands.none().withName("Hood.fixedAngle");
    return hood.setAngle(hoodAngleSupplier);
  }

  /**
   * Stows the hood to minimum angle.
   * No-op in fixed mode.
   */
  public Command stow() {
    return setAngle(Degrees.of(HoodConstants.kStowedAngleDegrees));
  }

  /**
   * Sets the hood to maximum angle.
   * No-op in fixed mode.
   */
  public Command max() {
    return setAngle(Degrees.of(HoodConstants.kMaxAngleDegrees));
  }

  /**
   * Sets the hood to open-loop duty cycle control.
   * No-op in fixed mode.
   */
  public Command set(double dutyCycle) {
    if (hood == null) return Commands.none().withName("Hood.fixedAngle");
    return hood.set(dutyCycle);
  }

  /**
   * Resets the hood encoder to zero.
   * No-op in fixed mode.
   */
  public Command rezero() {
    if (spark == null) return Commands.none().withName("Hood.noMotor");
    return Commands.runOnce(() -> spark.getEncoder().setPosition(0), this)
        .withName("Hood.Rezero");
  }

  /**
   * Runs system identification for tuning.
   * No-op in fixed mode.
   */
  public Command sysId() {
    if (hood == null) return Commands.none().withName("Hood.noMotor");
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  // ==================== GETTERS ====================

  /**
   * Gets the current hood angle.
   * In fixed mode, always returns the fixed shooting angle.
   */
  public Angle getAngle() {
    if (hood == null) return Degrees.of(HoodConstants.kFixedShootingAngle);
    return hood.getAngle();
  }

  /**
   * Checks if the hood is at the target angle (within tolerance).
   * In fixed mode, always returns true (hood is always "at angle").
   */
  public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
    if (hood == null) return true;
    double currentDegrees = getAngle().in(Degrees);
    return Math.abs(currentDegrees - targetDegrees) < toleranceDegrees;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    if (hood != null) {
      hood.updateTelemetry();
    }
    Logger.recordOutput("Hood/AngleDegrees", getAngle().in(Degrees));
    Logger.recordOutput("Hood/HasMotor", HoodConstants.kHasMotor);
  }

  @Override
  public void simulationPeriodic() {
    if (hood != null) {
      hood.simIterate();
    }
  }
}
