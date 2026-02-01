// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
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
 * Hood subsystem using YAMS Pivot for position control.
 * 
 * <p>The hood adjusts the shooter angle to control ball trajectory.
 * Higher angles for close shots, lower angles for far shots.
 * 
 * <p>Hardware:
 * <ul>
 *   <li>1x NEO 550 motor</li>
 *   <li>50:1 gear reduction</li>
 *   <li>0° to 60° travel range</li>
 * </ul>
 */
public class HoodSubsystem extends SubsystemBase {

  // === MOTORS ===
  private final SparkMax spark;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final Pivot hood;

  /**
   * Creates a new HoodSubsystem.
   */
  public HoodSubsystem() {
    // Initialize motor (NEO 550 for smaller mechanism)
    spark = new SparkMax(HoodConstants.kMotorId, MotorType.kBrushless);

    // Configure YAMS SmartMotorController
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            100, 0, 0,
            DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0, 0.3, 0.1))  // Gravity compensation for arm
        .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(HoodConstants.kGearRatio)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)  // Brake to hold position
        .withSoftLimit(
            Degrees.of(HoodConstants.kMinAngleDegrees),
            Degrees.of(HoodConstants.kMaxAngleDegrees))
        .withStatorCurrentLimit(Amps.of(HoodConstants.kCurrentLimitAmps))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    motorController = new SparkWrapper(spark, DCMotor.getNeo550(1), smcConfig);

    // Configure YAMS Pivot
    PivotConfig hoodConfig = new PivotConfig(motorController)
        .withHardLimit(
            Degrees.of(HoodConstants.kMinAngleDegrees - 5),
            Degrees.of(HoodConstants.kMaxAngleDegrees + 5))
        .withStartingPosition(Degrees.of(HoodConstants.kStowedAngleDegrees))
        .withMOI(0.001)  // Small moment of inertia
        .withTelemetry("Hood", TelemetryVerbosity.HIGH);

    hood = new Pivot(hoodConfig);
  }

  // ==================== COMMANDS ====================

  /**
   * Sets the hood to a specific angle.
   * 
   * @param angle Target angle (0° = flat, higher = steeper)
   * @return Command that moves to the angle
   */
  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  /**
   * Sets the hood to a dynamic angle from a supplier.
   * Useful for range-based shooting where angle changes with distance.
   * 
   * @param hoodAngleSupplier Supplier that provides target angle
   * @return Command that continuously updates angle
   */
  public Command setAngleDynamic(Supplier<Angle> hoodAngleSupplier) {
    return hood.setAngle(hoodAngleSupplier);
  }

  /**
   * Stows the hood to minimum angle.
   * 
   * @return Command that stows the hood
   */
  public Command stow() {
    return setAngle(Degrees.of(HoodConstants.kStowedAngleDegrees));
  }

  /**
   * Sets the hood to maximum angle.
   * 
   * @return Command that moves to max angle
   */
  public Command max() {
    return setAngle(Degrees.of(HoodConstants.kMaxAngleDegrees));
  }

  /**
   * Sets the hood to open-loop duty cycle control.
   * 
   * @param dutyCycle Motor output (-1 to 1)
   * @return Command that applies the duty cycle
   */
  public Command set(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  /**
   * Resets the hood encoder to zero.
   * Use when hood is physically at the minimum position.
   * 
   * @return Command that resets the encoder
   */
  public Command rezero() {
    return Commands.runOnce(() -> spark.getEncoder().setPosition(0), this)
        .withName("Hood.Rezero");
  }

  /**
   * Runs system identification for tuning.
   * 
   * @return SysId command sequence
   */
  public Command sysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  // ==================== GETTERS ====================

  /**
   * Gets the current hood angle.
   * 
   * @return Current angle
   */
  public Angle getAngle() {
    return hood.getAngle();
  }

  /**
   * Checks if the hood is at the target angle (within tolerance).
   * 
   * @param targetDegrees Target angle in degrees
   * @param toleranceDegrees Acceptable error in degrees
   * @return True if at target
   */
  public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
    double currentDegrees = getAngle().in(Degrees);
    return Math.abs(currentDegrees - targetDegrees) < toleranceDegrees;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    hood.updateTelemetry();
    Logger.recordOutput("Hood/AngleDegrees", getAngle().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
