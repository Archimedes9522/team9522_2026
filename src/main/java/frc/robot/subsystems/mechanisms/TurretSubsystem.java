// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Turret subsystem using YAMS Pivot for position control.
 * 
 * <p>The turret rotates the shooter to aim at the hub.
 * Has a ±90° field of view for targeting flexibility.
 * 
 * <p>Hardware:
 * <ul>
 *   <li>1x NEO motor</li>
 *   <li>40:1 gear reduction (4:1 gearbox × 10:1 pivot gearing)</li>
 *   <li>Non-continuous rotation (limited travel)</li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {

  /** Maximum rotation in one direction (degrees) */
  private static final double MAX_ONE_DIR_FOV = TurretConstants.kMaxAngleDegrees;
  
  /** Turret position relative to robot center (meters) */
  public static final Translation3d TURRET_TRANSLATION = new Translation3d(-0.205, 0.0, 0.375);

  // === MOTORS ===
  private final SparkMax spark;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final Pivot turret;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    // Initialize motor
    spark = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);

    // Configure YAMS SmartMotorController - CA26 exact PID values
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            15, 0, 0,  // CA26 exact: P=15, I=0, D=0
            DegreesPerSecond.of(2440),  // CA26 exact velocity
            DegreesPerSecondPerSecond.of(2440))  // CA26 exact acceleration
        .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))  // 40:1 total
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
        .withStatorCurrentLimit(Amps.of(TurretConstants.kCurrentLimitAmps))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    motorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    // Configure YAMS Pivot
    PivotConfig turretConfig = new PivotConfig(motorController)
        .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
        .withStartingPosition(Degrees.of(0))
        .withMOI(0.05)  // Moment of inertia for simulation
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(
            new MechanismPositionConfig()
                .withMovementPlane(Plane.XY)
                .withRelativePosition(TURRET_TRANSLATION));

    turret = new Pivot(turretConfig);
    
    // CA26 does NOT use a default command - removed to prevent conflicts with aimDynamicCommand
  }

  // ==================== COMMANDS ====================

  /**
   * Sets the turret to a specific angle.
   * 
   * @param angle Target angle (0 = forward, positive = clockwise)
   * @return Command that moves to the angle
   */
  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  /**
   * Sets the turret to a dynamic angle from a supplier.
   * Useful for auto-aiming where target angle changes.
   * 
   * @param turretAngleSupplier Supplier that provides target angle
   * @return Command that continuously updates angle
   */
  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

  /**
   * Centers the turret (returns to 0°).
   * 
   * @return Command that centers the turret
   */
  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  /**
   * Sets the turret to open-loop duty cycle control.
   * 
   * @param dutyCycle Motor output (-1 to 1)
   * @return Command that applies the duty cycle
   */
  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  /**
   * Resets the turret encoder to zero.
   * Use when turret is physically at the center position.
   * 
   * @return Command that resets the encoder
   */
  public Command rezero() {
    return Commands.runOnce(() -> spark.getEncoder().setPosition(0), this)
        .withName("Turret.Rezero");
  }

  /**
   * Runs system identification for tuning.
   * 
   * @return SysId command sequence
   */
  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  // ==================== GETTERS ====================

  /**
   * Gets the turret angle adjusted for robot coordinate frame.
   * Since the turret may be mounted backwards, this adds 180° offset.
   * 
   * @return Angle in robot frame
   */
  public Angle getRobotAdjustedAngle() {
    return turret.getAngle().plus(Degrees.of(180));
  }

  /**
   * Gets the raw turret angle without adjustment.
   * 
   * @return Raw angle from encoder
   */
  public Angle getRawAngle() {
    return turret.getAngle();
  }

  /**
   * Checks if the turret is at the target angle (within tolerance).
   * 
   * @param targetDegrees Target angle in degrees
   * @param toleranceDegrees Acceptable error in degrees
   * @return True if at target
   */
  public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
    double currentDegrees = getRawAngle().in(Degrees);
    return Math.abs(currentDegrees - targetDegrees) < toleranceDegrees;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    turret.updateTelemetry();

    // Log turret pose for AdvantageScope 3D visualization
    Logger.recordOutput("Turret/AngleDegrees", getRawAngle().in(Degrees));
    Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        new Pose3d(
            TURRET_TRANSLATION,
            new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    });
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
