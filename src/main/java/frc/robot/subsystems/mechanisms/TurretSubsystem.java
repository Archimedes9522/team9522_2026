// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
 *   <li>1x NEO motor (SparkMax)</li>
 *   <li>1x REV Through Bore Encoder for absolute position sensing (plugged into SparkMax data port)</li>
 *   <li>40:1 gear reduction (4:1 REV Sport Gearbox × 200:20 gear transmission)</li>
 *   <li>Through Bore on 1:10 overdrive from pinion = 1:1 with turret rotation</li>
 *   <li>Non-continuous rotation (limited travel)</li>
 * </ul>
 * 
 * <p>The REV Through Bore Encoder provides absolute position so the turret knows its true
 * position on startup without needing to be homed/zeroed.
 */
public class TurretSubsystem extends SubsystemBase {

  /** Maximum rotation in one direction (degrees) */
  private static final double MAX_ONE_DIR_FOV = TurretConstants.kMaxAngleDegrees;
  
  /** Turret position relative to robot center (meters) */
  public static final Translation3d TURRET_TRANSLATION = new Translation3d(-0.205, 0.0, 0.375);
  
  /** 
   * Through Bore Encoder zero offset in rotations (0.0 - 1.0).
   * Calibrate this so that the encoder reads 0 when the turret is physically centered.
   * To calibrate: physically center turret, read absolute encoder position, 
   * set this value to what you read so it gets subtracted out.
   */
  private static final double THROUGH_BORE_ZERO_OFFSET = 0.0;

  // === HARDWARE ===
  private final SparkMax spark;
  private final SparkAbsoluteEncoder throughBoreEncoder;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final Pivot turret;
  
  /** Flag to track if we've synced the NEO encoder with the absolute encoder */
  private boolean hasBeenSynced = false;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    // Initialize motor (NEO uses SparkMax)
    spark = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);
    
    // Get the REV Through Bore Encoder from the SparkMax data port
    // The Through Bore is plugged directly into the SparkMax's absolute encoder port
    throughBoreEncoder = spark.getAbsoluteEncoder();

    // Configure YAMS SmartMotorController
    // CA26 uses: P=15, I=0, D=0, velocity=2440, accel=2440, ramp=0.1, kV=7.5
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(
        TurretConstants.kP,  // P for tracking
        TurretConstants.kI,  // I=0.0
        TurretConstants.kD,  // D for damping
        DegreesPerSecond.of(2440),  // Max velocity - matching CA26
        DegreesPerSecondPerSecond.of(2440))  // Max acceleration - matching CA26
          .withFeedforward(new SimpleMotorFeedforward(0, 1.5, 0))  // kV=1.5 feedforward
          .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))  // 40:1 total
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)  // CA26 uses COAST
          .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
          .withStatorCurrentLimit(Amps.of(40))  // Increased from 10A for faster response
          .withClosedLoopRampRate(Seconds.of(0))  // Remove ramp for faster response
          .withOpenLoopRampRate(Seconds.of(0));

    motorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    // Configure YAMS Pivot - MOI matching CA26 (0.05)
    // Get starting position from Through Bore Encoder if on real robot
    Angle startingPosition = Degrees.of(0);
    if (Constants.currentMode == Constants.Mode.REAL) {
      startingPosition = getThroughBoreAbsolutePosition();
    }
    
    PivotConfig turretConfig = new PivotConfig(motorController)
        .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
        .withStartingPosition(startingPosition)
        .withMOI(KilogramSquareMeters.of(0.05))  // Moment of inertia matching CA26
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(
            new MechanismPositionConfig()
                .withMovementPlane(Plane.XY)
                .withRelativePosition(TURRET_TRANSLATION));

    turret = new Pivot(turretConfig);
    
    // Sync NEO encoder with Through Bore Encoder absolute position on startup
    if (Constants.currentMode == Constants.Mode.REAL) {
      syncEncoderWithThroughBore();
    }
    
    // CA26 does NOT use a default command - removed to prevent conflicts with aimDynamicCommand
  }
  
  /**
   * Gets the absolute position from the REV Through Bore Encoder.
   * The encoder is on a 1:10 overdrive from the pinion, giving 1:1 with turret rotation.
   * This survives power cycles and always knows the true turret position.
   * 
   * <p>The SparkMax absolute encoder API returns 0.0 to 1.0 rotations.
   * We convert to ±180° centered on the zero offset.
   * 
   * @return Absolute turret angle
   */
  public Angle getThroughBoreAbsolutePosition() {
    // SparkAbsoluteEncoder.getPosition() returns rotations (0.0 to 1.0)
    double rotations = throughBoreEncoder.getPosition();
    
    // Subtract zero offset and wrap to ±0.5 rotations (±180°)
    double adjusted = rotations - THROUGH_BORE_ZERO_OFFSET;
    // Wrap to -0.5 .. +0.5 range
    adjusted = adjusted - Math.round(adjusted);
    
    return Degrees.of(adjusted * 360.0);
  }
  
  /**
   * Syncs the NEO's relative encoder with the Through Bore Encoder's absolute position.
   * Call this on startup or if drift is detected.
   */
  public void syncEncoderWithThroughBore() {
    Angle absolutePosition = getThroughBoreAbsolutePosition();
    // Convert to motor rotations (accounting for gear ratio)
    double turretDegrees = absolutePosition.in(Degrees);
    double motorRotations = turretDegrees / 360.0 * TurretConstants.kGearRatio;
    spark.getEncoder().setPosition(motorRotations);
    hasBeenSynced = true;
    Logger.recordOutput("Turret/SyncedToThroughBore", turretDegrees);
  }
  
  /**
   * Command to sync the encoder with the Through Bore Encoder.
   * 
   * @return Command that performs the sync
   */
  public Command syncWithAbsoluteEncoder() {
    return Commands.runOnce(this::syncEncoderWithThroughBore, this)
        .withName("Turret.SyncWithThroughBore");
  }

  // ==================== COMMANDS ====================

  /**
   * Directly sets the turret target angle. Call this every loop when doing
   * dynamic aiming from a command that already requires this subsystem.
   * 
   * <p>This method directly controls the motor without creating a command,
   * so it can be called from within another command's execute() method.
   * 
  * @param angle Target angle (0 = turret forward, positive = left, negative = right)
  *              Note: turret forward points toward the robot rear due to backwards mounting.
   */
  public void setTargetAngle(Angle angle) {
    // Clamp to soft limits
    double angleDeg = angle.in(Degrees);
    double clampedDeg = Math.max(-MAX_ONE_DIR_FOV, Math.min(MAX_ONE_DIR_FOV, angleDeg));
    // Use direct motor control instead of scheduling a command to avoid conflicts
    motorController.setPosition(Degrees.of(clampedDeg));
  }

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
    Logger.recordOutput("Turret/ThroughBoreAngleDegrees", getThroughBoreAbsolutePosition().in(Degrees));
    Logger.recordOutput("Turret/HasBeenSynced", hasBeenSynced);
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
