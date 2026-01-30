// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

/**
 * MAXSwerveModule represents a single swerve module (one corner of the robot).
 * 
 * Each MAXSwerve module consists of:
 * - Driving motor (NEO) with relative encoder - controls wheel speed
 * - Turning motor (NEO 550) with absolute encoder (Through Bore) - controls wheel angle
 * 
 * The absolute encoder on the turning motor means we always know the exact wheel
 * direction, even after a power cycle - no zeroing required!
 * 
 * Key concepts:
 * - SwerveModuleState: Desired speed (m/s) and angle (Rotation2d)
 * - SwerveModulePosition: Distance traveled (m) and angle (for odometry)
 * - Angular offset: Each module's encoder may be installed at a different angle
 */
public class MAXSwerveModule {
  // ==================== HARDWARE ====================
  /** Driving motor - controls wheel speed. Uses relative encoder for velocity. */
  private final SparkMax m_drivingSpark;
  
  /** Turning motor - controls wheel angle. Uses absolute encoder for position. */
  private final SparkMax m_turningSpark;

  /** Relative encoder on driving motor - tracks distance traveled and velocity */
  private final RelativeEncoder m_drivingEncoder;
  
  /** Absolute encoder on turning motor - knows exact wheel angle without zeroing */
  private final AbsoluteEncoder m_turningEncoder;

  /** PID controller for driving motor velocity control */
  private final SparkClosedLoopController m_drivingClosedLoopController;
  
  /** PID controller for turning motor position control */
  private final SparkClosedLoopController m_turningClosedLoopController;

  // ==================== STATE ====================
  /**
   * Angular offset compensates for how the module is mounted on the chassis.
   * This is set per-module in DriveConstants based on module position.
   */
  private double m_chassisAngularOffset = 0;
  
  /** Last commanded state (for debugging/logging) */
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule with the specified CAN IDs and angular offset.
   *
   * @param drivingCANId CAN ID of the driving SparkMax (1-62)
   * @param turningCANId CAN ID of the turning SparkMax (1-62)
   * @param chassisAngularOffset Offset in radians to align module with chassis
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    // Create SparkMax motor controllers
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    // Get encoder references
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    // Get closed-loop controller references
    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply motor configurations from Configs.java
    // ResetMode.kResetSafeParameters: Reset to defaults first for clean state
    // PersistMode.kPersistParameters: Save to flash so settings survive power loss
    m_drivingSpark.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Store offset and initialize state
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0); // Reset drive distance to 0
  }

  /**
   * Returns the current state (velocity and angle) of the module.
   * Used for odometry and telemetry.
   *
   * @return Current SwerveModuleState with velocity in m/s and angle in radians
   */
  public SwerveModuleState getState() {
    // Subtract offset to get angle relative to chassis (not encoder)
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position (distance and angle) of the module.
   * Used by SwerveDrivePoseEstimator for odometry calculations.
   *
   * @return Current SwerveModulePosition with distance in meters and angle
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(), // Total distance driven in meters
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Commands the module to the desired state.
   * This is the main method called by DriveSubsystem.
   *
   * @param desiredState Target speed (m/s) and angle
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Add angular offset to convert from chassis-relative to encoder-relative
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize: If we need to turn more than 90°, reverse direction instead
    // Example: Instead of turning 180° and driving forward, just drive backward
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command the motors using closed-loop control
    // Driving: Velocity PID control (m/s)
    m_drivingClosedLoopController.setSetpoint(
        correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    // Turning: Position PID control (radians)
    m_turningClosedLoopController.setSetpoint(
        correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Resets the driving encoder position to zero. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
