// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.io.File;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveInputStream;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * SwerveSubsystem using YAGSL (Yet Another Generic Swerve Library).
 * 
 * <p>YAGSL simplifies swerve drive implementation by:
 * <ul>
 *   <li>Loading configuration from JSON files in deploy/swerve/</li>
 *   <li>Handling motor controller configuration automatically</li>
 *   <li>Providing built-in features like cosine compensation, heading correction</li>
 *   <li>Supporting multiple hardware configurations (SparkMax, TalonFX, etc.)</li>
 * </ul>
 * 
 * <p>Configuration files are in src/main/deploy/swerve/:
 * <ul>
 *   <li>swervedrive.json - Main config (gyro, module references)</li>
 *   <li>modules/*.json - Per-module configs (CAN IDs, locations)</li>
 *   <li>modules/physicalproperties.json - Gear ratios, current limits</li>
 *   <li>modules/pidfproperties.json - PID values</li>
 *   <li>controllerproperties.json - Heading controller</li>
 * </ul>
 * 
 * @see <a href="https://docs.yagsl.com">YAGSL Documentation</a>
 */
public class SwerveSubsystem extends SubsystemBase {
  
  /** Maximum robot speed in meters per second */
  private final double maximumSpeed = Units.feetToMeters(15.76); // ~4.8 m/s
  
  /** YAGSL SwerveDrive object - handles all swerve logic */
  private final SwerveDrive swerveDrive;
  
  /** PathPlanner robot configuration */
  private RobotConfig config;

  /**
   * Creates a new SwerveSubsystem.
   * Loads configuration from JSON files and configures PathPlanner.
   */
  public SwerveSubsystem() {
    // ==================== TELEMETRY ====================
    // Set telemetry verbosity for debugging
    // HIGH = all data, LOW = essential only, NONE = disabled
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    // ==================== LOAD CONFIGURATION ====================
    // YAGSL reads JSON config files from the deploy/swerve directory
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    
    // Starting pose for simulation - depends on alliance color
    // Blue alliance: Left side of field (X ~2.75m), facing right (0°)
    // Red alliance: Right side of field (X ~14.25m), facing left (180°)
    // Note: In sim, DriverStation.getAlliance() may not be set yet at construction time,
    // so we default to blue. The pose will be corrected when alliance is set.
    boolean isBlueAlliance = DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Blue)
        .orElse(true); // Default to blue if not set
    
    Pose2d simulationStartPose = isBlueAlliance 
        ? new Pose2d(2.75, 4.0, Rotation2d.fromDegrees(0))      // Blue: left side, facing right
        : new Pose2d(14.25, 4.0, Rotation2d.fromDegrees(180));  // Red: right side, facing left
    
    try {
      // Parse JSON files and create SwerveDrive object with starting pose
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, simulationStartPose);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create SwerveDrive from JSON config", e);
    }
    
    // ==================== YAGSL CONFIGURATION ====================
    // Heading correction should only be used while controlling the robot via angle (not angular velocity)
    // When enabled, it can cause unwanted rotation when driving translationally
    swerveDrive.setHeadingCorrection(false);
    
    // Correct for skew that gets worse as angular velocity increases
    // This compensates for the robot drifting while rotating at speed
    // DISABLED in simulation - causes pose glitching/jumping in sim
    if (!RobotBase.isSimulation()) {
      swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    }
    
    // Cosine compensation improves accuracy at high angles, but causes discrepancies in simulation
    swerveDrive.setCosineCompensator(!RobotBase.isSimulation());
    
    // ==================== PATHPLANNER CONFIGURATION ====================
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    
    // Configure PathPlanner AutoBuilder
    AutoBuilder.configure(
        this::getPose,           // Get current pose
        this::resetOdometry,     // Reset pose
        this::getRobotRelativeSpeeds, // Get robot-relative speeds
        
        // Drive with speeds (robot-relative)
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        
        // Holonomic path controller
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID
            new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
        ),
        config,
        
        // Flip path for red alliance
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
    );
  }

  // ==================== POSE ESTIMATION ====================
  
  /**
   * Gets the current robot pose from the pose estimator.
   * @return The robot's estimated position and rotation on the field
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }
  
  /**
   * Gets the current robot pose as a 3D pose.
   * Z is always 0 (ground level), pitch and roll are always 0.
   * @return The robot's pose in 3D space
   */
  public Pose3d getPose3d() {
    Pose2d pose2d = getPose();
    return new Pose3d(
        pose2d.getX(),
        pose2d.getY(),
        0.0,  // On the ground
        new Rotation3d(0, 0, pose2d.getRotation().getRadians())
    );
  }
  
  /**
   * Gets the robot's current rotation.
   * @return The robot's heading
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }
  
  /**
   * Resets the pose estimator to a specific pose.
   * @param pose The new pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }
  
  /**
   * Resets pose to origin (0, 0, 0).
   */
  public void resetPose() {
    resetOdometry(new Pose2d());
  }
  
  // ==================== VISION INTEGRATION ====================
  
  /**
   * Adds a vision measurement to the pose estimator.
   * Called by the Vision subsystem when a valid AprilTag pose is detected.
   *
   * @param visionRobotPoseMeters Robot pose from vision (field-relative)
   * @param timestampSeconds When the image was captured
   * @param visionMeasurementStdDevs How much to trust this measurement [x, y, theta]
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
  
  // ==================== SPEED GETTERS ====================
  
  /**
   * Gets the robot-relative chassis speeds.
   * @return Current ChassisSpeeds (robot frame)
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }
  
  /**
   * Gets the field-relative chassis speeds.
   * @return Current ChassisSpeeds (field frame)
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return swerveDrive.getFieldVelocity();
  }
  
  /**
   * Returns the robot's total velocity magnitude in m/s.
   */
  public double getRobotVelocity() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(
        speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
            + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
  }
  
  // ==================== DRIVE METHODS ====================
  
  /**
   * Main drive method for teleop - uses joystick inputs.
   * 
   * @param xSpeed Normalized speed (-1 to 1) forward
   * @param ySpeed Normalized speed (-1 to 1) left
   * @param rot Normalized rotation (-1 to 1) counter-clockwise
   * @param fieldRelative True for field-relative, false for robot-relative
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert normalized inputs to actual speeds
    double xSpeedMPS = xSpeed * maximumSpeed;
    double ySpeedMPS = ySpeed * maximumSpeed;
    double rotRadPS = rot * DriveConstants.kMaxAngularSpeed;
    
    // Create translation vector
    Translation2d translation = new Translation2d(xSpeedMPS, ySpeedMPS);
    
    // Drive the robot
    swerveDrive.drive(translation, rotRadPS, fieldRelative, false);
  }
  
  /**
   * Drives with a ChassisSpeeds object (robot-relative).
   * Used by PathPlanner for autonomous.
   * 
   * @param speeds Robot-relative chassis speeds
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }
  
  /**
   * Drives with a ChassisSpeeds object.
   * 
   * @param speeds Chassis speeds
   * @param fieldRelative True for field-relative
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      swerveDrive.driveFieldOriented(speeds);
    } else {
      swerveDrive.drive(speeds);
    }
  }
  
  // ==================== COMMANDS ====================
  
  /**
   * Creates a command that drives field-oriented using a SwerveInputStream.
   * This is the preferred method for teleop driving with YAGSL.
   * 
   * @param inputStream The SwerveInputStream providing joystick inputs
   * @return A command that continuously drives based on the input stream
   */
  public Command driveFieldOriented(SwerveInputStream inputStream) {
    return run(() -> swerveDrive.driveFieldOriented(inputStream.get()))
        .withName("SwerveSubsystem.driveFieldOriented");
  }
  
  /**
   * Creates a command that locks wheels in an X pattern.
   * Prevents the robot from being pushed.
   */
  public Command setXCommand() {
    return this.run(() -> swerveDrive.lockPose());
  }
  
  /**
   * Creates a command that locks wheels in an X pattern.
   * Alias for setXCommand() for CA26 compatibility.
   */
  public Command lockCommand() {
    return setXCommand();
  }
  
  /**
   * Locks the wheels in an X pattern immediately.
   * Use lockCommand() for a command-based approach.
   */
  public void lock() {
    swerveDrive.lockPose();
  }
  
  /**
   * Creates a command that centers all modules (points them forward).
   * Useful for testing and calibration.
   */
  public Command centerModulesCommand() {
    return run(() -> {
      // Create states with 0 speed and 0 angle
      SwerveModuleState[] centeredStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        centeredStates[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
      }
      swerveDrive.setModuleStates(centeredStates, false);
    }).withName("SwerveSubsystem.centerModules");
  }
  
  /**
   * Creates a command that resets the gyro heading.
   * Use when the robot is facing away from the driver station.
   */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> swerveDrive.zeroGyro());
  }
  
  /**
   * Zeros the gyro heading immediately.
   * Use zeroHeadingCommand() for a command-based approach.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
  
  /**
   * Gets the underlying YAGSL SwerveDrive object.
   * Useful for advanced operations and SwerveInputStream.
   * 
   * @return The SwerveDrive instance
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  
  // ==================== UTILITY METHODS ====================
  
  /**
   * Gets the current heading in degrees.
   * @return Heading in degrees (-180 to 180)
   */
  public double getHeading() {
    return getRotation().getDegrees();
  }
  
  /**
   * Returns the current swerve module states.
   * @return Array of module states
   */
  public SwerveModuleState[] getModuleStates() {
    return swerveDrive.getStates();
  }
  
  /**
   * Sets module states directly.
   * @param desiredStates Array of desired states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    swerveDrive.setModuleStates(desiredStates, false);
  }
  
  /**
   * Stops all drive motors.
   */
  public void stop() {
    drive(0, 0, 0, true);
  }
  
  /**
   * Resets drive motor encoders to zero.
   * Note: YAGSL handles encoder management internally.
   */
  public void resetEncoders() {
    // YAGSL doesn't expose resetEncoders directly
    // Encoder positions are handled internally by the library
    // If you need to reset odometry, use resetOdometry(new Pose2d()) instead
  }
  
  /**
   * Gets the turn rate in degrees per second.
   * @return Turn rate
   */
  public double getTurnRate() {
    return swerveDrive.getGyro().getYawAngularVelocity().magnitude();
  }
  
  @Override
  public void periodic() {
    // YAGSL handles odometry updates internally
    
    // Log robot pose for AdvantageScope 3D visualization
    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Odometry/Robot3d", getPose3d());
    
    // Log module states for visualization
    SwerveModuleState[] states = getModuleStates();
    Logger.recordOutput("SwerveStates/Measured", states);
    
    // Log chassis speeds
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    Logger.recordOutput("SwerveStates/VelocityX", speeds.vxMetersPerSecond);
    Logger.recordOutput("SwerveStates/VelocityY", speeds.vyMetersPerSecond);
    Logger.recordOutput("SwerveStates/AngularVelocity", Math.toDegrees(speeds.omegaRadiansPerSecond));
    
    // Log heading
    Logger.recordOutput("Odometry/Heading", getHeading());
  }
}
