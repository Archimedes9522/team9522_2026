// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * DriveSubsystem controls the swerve drivetrain.
 * 
 * Key responsibilities:
 * - Manage 4 MAXSwerve modules (each has a drive + turn motor)
 * - Track robot position using SwerveDrivePoseEstimator (odometry + vision fusion)
 * - Provide methods for teleop driving and autonomous path following
 * - Interface with PathPlanner for autonomous routines
 * 
 * Coordinate System:
 * - X: Forward is positive (toward front of robot)
 * - Y: Left is positive (toward left side of robot)
 * - Rotation: Counter-clockwise is positive
 */
public class DriveSubsystem extends SubsystemBase {
  /** PathPlanner robot configuration (mass, MOI, etc.) loaded from GUI settings */
  private RobotConfig config;

  // ==================== SWERVE MODULES ====================
  // Each module has a unique angular offset based on its position on the robot.
  // This offset accounts for how the absolute encoder is mounted.
  
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // ==================== SENSORS ====================
  /** NavX gyroscope - provides robot heading. Connected via MXP SPI port. */
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // ==================== POSE ESTIMATION ====================
  /**
   * SwerveDrivePoseEstimator fuses wheel odometry with vision measurements.
   * This provides a more accurate robot position than odometry alone.
   * 
   * How it works:
   * 1. Wheel encoders track distance traveled (fast, always available)
   * 2. Vision provides absolute position from AprilTags (slower, but corrects drift)
   * 3. Kalman filter combines both based on their standard deviations
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d()); // Initial pose at origin

  /** Creates a new DriveSubsystem and configures PathPlanner. */
  public DriveSubsystem() {
    // Load PathPlanner robot config from .pathplanner/settings.json
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // ==================== PATHPLANNER CONFIGURATION ====================
    // AutoBuilder is PathPlanner's interface to your drivetrain.
    // It needs to know how to: get pose, reset pose, drive, and when to flip for red alliance.
    AutoBuilder.configure(
        this::getPose,          // Supplier: Current robot pose
        this::resetOdometry,    // Consumer: Reset pose (called at auto start)
        this::getRobotRelativeSpeeds, // Supplier: Current chassis speeds (ROBOT RELATIVE!)
        
        // Consumer: Drive the robot given chassis speeds and optional feedforwards
        (speeds, feedforwards) -> drive(
            speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
            speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
            speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
            false), // false = robot-relative (PathPlanner handles field-relative internally)
        
        // PID controller for path following
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID (X and Y)
            new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
        ),
        config, // Robot physical configuration
        
        // Supplier: Should path be mirrored for red alliance?
        // PathPlanner paths are drawn on blue side - this flips for red
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Subsystem requirement (prevents other commands from using drive during auto)
    );

    // ==================== SWERVE WIDGET ====================
    // Publishes swerve state to SmartDashboard for visualization
    SmartDashboard.putData(
        "Swerve",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> m_frontLeft.getPosition().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> m_frontRight.getPosition().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> m_rearLeft.getPosition().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> m_rearLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> m_rearRight.getPosition().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> m_rearRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Robot Angle", () -> Rotation2d.fromDegrees(-m_gyro.getAngle()).getRadians(), null);
        });

    // Report MAXSwerve usage to WPILib for statistics
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  /**
   * Called every 20ms by the CommandScheduler.
   * Updates pose estimation from wheel odometry.
   */
  @Override
  public void periodic() {
    // Feed current gyro angle and module positions to pose estimator
    m_poseEstimator.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()), // Negated for WPILib convention
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  // ==================== VISION INTEGRATION ====================
  
  /**
   * Adds a vision measurement to the pose estimator.
   * Called by the Vision subsystem when a valid AprilTag pose is detected.
   *
   * @param visionRobotPoseMeters Robot pose from vision (field-relative)
   * @param timestampSeconds When the image was captured (for latency compensation)
   * @param visionMeasurementStdDevs How much to trust this measurement [x, y, theta]
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // ==================== POSE GETTERS ====================

  /** Returns the estimated robot pose on the field. */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Returns the robot's current rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns current chassis speeds (robot-relative). Used by PathPlanner. */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the robot's total velocity magnitude in m/s. */
  public double getRobotVelocity() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(
        speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
            + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
  }

  /** Returns current state of all swerve modules. */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
    };
  }

  // ==================== ODOMETRY RESET ====================

  /**
   * Resets the pose estimator to a specific pose.
   * Called by PathPlanner at the start of auto to set initial position.
   *
   * @param pose The new pose (position and rotation)
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  // ==================== DRIVE METHODS ====================

  /**
   * Main drive method - converts joystick inputs to swerve module states.
   *
   * @param xSpeed Normalized speed (-1 to 1) in the X direction (forward)
   * @param ySpeed Normalized speed (-1 to 1) in the Y direction (left)
   * @param rot Normalized rotation speed (-1 to 1)
   * @param fieldRelative If true, X/Y are relative to field. If false, relative to robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Scale normalized inputs to actual speeds
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // Convert chassis speeds to individual module states
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            // Field-relative: use gyro to rotate inputs to field frame
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
            // Robot-relative: use speeds as-is
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    // Desaturate to ensure no wheel exceeds max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    // Send states to modules
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Overload that takes ChassisSpeeds directly. */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    drive(
        speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
        speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
        speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
        fieldRelative);
  }

  // ==================== COMMANDS ====================

  /**
   * Creates a command that locks wheels in an X pattern.
   * This prevents the robot from being pushed and is useful for defense.
   */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  /**
   * Directly sets module states (used for testing or manual control).
   * Desaturates speeds to ensure no wheel exceeds max.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets all drive encoder positions to zero. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Creates a command that resets the gyro heading.
   * Use this when the robot is facing away from the driver station.
   */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_gyro.reset());
  }

  /** Resets pose to origin (0, 0, 0). */
  public void resetPose() {
    resetOdometry(new Pose2d());
  }

  /** Returns the robot heading in degrees (-180 to 180). */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /** Returns the turn rate in degrees per second. */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Stops all drive motors. */
  public void stop() {
    drive(0, 0, 0, true);
  }
}
