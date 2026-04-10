// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.io.File;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.vision.VisionConstants;
import swervelib.SwerveInputStream;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve drive subsystem using YAGSL. Configuration is loaded from
 * JSON files in {@code src/main/deploy/swerve/}.
 *
 * @see <a href="https://docs.yagsl.com">YAGSL Documentation</a>
 */
public class SwerveSubsystem extends SubsystemBase {

  private final double maximumSpeed = Units.feetToMeters(15.76);
  private final SwerveDrive swerveDrive;

  private RobotConfig config;
  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // Load YAGSL JSON configuration
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    // Simulation starting pose (default to blue; corrected when alliance is set)
    boolean isBlueAlliance = DriverStation.getAlliance()
        .map(alliance -> alliance == Alliance.Blue)
        .orElse(true);

    Pose2d simulationStartPose = isBlueAlliance
        ? new Pose2d(2.75, 4.0, Rotation2d.fromDegrees(0))
        : new Pose2d(14.25, 4.0, Rotation2d.fromDegrees(180));

    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, simulationStartPose);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create SwerveDrive from JSON config", e);
    }

    // ==================== YAGSL Advanced Features ====================
    // These match the CA26 recommended configuration for YAGSL swerve drives.
    // All four features are disabled in simulation to prevent MapleSim physics
    // feedback loops — they only activate on the real robot.

    // Heading correction: maintains heading when only translating (no rotation input).
    // Disabled for angular-velocity-based teleop; enable for heading-lock modes.
    swerveDrive.setHeadingCorrection(false);

    // Chassis discretization: applies ChassisSpeeds.discretize() to correct for
    // second-order kinematics skew when translating and rotating simultaneously.
    swerveDrive.setChassisDiscretization(true, 0.02);

    // Angular velocity compensation: uses IMU feedback to actively correct skew
    // that worsens with angular velocity. Coefficient sign depends on IMU orientation.
    // Negated from 0.1 → -0.1 after invertedIMU changed to false (commit 2db3c95).
    // If skew reverses direction, try 0.0 (disabled) or tune magnitude.
    swerveDrive.setAngularVelocityCompensation(
        !RobotBase.isSimulation(), !RobotBase.isSimulation(), -0.1);

    // Cosine compensation: scales drive output by cos(steering error) so modules
    // contribute proportionally even when not perfectly aligned yet.
    swerveDrive.setCosineCompensator(!RobotBase.isSimulation());

    // Module auto-sync: re-synchronizes relative encoders with absolute encoders
    // when modules are stationary. 3° tolerance before a sync is triggered.
    swerveDrive.setModuleEncoderAutoSynchronize(true, 3);

    // Drive motor feedforward from SysId characterization (values in volts).
    // Uses WPILib SimpleMotorFeedforward which handles kS (static friction).
    swerveDrive.replaceSwerveModuleFeedforward(
        new SimpleMotorFeedforward(0.097, 2.557, 0.222));

    // ==================== PathPlanner ====================
    try {
      config = RobotConfig.fromGUISettings();

      // SwerveSetpointGenerator produces kinematically feasible setpoints
      // for smoother teleop driving and reduced wheel scrub.
      double maxSteerVelocityRadPerSec = (5676.0 / 60.0 / 46.42) * 2.0 * Math.PI;
      setpointGenerator = new SwerveSetpointGenerator(config, maxSteerVelocityRadPerSec);

      previousSetpoint = new SwerveSetpoint(
          getRobotRelativeSpeeds(),
          swerveDrive.getStates(),
          DriveFeedforwards.zeros(config.numModules));
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> swerveDrive.drive(
            speeds,
            swerveDrive.kinematics.toSwerveModuleStates(speeds),
            feedforwards.linearForces()),
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPTranslation, 0.0, 0.0),
            new PIDConstants(AutoConstants.kPRotation, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false),
        this);
  }

  // ==================== Pose Estimation ====================

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Pose3d getPose3d() {
    Pose2d pose2d = getPose();
    return new Pose3d(
        pose2d.getX(), pose2d.getY(), 0.0,
        new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void resetPose() {
    resetOdometry(new Pose2d());
  }

  // ==================== Vision ====================

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionRobotPoseMeters    Robot pose from vision
   * @param timestampSeconds         Capture timestamp
   * @param visionMeasurementStdDevs Trust level [x, y, theta]
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // ==================== Speed Getters ====================

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return swerveDrive.getFieldVelocity();
  }

  /** Returns total translational velocity magnitude in m/s. */
  public double getRobotVelocity() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(
        speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
            + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
  }

  // ==================== Drive Methods ====================

  /**
   * Drives with normalized joystick inputs.
   *
   * @param xSpeed        Forward speed (-1 to 1)
   * @param ySpeed        Left speed (-1 to 1)
   * @param rot           CCW rotation (-1 to 1)
   * @param fieldRelative True for field-relative driving
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedMPS = xSpeed * maximumSpeed;
    double ySpeedMPS = ySpeed * maximumSpeed;
    double rotRadPS = rot * swerveDrive.getMaximumChassisAngularVelocity();
    swerveDrive.drive(new Translation2d(xSpeedMPS, ySpeedMPS), rotRadPS, fieldRelative, false);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      swerveDrive.driveFieldOriented(speeds);
    } else {
      swerveDrive.drive(speeds);
    }
  }

  /**
   * Drives using PathPlanner's SwerveSetpointGenerator for smoother
   * module transitions and reduced wheel scrub.
   */
  public void driveWithSetpoints(ChassisSpeeds speeds) {
    if (setpointGenerator == null || previousSetpoint == null) {
      swerveDrive.drive(speeds);
      return;
    }
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
    swerveDrive.drive(
        previousSetpoint.robotRelativeSpeeds(),
        previousSetpoint.moduleStates(),
        previousSetpoint.feedforwards().linearForces());
  }

  // ==================== Heading ====================

  /**
   * Enables/disables heading correction. When enabled, the robot actively
   * maintains its heading while translating without rotational input.
   */
  public void setHeadingCorrection(boolean enabled) {
    swerveDrive.setHeadingCorrection(enabled);
  }

  // ==================== Commands ====================

  /** Drives field-oriented using a SwerveInputStream (default teleop command). */
  public Command driveFieldOriented(SwerveInputStream inputStream) {
    return run(() -> swerveDrive.driveFieldOriented(inputStream.get()))
        .withName("SwerveSubsystem.driveFieldOriented");
  }

  /**
   * Drives field-oriented with setpoint generation for smoother module
   * transitions. Recommended for competition.
   */
  public Command driveFieldOrientedWithSetpoints(SwerveInputStream inputStream) {
    return run(() -> {
      ChassisSpeeds fieldSpeeds = inputStream.get();
      ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getRotation());
      driveWithSetpoints(robotSpeeds);
    }).withName("SwerveSubsystem.driveFieldOrientedWithSetpoints");
  }

  /** Locks wheels in an X pattern to prevent being pushed. */
  public Command lockCommand() {
    return this.run(() -> swerveDrive.lockPose())
        .withName("SwerveSubsystem.lock");
  }

  /** Centers all modules (points wheels forward). Useful for calibration. */
  public Command centerModulesCommand() {
    return run(() -> {
      SwerveModuleState[] centeredStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        centeredStates[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
      }
      swerveDrive.setModuleStates(centeredStates, false);
    }).withName("SwerveSubsystem.centerModules");
  }

  /**
   * Resets gyro heading for the current alliance. Press when the robot
   * is facing away from the driver station.
   */
  public Command zeroHeadingForAllianceCommand() {
    return this.runOnce(() -> {
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      Rotation2d heading = (alliance == Alliance.Red)
          ? Rotation2d.fromDegrees(180)
          : Rotation2d.fromDegrees(0);
      resetOdometry(new Pose2d(getPose().getTranslation(), heading));
      System.out.println("[Gyro] Zeroed heading to " + heading.getDegrees()
          + "° for " + alliance + " alliance");
    }).withName("SwerveSubsystem.zeroHeadingForAlliance");
  }

  /**
   * Flips the heading 180° to invert field-relative controls.
   * Use when alliance detection is wrong or controls feel backwards.
   */
  public Command flipHeadingCommand() {
    return this.runOnce(() -> {
      Rotation2d current = getPose().getRotation();
      Rotation2d flipped = current.rotateBy(Rotation2d.fromDegrees(180));
      resetOdometry(new Pose2d(getPose().getTranslation(), flipped));
      System.out.println("[Gyro] Flipped heading 180°: " + current.getDegrees()
          + "° -> " + flipped.getDegrees() + "°");
    }).withName("SwerveSubsystem.flipHeading");
  }

  // ==================== Utility ====================

  public void resetHeadingForAlliance(Alliance alliance) {
    Rotation2d heading = (alliance == Alliance.Red)
        ? Rotation2d.fromDegrees(180)
        : Rotation2d.fromDegrees(0);
    resetOdometry(new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public double getHeading() {
    return getRotation().getDegrees();
  }

  public SwerveModuleState[] getModuleStates() {
    return swerveDrive.getStates();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    swerveDrive.setModuleStates(desiredStates, false);
  }

  public void stop() {
    drive(0, 0, 0, true);
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public double getTurnRate() {
    return swerveDrive.getGyro().getYawAngularVelocity().magnitude();
  }

  // ==================== SysId ====================

  /**
   * Returns a SysId command sequence for the drive motors.
   * Locks all modules to 0° and ramps voltage on drive motors.
   * Log the .wpilog file and open in SysId tool to extract kS, kV, kA.
   */
  public Command sysIdDriveMotors() {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Second), // ramp rate
            Volts.of(7),             // step voltage
            Seconds.of(10)),         // timeout
        new SysIdRoutine.Mechanism(
            voltage -> {
              for (SwerveModule mod : swerveDrive.getModules()) {
                mod.getAngleMotor().setVoltage(0); // lock steering straight
                mod.getDriveMotor().setVoltage(voltage.in(Volts));
              }
            },
            log -> {
              for (SwerveModule mod : swerveDrive.getModules()) {
                log.motor("drive-" + mod.moduleNumber)
                    .voltage(Volts.of(mod.getDriveMotor().getVoltage()))
                    .linearPosition(Meters.of(mod.getDriveMotor().getPosition()))
                    .linearVelocity(MetersPerSecond.of(mod.getDriveMotor().getVelocity()));
              }
            },
            this));

    return Commands.sequence(
        routine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1),
        routine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1),
        routine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1),
        routine.dynamic(SysIdRoutine.Direction.kReverse)
    ).withName("SysId Drive Motors");
  }

  /**
   * Returns a SysId command sequence for the angle (steering) motors.
   * Stops drive motors and ramps voltage on angle motors.
   */
  public Command sysIdAngleMotors() {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), // slower ramp for angle
            Volts.of(4),               // lower step voltage
            Seconds.of(10)),           // timeout
        new SysIdRoutine.Mechanism(
            voltage -> {
              for (SwerveModule mod : swerveDrive.getModules()) {
                mod.getDriveMotor().setVoltage(0); // stop driving
                mod.getAngleMotor().setVoltage(voltage.in(Volts));
              }
            },
            log -> {
              for (SwerveModule mod : swerveDrive.getModules()) {
                log.motor("angle-" + mod.moduleNumber)
                    .voltage(Volts.of(mod.getAngleMotor().getVoltage()))
                    .angularPosition(Degrees.of(mod.getAngleMotor().getPosition()))
                    .angularVelocity(DegreesPerSecond.of(mod.getAngleMotor().getVelocity()));
              }
            },
            this));

    return Commands.sequence(
        routine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1),
        routine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1),
        routine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1),
        routine.dynamic(SysIdRoutine.Direction.kReverse)
    ).withName("SysId Angle Motors");
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Odometry/Heading", getHeading());
    Logger.recordOutput("Swerve/ModuleStates", getModuleStates());

    // Log camera field poses for 3D visualization in AdvantageScope
    Pose3d robotPose3d = getPose3d();
    Logger.recordOutput("Vision/CameraPoses", new Pose3d[] {
        robotPose3d.transformBy(VisionConstants.robotToCamera0),
        robotPose3d.transformBy(VisionConstants.robotToCamera1)
    });
  }
}
