// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Robot-wide constants organized into logical groups.
 * 
 * <p>This class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. All constants should be declared globally (public static) and should not contain
 * any functional code.
 * 
 * <p>Tip: Use static imports to reduce verbosity:
 * <pre>
 *   import static frc.robot.Constants.DriveConstants.*;
 *   // Now you can use kMaxSpeedMetersPerSecond directly
 * </pre>
 */
public final class Constants {

  // === CONTROLLER BUTTON MAPPINGS ===
  /** Right bumper puts the swerve drive into X-stance (wheels pointed inward to resist pushing) */
  public static final int kSetXButton = XboxController.Button.kRightBumper.value;

  // === ROBOT MODE DETECTION ===
  /** Default simulation mode when not running on real robot */
  public static final Mode simMode = Mode.SIM;
  
  /**
   * Current robot mode - automatically detected at startup.
   * REAL when deployed to robot, or simMode (SIM/REPLAY) when in simulation.
   */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // === REUSABLE ZERO OBJECTS (avoids creating garbage) ===
  public static final Translation2d zeroTranslation2d = new Translation2d();
  public static final Rotation2d zeroRotation2d = new Rotation2d();
  public static final Pose2d zeroPose2d = new Pose2d();
  public static final Pose3d zeroPose3d = new Pose3d();

  /**
   * Robot operating modes for AdvantageKit logging.
   */
  public static enum Mode {
    /** Running on a real robot with actual hardware */
    REAL,

    /** Running in simulation with physics models */
    SIM,

    /** Replaying from a log file for debugging/analysis */
    REPLAY
  }

  /**
   * Constants for the swerve drive subsystem.
   * 
   * <p>Includes physical dimensions, speed limits, module positions, and CAN IDs.
   */
  public static final class DriveConstants {
    // === SPEED LIMITS ===
    // Note: These are the ALLOWED maximums, not the theoretical maximums
    
    /** Maximum driving speed in meters per second */
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    
    /** Maximum rotation speed in radians per second (2π = one full rotation per second) */
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    // === CHASSIS DIMENSIONS ===
    // These measurements are between wheel CENTERS, not frame edges
    // Frame size is 27.5" (front-to-back) x 25.5" (side-to-side), with 3.5" inset to wheel centers
    
    /** Distance between left and right wheel centers (side to side) - frame is 25.5" wide */
    public static final double kTrackWidth = Units.inchesToMeters(22);
    
    /** Distance between front and back wheel centers (front to back) - frame is 27.5" long */
    public static final double kWheelBase = Units.inchesToMeters(24);
    
    /**
     * Kinematics object that converts chassis speeds to individual module states.
     * 
     * <p>Module positions are specified as (X, Y) from robot center:
     * - X positive = forward, X negative = backward
     * - Y positive = left, Y negative = right
     * 
     * Order: Front Left, Front Right, Back Left, Back Right
     */
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // Front Left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Back Left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back Right

    // === MODULE ANGULAR OFFSETS ===
    // Each module's "zero" position relative to the chassis
    // These compensate for how the modules are physically mounted
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;   // -90 degrees
    public static final double kFrontRightChassisAngularOffset = 0;              // 0 degrees
    public static final double kBackLeftChassisAngularOffset = Math.PI;          // 180 degrees
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;     // 90 degrees

    // === CAN BUS IDs ===
    // Each module has two motors: driving (wheel speed) and turning (wheel angle)
    // Make sure these match your actual wiring!
    
    // Front Left Module
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kFrontLeftTurningCanId = 4;

    // Front Right Module
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kFrontRightTurningCanId = 6;

    // Rear Left Module
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kRearLeftTurningCanId = 2;

    // Rear Right Module
    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    /** Set to true if gyro reports positive rotation when robot turns clockwise */
    public static final boolean kGyroReversed = true;
  }

  /**
   * Constants for individual swerve modules (MAXSwerve-specific).
   * 
   * <p>These values are used to calculate encoder conversion factors
   * and determine theoretical maximum speeds.
   */
  public static final class ModuleConstants {
    /** Number of teeth on the driving motor's pinion gear (changes speed/torque ratio) */
    public static final int kDrivingMotorPinionTeeth = 13;

    // === CALCULATED VALUES ===
    // These are derived from the physical constants above
    
    /** NEO motor free speed in rotations per second */
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    
    /** Wheel diameter (3 inch colson wheel) */
    public static final double kWheelDiameterMeters = 0.0762;
    
    /** Distance traveled in one wheel rotation */
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
    /**
     * Gear ratio from motor to wheel.
     * MAXSwerve uses: 45 teeth bevel gear / 22 teeth first stage / 15 teeth bevel pinion
     */
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    
    /** Theoretical maximum wheel speed (actual will be lower due to friction, battery sag, etc.) */
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    // === ENCODER CONVERSION FACTORS ===
    // These convert encoder units to real-world units
    
    /** Meters traveled per motor rotation */
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction;
    
    /** Meters per second from motor RPM */
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0;

    /** Radians per motor rotation (full rotation = 2π) */
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
    
    /** Radians per second from motor RPM */
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;

    /** PID wrapping minimum (0 radians) */
    public static final double kTurningEncoderPositionPIDMinInput = 0;
    
    /** PID wrapping maximum (2π radians) */
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;

    /** Brake mode holds position when not powered */
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    /** Current limits protect motors and battery */
    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  /**
   * Operator Interface constants (controllers and inputs).
   */
  public static final class OIConstants {
    /** USB port number for the driver's Xbox controller */
    public static final int kDriverControllerPort = 0;
    
    /** Joystick deadband - inputs below this are ignored (prevents drift) */
    public static final double kDriveDeadband = 0.1;
    
    /** Threshold for trigger buttons to register as pressed */
    public static final double kTriggerButtonThreshold = 0.2;
    
    /** Speed multiplier for driving (0.5 = half speed, good for learning/precision) */
    public static final double kDriveSpeedFactor = 0.5;
  }

  /**
   * Autonomous mode constants for trajectory following.
   */
  public static final class AutoConstants {
    /** Maximum speed during autonomous paths */
    public static final double kMaxSpeedMetersPerSecond = 4.46;
    
    /** Maximum acceleration during autonomous paths */
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
    /** Maximum rotation speed during autonomous */
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // === PID GAINS FOR PATH FOLLOWING ===
    /** X position correction gain */
    public static final double kPXController = 1;
    
    /** Y position correction gain */
    public static final double kPYController = 1;
    
    /** Rotation correction gain (usually higher since angle errors are more noticeable) */
    public static final double kPThetaController = 5;

    /** Motion profile constraints for rotation controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    /**
     * PathPlanner robot configuration, loaded from GUI settings.
     * This includes mass, moment of inertia, and module positions for path following.
     */
    public static RobotConfig config;

    static {
      try {
        // Load configuration from PathPlanner GUI (stored in deploy folder)
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * NEO motor specifications.
   */
  public static final class NeoMotorConstants {
    /** NEO brushless motor free speed (no load) in RPM */
    public static final double kFreeSpeedRpm = 5676;
  }
}
