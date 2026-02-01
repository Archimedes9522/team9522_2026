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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

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

  /**
   * Controller constants for driver and operator input.
   * Separated from OIConstants for cleaner organization.
   */
  public static final class ControllerConstants {
    /** USB port number for the driver's Xbox controller */
    public static final int kDriverControllerPort = 0;
    
    /** USB port number for the operator's Xbox controller */
    public static final int kOperatorControllerPort = 1;
    
    /** Joystick deadband - inputs below this are ignored (prevents drift) */
    public static final double kDeadband = 0.1;
    
    /** 
     * Drive speed scaling (0.0 to 1.0).
     * Start low for new drivers, increase as skill improves.
     * 0.25 = 25% max speed, good for learning
     * 0.5 = 50% max speed, good for precision
     * 1.0 = full speed, for experienced drivers
     */
    public static final double kDriveSpeedScale = 0.5;
    
    /** Rotation speed scaling (0.0 to 1.0) */
    public static final double kRotationSpeedScale = 0.5;
    
    /** Threshold for trigger buttons to register as pressed */
    public static final double kTriggerThreshold = 0.2;
  }

  /**
   * Aim points for the 2026 "Rebuilt" game field.
   * 
   * <p>These are field positions in meters that mechanisms can aim at.
   * The field is 16.54m x 8.05m. Red alliance is on the right side (high X).
   * 
   * <p>Each enum value contains a Translation3d (X, Y, Z) in meters.
   */
  public enum AimPoints {
    /** Red alliance hub (scoring target) - right side of field */
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    
    /** Red alliance outpost position */
    RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
    
    /** Red alliance far side position */
    RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

    /** Blue alliance hub (scoring target) - left side of field */
    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    
    /** Blue alliance outpost position */
    BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
    
    /** Blue alliance far side position */
    BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

    /** The 3D position of this aim point on the field */
    public final Translation3d value;

    private AimPoints(Translation3d value) {
      this.value = value;
    }

    /**
     * Gets the hub position for the current alliance.
     * @return Hub Translation3d for red or blue based on DriverStation alliance
     */
    public static Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value)
          .orElse(RED_HUB.value);
    }

    /**
     * Gets the outpost position for the current alliance.
     * @return Outpost Translation3d for red or blue based on DriverStation alliance
     */
    public static Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value)
          .orElse(RED_OUTPOST.value);
    }

    /**
     * Gets the far side position for the current alliance.
     * @return Far side Translation3d for red or blue based on DriverStation alliance
     */
    public static Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value)
          .orElse(RED_FAR_SIDE.value);
    }
  }

  // ==================== MECHANISM CONSTANTS ====================
  // These constants define the CAN IDs and parameters for game mechanisms.
  // Adjust based on your actual robot hardware.

  /**
   * Shooter mechanism constants.
   * 
   * <p>The shooter uses dual NEO motors with 4" wheels in a 1:1 ratio
   * to launch FUEL balls at the hub.
   */
  public static final class ShooterConstants {
    /** CAN ID for the leader (left) shooter motor */
    public static final int kLeaderMotorId = 15;
    
    /** CAN ID for the follower (right) shooter motor */
    public static final int kFollowerMotorId = 16;
    
    /** Shooter wheel diameter in inches */
    public static final double kWheelDiameterInches = 4.0;
    
    /** Gear ratio from motor to wheel (1:1 = direct drive) */
    public static final double kGearRatio = 1.0;
    
    /** Maximum allowed shooter speed in RPM */
    public static final double kMaxSpeedRpm = 6000;
    
    /** Default shooting speed in RPM */
    public static final double kShootingSpeedRpm = 5500;
    
    /** Speed tolerance for "at setpoint" detection in RPM */
    public static final double kSpeedToleranceRpm = 100;
    
    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 40;
    
    // PID gains for velocity control (tune these on actual robot)
    public static final double kP = 0.00936;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // Feedforward gains (tune with SysId)
    public static final double kS = 0.191;  // static friction voltage
    public static final double kV = 0.11858; // velocity gain (V per rad/s)
    public static final double kA = 0.0;     // acceleration gain
  }

  /**
   * Turret mechanism constants.
   * 
   * <p>The turret rotates the shooter ±90° to aim at the hub.
   * Uses a NEO motor with a 40:1 reduction and CANcoder for absolute position.
   */
  public static final class TurretConstants {
    /** CAN ID for the turret motor */
    public static final int kMotorId = 17;
    
    /** CAN ID for the CANcoder (absolute encoder) */
    public static final int kEncoderId = 18;
    
    /** Total gear reduction from motor to turret output */
    public static final double kGearRatio = 40.0;
    
    /** Soft limit for turret rotation in degrees (positive = clockwise) */
    public static final double kMaxAngleDegrees = 90.0;
    
    /** Soft limit for turret rotation in degrees (negative = counter-clockwise) */
    public static final double kMinAngleDegrees = -90.0;
    
    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 30;
    
    // PID gains for position control
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  /**
   * Hood mechanism constants.
   * 
   * <p>The hood adjusts the shooter angle to control trajectory.
   * Uses a NEO 550 with a 50:1 reduction.
   */
  public static final class HoodConstants {
    /** CAN ID for the hood motor */
    public static final int kMotorId = 19;
    
    /** Total gear reduction from motor to hood output */
    public static final double kGearRatio = 50.0;
    
    /** Minimum hood angle in degrees (flat) */
    public static final double kMinAngleDegrees = 0.0;
    
    /** Maximum hood angle in degrees (steep) */
    public static final double kMaxAngleDegrees = 60.0;
    
    /** Default stowed angle in degrees */
    public static final double kStowedAngleDegrees = 0.0;
    
    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 20;
    
    // PID gains for position control
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  /**
   * Intake mechanism constants.
   * 
   * <p>The intake has a pivot arm and rollers to collect FUEL from the floor.
   */
  public static final class IntakeConstants {
    /** CAN ID for the intake pivot motor */
    public static final int kPivotMotorId = 20;
    
    /** CAN ID for the intake roller motor */
    public static final int kRollerMotorId = 21;
    
    /** Pivot gear reduction */
    public static final double kPivotGearRatio = 5.0 * 5.0 * (60.0 / 18.0); // 83.33:1
    
    /** Deployed angle in degrees (down to ground) */
    public static final double kDeployedAngleDegrees = 90.0;
    
    /** Stowed angle in degrees (inside frame perimeter) */
    public static final double kStowedAngleDegrees = 0.0;
    
    /** Roller intake speed (duty cycle, -1 to 1) */
    public static final double kIntakeSpeed = 0.8;
    
    /** Roller outtake speed (duty cycle, -1 to 1) */
    public static final double kOuttakeSpeed = -0.5;
    
    /** Motor current limits in amps */
    public static final int kPivotCurrentLimitAmps = 30;
    public static final int kRollerCurrentLimitAmps = 25;
    
    // PID gains for pivot position control
    public static final double kPivotP = 0.1;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;
  }

  /**
   * Hopper mechanism constants.
   * 
   * <p>The hopper stores FUEL balls between intake and shooter.
   */
  public static final class HopperConstants {
    /** CAN ID for the hopper motor */
    public static final int kMotorId = 22;
    
    /** DIO port for the entry beam break sensor */
    public static final int kEntryBeamBreakPort = 0;
    
    /** DIO port for the exit beam break sensor */
    public static final int kExitBeamBreakPort = 1;
    
    /** Feed speed (duty cycle, -1 to 1) */
    public static final double kFeedSpeed = 0.6;
    
    /** Reverse speed for unjamming (duty cycle, -1 to 1) */
    public static final double kReverseSpeed = -0.4;
    
    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 20;
  }

  /**
   * Kicker mechanism constants.
   * 
   * <p>The kicker feeds FUEL from the hopper into the shooter.
   */
  public static final class KickerConstants {
    /** CAN ID for the kicker motor */
    public static final int kMotorId = 23;
    
    /** Feed speed (duty cycle, -1 to 1) */
    public static final double kFeedSpeed = 1.0;
    
    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 20;
  }
}
