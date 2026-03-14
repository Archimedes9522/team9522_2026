// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
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
 *   import static frc.robot.Constants.AutoConstants.*;
 *   // Now you can use kPTranslation directly
 * </pre>
 */
public final class Constants {

  // === CHASSIS-ONLY MODE ===
  /**
   * Set to true when running on a bare chassis with no mechanism motors connected.
   * This prevents the robot from trying to initialize SparkMax controllers for
   * motors that don't exist on the CAN bus (IDs 15-23), which would cause
   * ~30 second timeouts PER MOTOR and result in 5+ minute startup times.
   * 
   * Set to false once mechanisms (shooter, turret, hood, intake, hopper, kicker)
   * are physically wired and connected.
   */
  public static final boolean kChassisOnly = false;

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
  
  // === ROBOT PHYSICAL PROPERTIES ===
  /** Robot mass in pounds — matches physicalproperties.json robotMass */
  public static final double kRobotMassLbs = 112.39;
  
  /** Robot mass in kilograms (for WPILib/PathPlanner APIs) */
  public static final double kRobotMassKg = Units.lbsToKilograms(kRobotMassLbs);

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
   * Autonomous mode constants for trajectory following.
   * 
   * <p>These values should match what is configured in {@code SwerveSubsystem}'s
   * PathPlanner {@code AutoBuilder.configure()} call. Update both places if changed.
   * 
   * <p>The {@code RobotConfig} is loaded from PathPlanner GUI settings
   * ({@code deploy/pathplanner/settings.json}).
   * 
   * <p>Cross-reference with {@code SwerveSubsystem.java} PathPlanner config:
   * <ul>
   *   <li>Translation PID: P=5.0, I=0.0, D=0.0</li>
   *   <li>Rotation PID: P=5.0, I=0.0, D=0.0</li>
   * </ul>
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
    // These match the values in SwerveSubsystem's PPHolonomicDriveController
    
    /** Translation correction gain (X and Y) — matches PathPlanner AutoBuilder config */
    public static final double kPTranslation = 5.0;
    
    /** Rotation correction gain — matches PathPlanner AutoBuilder config */
    public static final double kPRotation = 5.0;

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
   * Controller constants for driver and operator input.
   */
  public static final class ControllerConstants {
    /** USB port number for the driver's Xbox controller */
    public static final int kDriverControllerPort = 0;
    
    /** USB port number for the operator's Xbox controller */
    public static final int kOperatorControllerPort = 1;
    
    /** USB port number for the pose adjustment controller (for testing/debugging) */
    public static final int kPoseControllerPort = 2;
    
    /** Joystick deadband - inputs below this are ignored (prevents drift) */
    public static final double kDeadband = 0.1;
    
    /** 
     * Drive speed scaling (0.0 to 1.0).
     * CA26 uses full speed - adjust if you want to limit for new drivers.
     * 0.5 = 50% max speed, good for precision/learning
     * 1.0 = full speed (~4.8 m/s)
     */
    public static final double kDriveSpeedScale = 1.0;
    
    /** Rotation speed scaling (0.0 to 1.0) */
    public static final double kRotationSpeedScale = 1.0;
    
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
    
    /** Red alliance outpost position - moved inward toward center */
    RED_OUTPOST(new Translation3d(12.5, 6.0, 0)),
    
    /** Red alliance far side position - moved inward toward center */
    RED_FAR_SIDE(new Translation3d(12.5, 2.0, 0)),

    /** Blue alliance hub (scoring target) - left side of field */
    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    
    /** Blue alliance outpost position - moved inward toward center */
    BLUE_OUTPOST(new Translation3d(4.0, 2.0, 0)),
    
    /** Blue alliance far side position - moved inward toward center */
    BLUE_FAR_SIDE(new Translation3d(4.0, 6.0, 0));

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
   * Uses a NEO motor with a 40:1 reduction (4:1 REV Sport Gearbox + 200:20 gear).
   * Position is tracked via the NEO's internal encoder; use rezero when centered.
   */
  public static final class TurretConstants {
    /** CAN ID for the turret motor (NEO) */
    public static final int kMotorId = 17;

    /** Total gear reduction from motor to turret output */
    public static final double kGearRatio = 40.0;

    /** Soft limit for turret rotation in degrees (positive = clockwise) */
    public static final double kMaxAngleDegrees = 80.0;

    /** Soft limit for turret rotation in degrees (negative = counter-clockwise) */
    public static final double kMinAngleDegrees = -80.0;

    /**
     * Turret starting angle in degrees — where it physically sits at robot boot.
     * Position the turret at 0° (centered) before each match.
     * Use the rezero command (Start button) only when the turret is physically at 0°.
     */
    public static final double kStartingAngleDegrees = 0.0;

    /**
     * Operator preset offset for turret angles (degrees).
     * Use this to align D-pad presets with mounting orientation.
     */
    public static final double kOperatorPresetOffsetDegrees = 0.0;

    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 20;

    /** Motion profile limits (deg/s, deg/s²) */
    public static final double kMaxVelocityDegPerSec = 500.0;
    public static final double kMaxAccelDegPerSecSq = 500.0;

    // PID gains for position control
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Feedforward gains (YAMS passes mechanism rev/s to SimpleMotorFeedforward)
    // NEO free speed through 40:1 = 2.365 rev/s → theoretical kV = 12V / 2.365 = 5.07
    /** Static friction compensation voltage */
    public static final double kS = 0.025;
    /** Velocity feedforward (V per mechanism rev/s) */
    public static final double kV = 5.0;
  }

  /**
   * Hood mechanism constants.
   * 
   * <p>The hood is FIXED at a constant angle — no motor for the first qualifier.
   * A fixed polycarb hood is used. The shooting trajectory is controlled purely
   * by flywheel speed. A motorized hood may be added later.
   * 
   * <p>The HoodSubsystem runs in no-op mode (no physical motor).
   */
  public static final class HoodConstants {
    /** Whether the hood motor is physically present on the robot */
    public static final boolean kHasMotor = false;
    
    /** CAN ID for the hood motor (unused when kHasMotor = false) */
    public static final int kMotorId = 19;
    
    /** Total gear reduction from motor to hood output */
    public static final double kGearRatio = 50.0;
    
    /** Minimum hood angle in degrees (flat) */
    public static final double kMinAngleDegrees = 0.0;
    
    /** Maximum hood angle in degrees (steep) */
    public static final double kMaxAngleDegrees = 60.0;
    
    /** 
     * Fixed hood angle for shooting in degrees.
     * This is the launch angle for all shots - trajectory is controlled by flywheel speed.
     * ~55° provides a good arc for the expected shooting distances (2-6 meters).
     */
    public static final double kFixedShootingAngle = 55.0;
    
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
   * <ul>
   *   <li>Rollers: 4:1 REV Sport Gearbox + 1:1 belt = 4:1 total</li>
   *   <li>Pivot: 5:1 × 5:1 MAXPlanetary + 60:18 belt = 83.33:1 total</li>
   * </ul>
   */
  public static final class IntakeConstants {
    /** CAN ID for the intake pivot motor */
    public static final int kPivotMotorId = 20;

    /** CAN ID for the intake roller motor */
    public static final int kRollerMotorId = 21;

    /** Roller gear reduction: 4:1 REV Sport Gearbox + 1:1 belt */
    public static final double kRollerGearRatio = 4.0;

    /** Pivot gear reduction: 5:1 × 5:1 MAXPlanetary + 60:18 belt = 83.33:1 */
    public static final double kPivotGearRatio = 5.0 * 5.0 * (60.0 / 18.0); // 83.33:1

    /** Deployed angle in degrees (down to ground) — matching CA26 */
    public static final double kDeployedAngleDegrees = 148.0;

    /** Pivot soft limit minimum (degrees) */
    public static final double kPivotSoftMinAngleDeg = 0.0;

    /** Pivot soft limit maximum (degrees) */
    public static final double kPivotSoftMaxAngleDeg = 148.0;

    /** Pivot hard limit minimum (degrees) */
    public static final double kPivotHardMinAngleDeg = 0.0;

    /** Pivot hard limit maximum (degrees) */
    public static final double kPivotHardMaxAngleDeg = 153.0;

    /** Stowed angle in degrees (inside frame perimeter) */
    public static final double kStowedAngleDegrees = 0.0;

    /** Roller intake speed (duty cycle, -1 to 1) */
    public static final double kIntakeSpeed = 1.0;

    /** Roller outtake speed (duty cycle, -1 to 1) */
    public static final double kOuttakeSpeed = -0.5;

    /** Motor current limits in amps */
    public static final int kPivotCurrentLimitAmps = 30;
    public static final int kRollerCurrentLimitAmps = 40;

    // PID gains for pivot position control
    public static final double kPivotP = 100.0;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 15.0;

    /** Motion profile limits — faster for hopper push-out */
    public static final double kPivotMaxVelocityDegPerSec = 750.0;
    public static final double kPivotMaxAccelDegPerSec2 = 900.0;

    /** Closed-loop ramp rate (seconds to full output) */
    public static final double kPivotClosedLoopRampSec = 0.1;

  /** Closed-loop position tolerance (degrees) */
  public static final double kPivotToleranceDeg = 4.0;

    // Feedforward gains (YAMS passes mechanism rev/s to SimpleMotorFeedforward)
    // Vortex free speed through 83.33:1 = 1.357 rev/s → theoretical kV = 12V / 1.357 = 8.84
    /** Static friction compensation voltage */
    public static final double kPivotKs = 0.025;
    /** Gravity compensation — YAMS Arm handles this via mass/length in ArmConfig */
    public static final double kPivotKg = 0.5;
    /** Velocity feedforward (V per mechanism rev/s) — passed as kV to SimpleMotorFeedforward */
    public static final double kPivotKv = 1.0;
  }

  /**
   * Hopper (Indexer) mechanism constants.
   * 
   * <p>The hopper/indexer stores and queues FUEL balls between intake and shooter.
   * Uses a 4:1 MAXPlanetary Sport Gearbox + 1:1 belt = 4:1 total.
   */
  public static final class HopperConstants {
    /** CAN ID for the hopper motor */
    public static final int kMotorId = 22;
    
    /** Gear reduction: 4:1 MAXPlanetary Sport Gearbox + 1:1 belt */
    public static final double kGearRatio = 4.0;
    
    /** DIO port for the entry beam break sensor */
    public static final int kEntryBeamBreakPort = 0;
    
    /** DIO port for the exit beam break sensor */
    public static final int kExitBeamBreakPort = 1;
    
    /** Feed speed (duty cycle, -1 to 1) */
    public static final double kFeedSpeed = 1;  // Increased from 0.6 to reduce jamming
    
    /** Reverse speed for unjamming (duty cycle, -1 to 1) — matching CA26 */
    public static final double kReverseSpeed = -1.0;
    
    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 40;  // Increased from 20A to match CA26 — low limit was causing stalls under load
  }

  /**
   * Kicker mechanism constants.
   * 
   * <p>The kicker feeds FUEL from the hopper into the shooter.
   * Uses a NEO motor with 4:1 gearbox + 32:40 gear = 3.2:1 total.
   */
  public static final class KickerConstants {
    /** CAN ID for the kicker motor (NEO) */
    public static final int kMotorId = 23;
    
    /** Gear reduction: 4:1 gearbox + 32:40 gear transmission */
    public static final double kGearRatio = 4.0 * (32.0 / 40.0); // 3.2:1
    
    /** Feed speed (duty cycle, -1 to 1) */
    public static final double kFeedSpeed = 1.0;
    
    /** Motor current limit in amps */
    public static final int kCurrentLimitAmps = 20;
  }
}
