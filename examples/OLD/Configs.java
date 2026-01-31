package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

/**
 * Motor configuration classes for REV Robotics SparkMax controllers.
 * 
 * <p>This class uses REVLib 2026's fluent configuration API where all settings
 * are built into SparkMaxConfig objects, then applied to motors at initialization.
 * 
 * <p>Benefits of this approach:
 * - All motor settings are in one place for easy tuning
 * - Configuration is applied atomically (all-at-once)
 * - Reduces CAN bus traffic vs setting each parameter individually
 * 
 * @see MAXSwerveModule For where these configs are applied to actual motors
 */
public final class Configs {
  
  /**
   * Configuration for MAXSwerve module motors (driving and turning).
   * 
   * <p>MAXSwerve modules use two NEO brushless motors:
   * - Driving motor: Controls wheel speed, uses built-in encoder
   * - Turning motor: Controls wheel angle, uses through-bore absolute encoder
   */
  public static final class MAXSwerveModule {
    /** Configuration for the driving motor (controls wheel velocity) */
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    
    /** Configuration for the turning motor (controls wheel angle) */
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    // Static block runs once when class is loaded - builds all configurations
    static {
      // === CONVERSION FACTORS ===
      // These convert raw encoder units to real-world units (meters, radians)
      
      // Driving: Convert motor rotations to wheel meters traveled
      // Formula: wheel_circumference / gear_ratio
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      
      // Turning: Convert motor rotations to wheel angle in radians (full rotation = 2π)
      double turningFactor = 2 * Math.PI;
      
      // Feed forward: Approximate motor output needed for desired velocity
      // kV = 1 / max_free_speed (so at max speed, feed forward outputs 1.0)
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      // === DRIVING MOTOR CONFIGURATION ===
      drivingConfig
          .idleMode(IdleMode.kBrake)      // Brake when not commanded (helps hold position)
          .smartCurrentLimit(40)           // 40A current limit protects motor and battery
          .openLoopRampRate(2);            // 2 seconds to reach full power (prevents wheel slip)
      
      // Encoder settings - convert ticks to meters
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor)        // meters per rotation
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second (RPM -> RPS)
      
      // Closed-loop velocity control (PID + feed forward)
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Use built-in NEO encoder
          .pid(0.04, 0, 0)     // P-only works well for velocity control - TUNE THESE!
          .outputRange(-1, 1); // Full forward/reverse range
      
      // Feed forward helps the PID by providing a baseline output
      drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward);

      // === TURNING MOTOR CONFIGURATION ===
      turningConfig
          .idleMode(IdleMode.kBrake)   // Brake to hold wheel angle
          .smartCurrentLimit(20);       // 20A is enough for just rotating the wheel
      
      // Through-bore absolute encoder settings
      turningConfig.absoluteEncoder
          .inverted(true)  // Module gearing reverses direction - encoder must match
          .positionConversionFactor(turningFactor)        // radians per rotation
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      
      // Closed-loop position control (PID)
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // Use through-bore encoder
          .pid(1, 0, 0)       // P-only works well for position - TUNE THIS!
          .outputRange(-1, 1) // Full forward/reverse range
          
          // Position wrapping lets the controller take the shortest path
          // Example: Going from 350° to 10° goes through 0° (20° turn)
          //          instead of going the long way (340° turn)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor); // Wrap at 0 to 2π radians
    }
  }

}
