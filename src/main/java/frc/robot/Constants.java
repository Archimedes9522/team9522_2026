// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Robot-wide constants organized into logical groups. */
public final class Constants {

  /** Set true when running on a bare chassis (no mechanism motors wired). */
  public static final boolean kChassisOnly = false;

  public static final int kSetXButton = XboxController.Button.kRightBumper.value;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Reusable zero objects
  public static final Translation2d zeroTranslation2d = new Translation2d();
  public static final Rotation2d zeroRotation2d = new Rotation2d();
  public static final Pose2d zeroPose2d = new Pose2d();
  public static final Pose3d zeroPose3d = new Pose3d();

  public static final double kRobotMassLbs = 112.39;
  public static final double kRobotMassKg = Units.lbsToKilograms(kRobotMassLbs);

  public static enum Mode {
    REAL, SIM, REPLAY
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.46;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPTranslation = 5.0;
    public static final double kPRotation = 5.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kPoseControllerPort = 2;
    public static final double kDeadband = 0.1;
    public static final double kDriveSpeedScale = 1.0;
    public static final double kRotationSpeedScale = 1.0;
    public static final double kTriggerThreshold = 0.2;
  }

  // ==================== Mechanism Constants ====================

  /** Dual NEO shooter with 4" wheels, 1:1 ratio. */
  public static final class ShooterConstants {
    public static final int kLeaderMotorId = 15;
    public static final int kFollowerMotorId = 16;
    public static final double kWheelDiameterInches = 4.0;
    public static final double kGearRatio = 1.0;
    public static final double kMaxSpeedRpm = 5000;
    public static final double kShootingSpeedRpm = 4500;
    public static final double kSpeedToleranceRpm = 100;
    public static final int kCurrentLimitAmps = 40;

    public static final double kP = 0.00936;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.191;
    public static final double kV = 0.11858;
    public static final double kA = 0.0;
  }

  /** Turret: NEO + 40:1 reduction (4:1 gearbox + 200:20 gear). */
  public static final class TurretConstants {
    public static final int kMotorId = 17;
    public static final double kGearRatio = 40.0;

    // Vernier absolute encoders (19t + 21t gears on turret ring, CRT for absolute position).
    // Each encoder is a REV Through Bore Encoder with 6-pin JST → 4-channel PWM breakout:
    //   Red+Black+White  = ABS (absolute duty cycle) → PLUG THIS ONE into DIO on RoboRIO
    //   Red+Black+Blue   = ENC A (quadrature A, not used)
    //   Red+Black+Yellow = ENC B (quadrature B, not used)
    //   Red+Black+Green  = ENC I (index pulse, not used)
    // Encoder A (19t gear) → DIO 0, Encoder B (21t gear) → DIO 1.
    // Calibration: center turret at 0°, run "[Calibration] Turret Vernier" auto, copy offsets here.
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final double kEncoderAOffset = 0.884638122115953;
    public static final double kEncoderBOffset = 0.17972560449314012;

    public static final double kMaxAngleDegrees = 80.0;
    public static final double kMinAngleDegrees = -80.0;
    public static final double kOperatorPresetOffsetDegrees = 0.0;
    public static final int kCurrentLimitAmps = 20;

    public static final double kMaxVelocityDegPerSec = 720.0;
    public static final double kMaxAccelDegPerSecSq = 1000.0;

    // PID (SysId position loop, kD hand-tuned — SysId Kd unreliable due to timestep jitter)
    public static final double kP = 4.0;
    public static final double kI = 0.0;
    public static final double kD = 0.03;
    /*public static final double kP = 5.7975;
    public static final double kI = 0.0;
    public static final double kD = 1.5;*/

    // Feedforward from SysId (radians, 2026-04-01)
    public static final double kS = 0.43; //0.1875; //0.8682;
    public static final double kV = 1.15; //2.2943;
    public static final double kA = 0.33; //0.65936;
    public static final double kG = 0; //0.30435; // Horizontal turret — not actively used
  }

  /** Hood: FIXED angle (no motor for first qualifier). */
  public static final class HoodConstants {
    public static final boolean kHasMotor = false;
    public static final int kMotorId = 19;
    public static final double kGearRatio = 50.0;
    public static final double kMinAngleDegrees = 0.0;
    public static final double kMaxAngleDegrees = 60.0;
    public static final double kFixedShootingAngle = 55.0;
    public static final double kStowedAngleDegrees = 0.0;
    public static final int kCurrentLimitAmps = 20;
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  /** Intake: pivot (5:1×5:1 + 60:18 belt = 83.33:1) + rollers (4:1). */
  public static final class IntakeConstants {
    public static final int kPivotMotorId = 20;
    public static final int kRollerMotorId = 21;
    public static final double kRollerGearRatio = 4.0;
    public static final double kPivotGearRatio = 5.0 * 5.0 * (60.0 / 18.0); // 83.33:1

    public static final double kDeployedAngleDegrees = 154.0;
    public static final double kPivotSoftMinAngleDeg = 0.0;
    public static final double kPivotSoftMaxAngleDeg = 154.0;
    public static final double kPivotHardMinAngleDeg = 0.0;
    public static final double kPivotHardMaxAngleDeg = 155.0;
    public static final double kStowedAngleDegrees = 0.0;

    public static final double kIntakeSpeed = 1.0;
    public static final double kOuttakeSpeed = -0.5;

    public static final int kPivotCurrentLimitAmps = 30;
    public static final int kRollerCurrentLimitAmps = 40;

    public static final double kPivotP = 100.0;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 15.0;
    public static final double kPivotMaxVelocityDegPerSec = 750.0;
    public static final double kPivotMaxAccelDegPerSec2 = 900.0;
    public static final double kPivotClosedLoopRampSec = 0.1;

    public static final double kPivotKs = 0.025;
    public static final double kPivotKg = 0.5;
    public static final double kPivotKv = 1.0;
    public static final double kPivotToleranceDeg = 3.0;
  }

  /** Hopper/indexer: 4:1 gearbox + 1:1 belt. */
  public static final class HopperConstants {
    public static final int kMotorId = 22;
    public static final double kGearRatio = 4.0;
    public static final double kFeedSpeed = 1;
    public static final double kReverseSpeed = -1.0;
    public static final int kCurrentLimitAmps = 40;
  }

  /** Kicker: NEO + 4:1 gearbox + 32:40 gear = 3.2:1. */
  public static final class KickerConstants {
    public static final int kMotorId = 23;
    public static final double kGearRatio = 4.0 * (32.0 / 40.0); // 3.2:1
    public static final double kFeedSpeed = 1.0;
    public static final int kCurrentLimitAmps = 20;
  }

  /** Addressable LED Strip (WS2812B). */
  public static final class LEDConstants {
    public static final int kPwmPort = 9; // Default PWM port on RoboRIO
    public static final int kStripLength = 60; // Adjust to actual physical strip length
  }
}
