package frc.robot.commands.mechanisms;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Superstructure;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * ShootOnTheMoveCommand continuously calculates and applies turret and shooter setpoints
 * to track a moving target while the robot drives.
 * 
 * <p>Requirements: Turret, Shooter, Hood.
 */
public class ShootOnTheMoveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final Superstructure superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  // === HARDENING CONSTANTS ===
  /** Minimum distance for table lookup (meters). Closer shots clamp to this. */
  private static final double MIN_DISTANCE_M = 1.5;
  /** Maximum distance for table lookup (meters). Farther shots clamp to this. */
  private static final double MAX_DISTANCE_M = 12.0;
  /** Robot speed below which lead compensation is zeroed (m/s). Prevents jitter when nearly stopped. */
  private static final double LEAD_DEADBAND_MPS = 0.1;
  /** Maximum turret angle change per cycle in degrees. Prevents sudden snapping.
   *  At 50Hz: 10°/cycle = 500 deg/sec, matching the YAMS motion profile limit. */
  private static final double MAX_TURRET_SLEW_DEG_PER_CYCLE = 10.0;

  /** Previous turret target for slew limiting */
  private double previousTurretTargetDeg = 0.0;

  /** Live-tunable RPM map, rebuilt from SmartDashboard on each command start */
  private InterpolatingDoubleTreeMap liveRpmMap;

  /** Distance keys for the RPM table (meters) */
  private static final double[] RPM_TABLE_DISTANCES = {2.0, 3.0, 4.0, 4.86, 6.0, 8.0, 10.0, 12.0};

  public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure,
      Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;

    // Publish default RPM values to SmartDashboard for live tuning
    for (double dist : RPM_TABLE_DISTANCES) {
      SmartDashboard.putNumber("ShootRPM/" + dist + "m", SHOOTING_SPEED_BY_DISTANCE.get(dist));
    }
    SmartDashboard.putNumber("ShootRPM/GlobalOffset", 0.0);

    // Require the turret, shooter, and hood subsystems so we have exclusive control
    addRequirements(superstructure.turret, superstructure.shooter, superstructure.hood);
  }

  @Override
  public void initialize() {
    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = RPM.of(3000); // Start with a reasonable default speed
    previousTurretTargetDeg = latestTurretAngle.in(Degrees);
    SmartDashboard.putBoolean("Auto-Aim Active", true);

    // Rebuild RPM map from SmartDashboard values (allows live tuning between shots)
    liveRpmMap = new InterpolatingDoubleTreeMap();
    for (double dist : RPM_TABLE_DISTANCES) {
      double rpm = SmartDashboard.getNumber("ShootRPM/" + dist + "m",
          SHOOTING_SPEED_BY_DISTANCE.get(dist));
      liveRpmMap.put(dist, rpm);
    }

    System.out.println("[ShootOnTheMove] Started - aiming at " + aimPointSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();
    
    // Log target for debugging
    Logger.recordOutput("ShootOnTheMove/AimTarget", new Pose3d(target, Rotation3d.kZero));

    var shooterLocation = drivetrain.getPose3d().getTranslation()
        .plus(superstructure.getShooterPose().getTranslation());

    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    double rawDistanceM = shooterOnGround.getDistance(targetOnGround);
    
    // ── Distance clamping ──
    // Clamp to lookup table range so interpolation stays valid
    double clampedDistanceM = Math.max(MIN_DISTANCE_M, Math.min(MAX_DISTANCE_M, rawDistanceM));
    var distanceToTarget = Meters.of(clampedDistanceM);
    Logger.recordOutput("ShootOnTheMove/RawDistanceM", rawDistanceM);
    Logger.recordOutput("ShootOnTheMove/ClampedDistanceM", clampedDistanceM);
    if (rawDistanceM != clampedDistanceM) {
      Logger.recordOutput("ShootOnTheMove/DistanceClamped", true);
    }

    // Get time of flight for lead compensation
    double timeOfFlight = getFlightTime(distanceToTarget);

    // ── Lead compensation with low-speed deadband ──
    var robotVelocity = drivetrain.getFieldRelativeSpeeds();
    double chassisSpeedMps = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    
    Logger.recordOutput("ShootOnTheMove/RobotVelX", robotVelocity.vxMetersPerSecond);
    Logger.recordOutput("ShootOnTheMove/RobotVelY", robotVelocity.vyMetersPerSecond);
    Logger.recordOutput("ShootOnTheMove/ChassisSpeedMps", chassisSpeedMps);
    Logger.recordOutput("ShootOnTheMove/TimeOfFlight", timeOfFlight);
    
    Translation2d correctiveVector;
    if (chassisSpeedMps < LEAD_DEADBAND_MPS) {
      // Robot is nearly stationary — zero out lead to prevent jitter
      correctiveVector = new Translation2d();
      Logger.recordOutput("ShootOnTheMove/LeadDeadbandActive", true);
    } else {
      // X/Y scale factors were tuned empirically: Y=0.5 accounts for the
      // robot's tendency to overshoot laterally at high speeds.
      double leadCompensationScaleX = 1.0;
      double leadCompensationScaleY = 0.5;
      correctiveVector = new Translation2d(
          robotVelocity.vxMetersPerSecond * timeOfFlight * leadCompensationScaleX,
          robotVelocity.vyMetersPerSecond * timeOfFlight * leadCompensationScaleY)
          .unaryMinus();
      Logger.recordOutput("ShootOnTheMove/LeadDeadbandActive", false);
    }
    
    Logger.recordOutput("ShootOnTheMove/CorrectiveX", correctiveVector.getX());
    Logger.recordOutput("ShootOnTheMove/CorrectiveY", correctiveVector.getY());
    
    var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

    Logger.recordOutput("FieldSimulation/AimTargetCorrected",
        new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

    var correctedTarget = targetOnGround.plus(correctiveVector);

    // Vector from robot to target (target - robot position)
    var vectorToTarget = correctedTarget.minus(drivetrain.getPose().getTranslation());

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    
    // Calculate the field-relative angle to target, then convert to robot-relative
    // The turret is mounted BACKWARDS on the robot (turret 0° = robot rear)
    var fieldAngleToTarget = vectorToTarget.getAngle();
    var robotHeading = drivetrain.getRotation();
    
    double fieldAngleDeg = fieldAngleToTarget.getDegrees();
    double robotHeadingDeg = robotHeading.getDegrees();
    double robotRelativeDeg = fieldAngleDeg - robotHeadingDeg;
    
    // Normalize robot-relative to [-180, 180]
    while (robotRelativeDeg > 180) robotRelativeDeg -= 360;
    while (robotRelativeDeg < -180) robotRelativeDeg += 360;
    
    // Turret is backwards, so turret angle = robot-relative angle + 180°
    double turretAngleDeg = robotRelativeDeg + 180.0;
    
    // Normalize to [-180, 180]
    while (turretAngleDeg > 180) turretAngleDeg -= 360;
    while (turretAngleDeg < -180) turretAngleDeg += 360;
    
    // Clamp turret angle to physical soft limits
    double maxDeg = TurretConstants.kMaxAngleDegrees;
    double clampedDeg = Math.max(-maxDeg, Math.min(maxDeg, turretAngleDeg));
    
    // ── Turret slew limiting ──
    // Prevent sudden snapping by limiting how fast the turret target can change
    double delta = clampedDeg - previousTurretTargetDeg;
    if (Math.abs(delta) > MAX_TURRET_SLEW_DEG_PER_CYCLE) {
      clampedDeg = previousTurretTargetDeg + Math.signum(delta) * MAX_TURRET_SLEW_DEG_PER_CYCLE;
      Logger.recordOutput("ShootOnTheMove/SlewLimited", true);
    } else {
      Logger.recordOutput("ShootOnTheMove/SlewLimited", false);
    }
    previousTurretTargetDeg = clampedDeg;
    
    latestTurretAngle = Degrees.of(clampedDeg);
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    
    // Hood is FIXED at a constant angle — trajectory controlled by flywheel speed
    latestHoodAngle = Degrees.of(HoodConstants.kFixedShootingAngle);

    // ── Error logging ──
    double turretErrorDeg = clampedDeg - superstructure.turret.getRawAngle().in(Degrees);
    double shooterErrorRPM = latestShootSpeed.in(RPM) - superstructure.shooter.getSpeed().in(RPM);
    Logger.recordOutput("ShootOnTheMove/TurretErrorDeg", turretErrorDeg);
    Logger.recordOutput("ShootOnTheMove/ShooterErrorRPM", shooterErrorRPM);
    
    // Log all targets
    Logger.recordOutput("ShootOnTheMove/RobotHeadingDeg", robotHeadingDeg);
    Logger.recordOutput("ShootOnTheMove/FieldAngleToTargetDeg", fieldAngleDeg);
    Logger.recordOutput("ShootOnTheMove/RobotRelativeAngleDeg", robotRelativeDeg);
    Logger.recordOutput("ShootOnTheMove/TurretAnglePreClamp", turretAngleDeg);
    Logger.recordOutput("ShootOnTheMove/ClampedTurretAngle", clampedDeg);
    Logger.recordOutput("ShootOnTheMove/DistanceToTarget", correctedDistance.in(Meters));
    Logger.recordOutput("ShootOnTheMove/TargetShooterRPM", latestShootSpeed.in(RPM));
    Logger.recordOutput("ShootOnTheMove/TargetHoodAngle", latestHoodAngle.in(Degrees));

    // DIRECTLY command the subsystems (we have requirements so this is safe)
    superstructure.turret.setTargetAngle(latestTurretAngle);
    superstructure.shooter.setTargetSpeed(latestShootSpeed);
    superstructure.hood.setTargetAngle(latestHoodAngle);
    
    // Update setpoints for readiness triggers
    superstructure.setShooterSetpoints(latestShootSpeed, latestTurretAngle, latestHoodAngle);
  }
  
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Auto-Aim Active", false);
    System.out.println("[ShootOnTheMove] Ended (interrupted=" + interrupted + ")");
  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    InterpolatingDoubleTreeMap map = (liveRpmMap != null) ? liveRpmMap : SHOOTING_SPEED_BY_DISTANCE;
    double baseRpm = map.get(distanceToTarget.in(Meters));
    double offset = SmartDashboard.getNumber("ShootRPM/GlobalOffset", 0.0);
    return RPM.of(baseRpm + offset);
  }

  // meters, seconds (time of flight lookup for lead compensation)
  // CA26 values: 4.86m=1.5s — extended with linear estimates. Minimum distance is 1.5m (clamped above).
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.5, 1.0),
      Map.entry(2.0, 1.1),
      Map.entry(3.0, 1.2),
      Map.entry(4.0, 1.35),
      Map.entry(4.86, 1.5),
      Map.entry(6.0, 1.65),
      Map.entry(8.0, 1.85),
      Map.entry(10.0, 2.0),
      Map.entry(12.0, 2.2));

  // meters, RPM (shooter speed lookup - TUNE THESE on the real robot!)
  // Based on CA26 values: 2m=2700, 3m=3000, 4m=3300, 4.86m=3750 RPM
  // Extended with interpolated estimates for longer distances
  // With fixed hood at 55°, flywheel speed is the only trajectory control
  public static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.5, 2800.0),   // Close-range — tune on real robot
      Map.entry(2.0, 3000.0),   // Bumped up from 2700 (too low at 55° fixed hood)
      Map.entry(3.0, 3200.0),   // Bumped up from 3000
      Map.entry(4.0, 3300.0),   // CA26 data point
      Map.entry(4.86, 3750.0),  // CA26 data point
      Map.entry(6.0, 4200.0),   // Estimated — needs real tuning
      Map.entry(8.0, 4800.0),   // Estimated — needs real tuning
      Map.entry(10.0, 5200.0),  // Estimated — needs real tuning
      Map.entry(12.0, 5500.0)); // Estimated — needs real tuning
}
