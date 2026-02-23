package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Superstructure;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * Command that continuously calculates and applies turret/shooter setpoints
 * to track a moving target while the robot drives.
 * 
 * <p>This command directly controls the turret, shooter, and hood subsystems
 * via the Superstructure. The hood uses a FIXED angle (like CA26) and
 * trajectory is controlled purely by varying shooter speed. It runs until cancelled.
 */
public class ShootOnTheMoveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final Superstructure superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure,
      Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;

    // Require the turret, shooter, and hood subsystems so we have exclusive control
    addRequirements(superstructure.turret, superstructure.shooter, superstructure.hood);
  }

  @Override
  public void initialize() {
    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = RPM.of(3000); // Start with a reasonable default speed
    
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

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

    // Get time of flight. We could try to do this analytically but for now it's
    // easier and more realistic
    // to use a simple linear approximation based on empirical data.
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight.
    // If we're stationary, this should be zero. If we're backing up, this will be
    // "ahead" of the target, etc.
    var robotVelocity = drivetrain.getFieldRelativeSpeeds();
    
    // Log velocity for debugging
    Logger.recordOutput("ShootOnTheMove/RobotVelX", robotVelocity.vxMetersPerSecond);
    Logger.recordOutput("ShootOnTheMove/RobotVelY", robotVelocity.vyMetersPerSecond);
    Logger.recordOutput("ShootOnTheMove/TimeOfFlight", timeOfFlight);
    Logger.recordOutput("ShootOnTheMove/DistanceToTarget", distanceToTarget.in(Meters));
    
    // Corrective vector: opposite of where the ball will drift due to robot motion
    // Ball inherits robot velocity, so aim "behind" our motion
    // Separate scale factors for X (forward/back) and Y (left/right) compensation
    double leadCompensationScaleX = 1.0;  // Forward/back toward/away from hub - slight increase
    double leadCompensationScaleY = 0.5;   // Left/right - slight decrease (over-correcting left)
    var correctiveVector = new Translation2d(
        robotVelocity.vxMetersPerSecond * timeOfFlight * leadCompensationScaleX, 
        robotVelocity.vyMetersPerSecond * timeOfFlight * leadCompensationScaleY)
        .unaryMinus();
    
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
    // So we need to add 180° to convert from robot-relative to turret-relative
    var fieldAngleToTarget = vectorToTarget.getAngle();
    var robotHeading = drivetrain.getRotation();
    
    // Robot-relative angle: field angle minus robot heading
    // Use getDegrees() directly and do manual normalization
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
    
    // Clamp turret angle to physical turret soft limits to avoid wrap/flip behavior
    double maxDeg = TurretConstants.kMaxAngleDegrees;
    double clampedDeg = Math.max(-maxDeg, Math.min(maxDeg, turretAngleDeg));
    latestTurretAngle = Degrees.of(clampedDeg);
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    
    // Hood is FIXED at a constant angle (like CA26) - trajectory controlled by flywheel speed
    latestHoodAngle = Degrees.of(HoodConstants.kFixedShootingAngle);

    // Log for debugging
    Logger.recordOutput("ShootOnTheMove/RobotHeadingDeg", robotHeadingDeg);
    Logger.recordOutput("ShootOnTheMove/FieldAngleToTargetDeg", fieldAngleDeg);
    Logger.recordOutput("ShootOnTheMove/RobotRelativeAngleDeg", robotRelativeDeg);
    Logger.recordOutput("ShootOnTheMove/TurretAnglePreClamp", turretAngleDeg);
    Logger.recordOutput("ShootOnTheMove/ClampedTurretAngle", clampedDeg);
    Logger.recordOutput("ShootOnTheMove/DistanceToTarget", correctedDistance);
    Logger.recordOutput("ShootOnTheMove/TargetShooterRPM", latestShootSpeed.in(RPM));
    Logger.recordOutput("ShootOnTheMove/TargetHoodAngle", latestHoodAngle.in(Degrees));

    // DIRECTLY command the subsystems (we have requirements so this is safe)
    superstructure.turret.setTargetAngle(latestTurretAngle);
    superstructure.shooter.setTargetSpeed(latestShootSpeed);
    superstructure.hood.setTargetAngle(latestHoodAngle);  // Move hood to fixed shooting position
    
    // Update setpoints for readiness triggers
    superstructure.setShooterSetpoints(latestShootSpeed, latestTurretAngle, latestHoodAngle);
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("[ShootOnTheMove] Ended (interrupted=" + interrupted + ")");
  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    return RPM.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  // meters, seconds (time of flight lookup for lead compensation)
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 0.4),
      Map.entry(2.0, 0.5),
      Map.entry(3.0, 0.6),
      Map.entry(4.0, 0.7),
      Map.entry(5.0, 0.8),
      Map.entry(6.0, 0.9),
      Map.entry(8.0, 1.0),
      Map.entry(10.0, 1.1),
      Map.entry(12.0, 1.2));

  // meters, RPM (shooter speed lookup - TUNE THESE for fixed hood angle!)
  // With fixed hood at 55° and hub at 72" (1.83m) height, adjust speeds for trajectory
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(2.0, 2500.0),   // Close: lower speed
      Map.entry(3.0, 2800.0),
      Map.entry(4.0, 3200.0),
      Map.entry(5.0, 3600.0),
      Map.entry(6.0, 4000.0),
      Map.entry(8.0, 4500.0),
      Map.entry(10.0, 5000.0),
      Map.entry(12.0, 5500.0)); // Far: higher speed
}
