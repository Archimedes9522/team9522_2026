package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;

/**
 * Command for dynamic aiming and shooting while the robot is moving.
 * 
 * This command calculates lead compensation based on:
 * - Current robot velocity
 * - Distance to target
 * - Estimated time of flight
 * 
 * It uses interpolation tables to determine:
 * - Required shooter RPM based on distance
 * - Required hood angle based on distance
 * - Time of flight for lead calculation
 */
public class ShootOnTheMoveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final Superstructure superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  /**
   * Creates a new ShootOnTheMoveCommand.
   * 
   * @param drivetrain The swerve drive subsystem for pose and velocity
   * @param superstructure The superstructure containing shooter, turret, hood
   * @param aimPointSupplier Supplier for the 3D target point (e.g., hub center)
   */
  public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure,
      Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;

    // We use the drivetrain to determine linear velocity, but don't require it for
    // control. We will be using the superstructure to control the shooting 
    // mechanism so it's a requirement.
    // addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    super.initialize();

    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = superstructure.getShooterSpeed();

    // Start the dynamic aim command with suppliers that track our calculations
    superstructure.aimDynamicCommand(
        () -> this.latestShootSpeed,
        () -> this.latestTurretAngle,
        () -> this.latestHoodAngle
    ).schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();

    // Get shooter position in field coordinates
    Translation3d shooterLocation = drivetrain.getPose3d().getTranslation()
        .plus(superstructure.getShooterPose().getTranslation());

    // Project positions to ground plane for distance calculation
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

    // Get time of flight from lookup table
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight. If we're stationary, this should be zero. If we're backing up, 
    // this will be "ahead" of the target, etc.
    ChassisSpeeds fieldVelocity = drivetrain.getFieldRelativeSpeeds();
    var correctiveVector = new Translation2d(
        fieldVelocity.vxMetersPerSecond * timeOfFlight, 
        fieldVelocity.vyMetersPerSecond * timeOfFlight
    ).unaryMinus();
    var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

    // Log corrected aim point for visualization
    Logger.recordOutput("FieldSimulation/AimTargetCorrected",
        new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

    var correctedTarget = targetOnGround.plus(correctiveVector);

    // Calculate vector from robot to corrected target
    var vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    
    // Calculate turret angle relative to robot heading
    Rotation2d robotHeading = drivetrain.getRotation();
    Angle calculatedHeading = vectorToTarget.getAngle()
        .rotateBy(robotHeading.unaryMinus())
        .getMeasure();

    // Log debug info
    Logger.recordOutput("ShootOnTheMove/RobotHeading", robotHeading.getDegrees());
    Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading.in(Degrees));
    Logger.recordOutput("ShootOnTheMove/DistanceToTarget", distanceToTarget.in(Meters));
    Logger.recordOutput("ShootOnTheMove/CorrectedDistance", correctedDistance.in(Meters));
    Logger.recordOutput("ShootOnTheMove/TimeOfFlight", timeOfFlight);

    // Update setpoints based on distance
    latestTurretAngle = calculatedHeading;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);

    // Apply setpoints to superstructure
    superstructure.setShooterSetpoints(
        latestShootSpeed,
        latestTurretAngle,
        latestHoodAngle);
  }

  @Override
  public void end(boolean interrupted) {
    // Could optionally stop the shooter here
    super.end(interrupted);
  }

  /**
   * Gets the estimated time of flight for a given distance.
   * @param distanceToTarget Distance to target
   * @return Time of flight in seconds
   */
  private double getFlightTime(Distance distanceToTarget) {
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  /**
   * Calculates required shooter speed for a given distance.
   * @param distanceToTarget Distance to target
   * @return Required shooter RPM
   */
  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    return RPM.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  /**
   * Calculates required hood angle for a given distance.
   * @param distanceToTarget Distance to target
   * @return Required hood angle
   */
  private Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
    return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  // ============================================================================
  // RANGE TABLES - Adjust these based on empirical testing!
  // ============================================================================
  
  // Distance (meters) -> Time of flight (seconds)
  // Start with estimates, tune during practice
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 0.3),    // Close shot - fast
      Map.entry(2.0, 0.5),
      Map.entry(3.0, 0.8),
      Map.entry(4.0, 1.0),
      Map.entry(5.0, 1.3));   // Far shot - slower arc

  // Distance (meters) -> Shooter speed (RPM)
  // Higher RPM for longer distances
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 2000.0),   // Close - lower speed
      Map.entry(2.0, 2700.0),
      Map.entry(3.0, 3000.0),
      Map.entry(4.0, 3300.0),
      Map.entry(5.0, 3750.0),   // Far - higher speed
      Map.entry(6.0, 4200.0));

  // Distance (meters) -> Hood angle (degrees)
  // Higher angle for closer shots (more arc), lower for far shots (flatter trajectory)
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 60.0),   // Close - high angle
      Map.entry(2.0, 50.0),
      Map.entry(3.0, 40.0),
      Map.entry(4.0, 30.0),
      Map.entry(5.0, 25.0),   // Far - flatter shot
      Map.entry(6.0, 20.0));
}
