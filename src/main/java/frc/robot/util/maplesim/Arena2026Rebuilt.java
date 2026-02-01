// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.maplesim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import org.dyn4j.dynamics.Settings;
import org.ironmaple.simulation.SimulatedArena;

/**
 * Custom arena simulation for the 2026 FRC game "Rebuilt".
 * 
 * <p>This arena simulates:
 * <ul>
 *   <li>Field obstacles (trench walls, hubs, tower poles)</li>
 *   <li>FUEL game pieces on the field</li>
 *   <li>Hubs for scoring (with 25-second active/inactive phases)</li>
 *   <li>Outposts for human player interaction</li>
 * </ul>
 * 
 * <p>Use with YAGSL's MapleSim integration by calling 
 * {@code arena.addDriveTrainSimulation(swerve.getMapleSimDrive().get())}
 */
public class Arena2026Rebuilt extends SimulatedArena {

  /** Whether the game clock should run (alternating hub activity) */
  protected boolean shouldClock = true;

  /** Current phase clock (counts down from 25 seconds) */
  protected double clock = 0;
  
  /** Whether blue alliance hub is currently active */
  protected boolean blueIsOnClock = Math.random() < 0.5;

  /** NetworkTables publishers for game state */
  protected DoublePublisher phaseClockPublisher = genericInfoTable.getDoubleTopic("Time left in current phase").publish();
  protected BooleanPublisher redActivePublisher = redTable.getBooleanTopic("Red is active").publish();
  protected BooleanPublisher blueActivePublisher = blueTable.getBooleanTopic("Blue is active").publish();

  /** Hub simulations */
  protected RebuiltHub blueHub;
  protected RebuiltHub redHub;

  /** Outpost simulations */
  protected RebuiltOutpost blueOutpost;
  protected RebuiltOutpost redOutpost;

  /** 
   * Efficiency mode reduces game piece count for better performance.
   * MapleSim wasn't designed for 400+ game pieces.
   */
  protected boolean isInEfficiencyMode = true;

  // Field positions for game piece spawning
  protected static Translation2d centerPieceBottomRightCorner = new Translation2d(7.35737, 1.724406);
  protected static Translation2d redDepotBottomRightCorner = new Translation2d(0.02, 5.53);
  protected static Translation2d blueDepotBottomRightCorner = new Translation2d(16.0274, 1.646936);

  /**
   * Field obstacle map for the 2026 competition field.
   */
  public static final class RebuiltFieldObstaclesMap extends FieldMap {
    private static final double FIELD_WIDTH = 16.54;
    private static final double FIELD_HEIGHT = 8.052;

    /**
     * Creates the field obstacle map.
     * 
     * @param addRampCollider Whether to add ramp areas as collision obstacles
     */
    public RebuiltFieldObstaclesMap(boolean addRampCollider) {
      // Field border walls
      super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, FIELD_HEIGHT));
      super.addBorderLine(new Translation2d(FIELD_WIDTH, 0), new Translation2d(FIELD_WIDTH, FIELD_HEIGHT));
      super.addBorderLine(new Translation2d(FIELD_WIDTH, FIELD_HEIGHT), new Translation2d(0, FIELD_HEIGHT));
      super.addBorderLine(new Translation2d(0, 0), new Translation2d(FIELD_WIDTH, 0));

      // Trench walls (4 walls around the center of the field)
      double trenchWallDistX = Inches.of(120.0).in(Meters) + Inches.of(47.0 / 2).in(Meters);
      double trenchWallDistY = Inches.of(73.0).in(Meters)
          + Inches.of(47.0 / 2).in(Meters)
          + Inches.of(6).in(Meters);

      // Bottom-left trench wall
      super.addRectangularObstacle(
          Inches.of(53).in(Meters),
          Inches.of(12).in(Meters),
          new Pose2d(8.27 - trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));
      // Bottom-right trench wall
      super.addRectangularObstacle(
          Inches.of(53).in(Meters),
          Inches.of(12).in(Meters),
          new Pose2d(8.27 + trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));
      // Top-left trench wall
      super.addRectangularObstacle(
          Inches.of(53).in(Meters),
          Inches.of(12).in(Meters),
          new Pose2d(8.27 - trenchWallDistX, 4.035 + trenchWallDistY, Rotation2d.kZero));
      // Top-right trench wall
      super.addRectangularObstacle(
          Inches.of(53).in(Meters),
          Inches.of(12).in(Meters),
          new Pose2d(8.27 + trenchWallDistX, 4.035 + trenchWallDistY, Rotation2d.kZero));

      // Tower poles (blue side and red side)
      super.addRectangularObstacle(
          Inches.of(2).in(Meters),
          Inches.of(47).in(Meters),
          new Pose2d(new Translation2d(Inches.of(42), Inches.of(159)), new Rotation2d()));
      super.addRectangularObstacle(
          Inches.of(2).in(Meters),
          Inches.of(47).in(Meters),
          new Pose2d(new Translation2d(Inches.of(651 - 42), Inches.of(170)), new Rotation2d()));

      // Hub colliders
      if (addRampCollider) {
        // Full hub + ramp collision area (47" x 217")
        super.addRectangularObstacle(
            Inches.of(47).in(Meters),
            Inches.of(217).in(Meters),
            new Pose2d(RebuiltHub.blueHubPose.toTranslation2d(), new Rotation2d()));
        super.addRectangularObstacle(
            Inches.of(47).in(Meters),
            Inches.of(217).in(Meters),
            new Pose2d(RebuiltHub.redHubPose.toTranslation2d(), new Rotation2d()));
      } else {
        // Just the hub collision (47" x 47")
        super.addRectangularObstacle(
            Inches.of(47).in(Meters),
            Inches.of(47).in(Meters),
            new Pose2d(RebuiltHub.blueHubPose.toTranslation2d(), new Rotation2d()));
        super.addRectangularObstacle(
            Inches.of(47).in(Meters),
            Inches.of(47).in(Meters),
            new Pose2d(RebuiltHub.redHubPose.toTranslation2d(), new Rotation2d()));
      }
    }
  }

  /**
   * Creates an Arena for the 2026 FRC game "Rebuilt".
   * 
   * <p>By default, ramps are treated as colliders and efficiency mode is enabled.
   */
  public Arena2026Rebuilt() {
    this(true);
  }

  /**
   * Creates an Arena for the 2026 FRC game "Rebuilt".
   * 
   * @param addRampCollider Whether ramps should be collision obstacles.
   *        Set to false if you're hitting invisible walls - the ramp colliders
   *        extend 217 inches around hubs which may not match AdvantageScope.
   */
  public Arena2026Rebuilt(boolean addRampCollider) {
    super(new RebuiltFieldObstaclesMap(addRampCollider));

    // Configure physics settings (matches CA26)
    Settings settings = physicsWorld.getSettings();
    settings.setMinimumAtRestTime(0.02);
    physicsWorld.setSettings(settings);

    // Create hub simulations
    blueHub = new RebuiltHub(this, true);
    super.addCustomSimulation(blueHub);
    redHub = new RebuiltHub(this, false);
    super.addCustomSimulation(redHub);

    // Create outpost simulations
    blueOutpost = new RebuiltOutpost(this, true);
    super.addCustomSimulation(blueOutpost);
    redOutpost = new RebuiltOutpost(this, false);
    super.addCustomSimulation(redOutpost);
  }

  /**
   * Flips a pose to the opposite alliance side.
   * 
   * @param toFlip The pose to flip
   * @return The mirrored pose
   */
  public static Pose3d flip(Pose3d toFlip) {
    return new Pose3d(
        new Translation3d(
            RebuiltFieldObstaclesMap.FIELD_WIDTH - toFlip.getX(),
            RebuiltFieldObstaclesMap.FIELD_HEIGHT - toFlip.getY(),
            toFlip.getZ()),
        toFlip.getRotation());
  }

  /**
   * Generates a random number within a range centered on 0.
   * 
   * @param variance The maximum deviation from 0
   * @return A random value between -variance and +variance
   */
  protected static double randomInRange(double variance) {
    return (Math.random() - 0.5) * variance;
  }

  /**
   * Adds a FUEL projectile with random variance applied.
   * 
   * @param piecePose Starting position
   * @param yaw Launch direction
   * @param height Initial height
   * @param speed Launch speed
   * @param pitch Launch angle
   * @param xVariance Position variance in X
   * @param yVariance Position variance in Y
   * @param yawVariance Direction variance
   * @param speedVariance Speed variance
   * @param pitchVariance Angle variance
   */
  public void addPieceWithVariance(
      Translation2d piecePose,
      Rotation2d yaw,
      Distance height,
      LinearVelocity speed,
      Angle pitch,
      double xVariance,
      double yVariance,
      double yawVariance,
      double speedVariance,
      double pitchVariance) {
    addGamePieceProjectile(new RebuiltFuelOnFly(
        piecePose.plus(new Translation2d(randomInRange(xVariance), randomInRange(yVariance))),
        new Translation2d(),
        new ChassisSpeeds(),
        yaw.plus(Rotation2d.fromDegrees(randomInRange(yawVariance))),
        height,
        speed.plus(MetersPerSecond.of(randomInRange(speedVariance))),
        Degrees.of(pitch.in(Degrees) + randomInRange(pitchVariance))));
  }

  @Override
  public void placeGamePiecesOnField() {
    blueOutpost.reset();
    redOutpost.reset();

    // Spawn center field game pieces
    for (int x = 0; x < 12; x += 1) {
      for (int y = 0; y < 30; y += isInEfficiencyMode ? 3 : 1) {
        addGamePiece(new RebuiltFuelOnField(centerPieceBottomRightCorner.plus(
            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
      }
    }

    boolean isOnBlue = !DriverStation.getAlliance().isEmpty()
        && DriverStation.getAlliance().get() == Alliance.Blue;

    // Spawn depot game pieces (only for current alliance in efficiency mode)
    if (isOnBlue || !isInEfficiencyMode) {
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 6; y++) {
          addGamePiece(new RebuiltFuelOnField(blueDepotBottomRightCorner.plus(
              new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
        }
      }
    }

    if (!isOnBlue || !isInEfficiencyMode) {
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 6; y++) {
          addGamePiece(new RebuiltFuelOnField(redDepotBottomRightCorner.plus(
              new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
        }
      }
    }

    // Initialize match breakdown tracking
    setupValueForMatchBreakdown("CurrentFuelInOutpost");
    setupValueForMatchBreakdown("TotalFuelInOutpost");
    setupValueForMatchBreakdown("TotalFuelInHub");
    setupValueForMatchBreakdown("WastedFuel");
  }

  @Override
  public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
    List<Pose3d> poses = super.getGamePiecesPosesByType(type);
    blueOutpost.draw(poses);
    redOutpost.draw(poses);
    return poses;
  }

  @Override
  public void simulationSubTick(int tickNum) {
    // Update game clock during teleop
    if (shouldClock && !DriverStation.isAutonomous() && DriverStation.isEnabled()) {
      clock -= getSimulationDt().in(Units.Seconds);

      if (clock <= 0) {
        clock = 25;  // 25-second phases
        blueIsOnClock = !blueIsOnClock;
      }
    } else {
      clock = 25;
    }

    phaseClockPublisher.set(clock);
    super.simulationSubTick(tickNum);

    blueActivePublisher.set(isActive(true));
    redActivePublisher.set(isActive(false));
  }

  /**
   * Returns whether the specified alliance's hub is currently active.
   * 
   * @param isBlue Whether to check the blue alliance hub
   * @return True if the hub is active and scoring counts
   */
  public boolean isActive(boolean isBlue) {
    if (isBlue) {
      return blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
    } else {
      return !blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
    }
  }

  /**
   * Sets whether the game clock should run.
   * 
   * <p>When false, both hubs are always active (useful for testing or endgame).
   * 
   * @param shouldRunClock Whether to run the phase clock
   */
  public void setShouldRunClock(boolean shouldRunClock) {
    shouldClock = shouldRunClock;
  }

  /**
   * Dumps FUEL from the specified outpost onto the field.
   * 
   * @param isBlue Whether to dump from the blue outpost
   */
  public void outpostDump(boolean isBlue) {
    (isBlue ? blueOutpost : redOutpost).dump();
  }

  /**
   * Has the human player throw a FUEL at the hub.
   * 
   * @param isBlue Whether to throw from the blue outpost
   */
  public void outpostThrowForGoal(boolean isBlue) {
    (isBlue ? blueOutpost : redOutpost).throwForGoal();
  }

  /**
   * Throws a FUEL from the outpost with specified parameters.
   * 
   * @param isBlue Whether to throw from the blue outpost
   * @param throwYaw Launch direction
   * @param throwPitch Launch angle
   * @param speed Launch speed
   */
  public void outpostThrow(boolean isBlue, Rotation2d throwYaw, Angle throwPitch, LinearVelocity speed) {
    (isBlue ? blueOutpost : redOutpost).throwFuel(throwYaw, throwPitch, speed);
  }

  /**
   * Sets efficiency mode to reduce game piece count.
   * 
   * <p>Call {@link #resetFieldForAuto()} for changes to take effect.
   * 
   * @param efficiencyMode Whether to enable efficiency mode
   */
  public void setEfficiencyMode(boolean efficiencyMode) {
    isInEfficiencyMode = efficiencyMode;
  }

  /**
   * Returns whether efficiency mode is enabled.
   * 
   * @return True if efficiency mode is on
   */
  public boolean getEfficiencyMode() {
    return isInEfficiencyMode;
  }
}
