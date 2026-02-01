// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.maplesim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.gamepieces.GamePiece;

/**
 * Simulates the HUB scoring target on the field.
 * 
 * <p>The HUB is where robots score FUEL balls for points. Each alliance has one hub
 * that alternates between active and inactive during teleop (25-second phases).
 * 
 * <p>Whether a hub is active can be checked using {@link Arena2026Rebuilt#isActive(boolean)}.
 */
public class RebuiltHub extends Goal {

  /** Blue hub position on the field */
  protected static final Translation3d blueHubPose = new Translation3d(4.5974, 4.034536, 1.5748);
  
  /** Red hub position on the field */
  protected static final Translation3d redHubPose = new Translation3d(11.938, 4.034536, 1.5748);
  
  /** Positions where scored FUEL balls are ejected from the hub */
  protected static final Pose3d[] blueShootPoses = {
      new Pose3d(
          blueHubPose.plus(new Translation3d(0.5969, 0.447675, -0.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(33.75))),
      new Pose3d(
          blueHubPose.plus(new Translation3d(0.5969, 0.149225, -0.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(11.25))),
      new Pose3d(
          blueHubPose.plus(new Translation3d(0.5969, -0.149225, -0.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(11.25))),
      new Pose3d(
          blueHubPose.plus(new Translation3d(0.5969, -0.447675, -0.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(-33.75)))
  };

  /** Radius of the goal opening */
  public static final double GoalRadius = 0.5969;
  
  /** Random number generator for ejection pose selection */
  static final Random rng = new Random();

  /** Red hub ejection poses (mirrored from blue) */
  public static final Pose3d[] redShootPoses = Arrays.stream(blueShootPoses)
      .map(Arena2026Rebuilt::flip)
      .map((Pose3d toRotate) -> toRotate.rotateBy(new Rotation3d(Rotation2d.fromDegrees(180))))
      .toArray(Pose3d[]::new);

  /** NetworkTables publisher for hub pose visualization */
  StructPublisher<Pose3d> posePublisher;
  
  /** Reference to the parent arena */
  protected final Arena2026Rebuilt arena;

  /**
   * Creates a HUB for the specified alliance.
   * 
   * @param arena The parent arena simulation
   * @param isBlue Whether this is the blue alliance hub (false = red)
   */
  public RebuiltHub(Arena2026Rebuilt arena, boolean isBlue) {
    super(
        arena,
        Inches.of(47),  // width
        Inches.of(47),  // depth
        Inches.of(10),  // height tolerance
        "Fuel",
        isBlue ? blueHubPose : redHubPose,
        isBlue,
        false);  // Required 8th parameter in maple-sim 0.4.0-beta

    this.arena = arena;
    
    // Publish hub pose for AdvantageScope visualization
    StructPublisher<Pose3d> HubPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/SmartDashboard/MapleSim/Goals/" + (isBlue ? "BlueHub" : "RedHub"), Pose3d.struct)
        .publish();
    HubPosePublisher.set(new Pose3d(position, new Rotation3d()));
  }

  @Override
  protected boolean checkCollision(GamePiece gamePiece) {
    // Check if game piece is within sphere radius of hub center
    return Math.pow(gamePiece.getPose3d().getX() - position.getX(), 2)
        + Math.pow(gamePiece.getPose3d().getY() - position.getY(), 2)
        + Math.pow(gamePiece.getPose3d().getZ() - position.getZ(), 2) < Math.pow(GoalRadius, 2);
  }

  @Override
  protected void addPoints() {
    // Track scoring statistics
    arena.addValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
    arena.addValueToMatchBreakdown(isBlue, "WastedFuel", arena.isActive(isBlue) ? 0 : 1);
    arena.addToScore(isBlue, arena.isActive(isBlue) ? 1 : 0);

    // Eject the FUEL from a random side of the hub
    Pose3d shootPose = isBlue ? blueShootPoses[rng.nextInt(4)] : redShootPoses[rng.nextInt(4)];

    arena.addPieceWithVariance(
        shootPose.getTranslation().toTranslation2d(),
        new Rotation2d(shootPose.getRotation().getZ()),
        shootPose.getMeasureZ(),
        MetersPerSecond.of(2),
        shootPose.getRotation().getMeasureY(),
        0,      // x variance
        0.02,   // y variance
        15,     // yaw variance
        0.2,    // speed variance
        5);     // pitch variance
  }

  @Override
  public void draw(List<Pose3d> drawList) {
    // Hub doesn't need to be drawn as game pieces
    return;
  }
}
