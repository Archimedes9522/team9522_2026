// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.maplesim;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.List;
import org.ironmaple.simulation.Goal;

/**
 * Simulates an OUTPOST on the field.
 * 
 * <p>The OUTPOST is where human players can:
 * <ul>
 *   <li>Store FUEL balls scored by robots</li>
 *   <li>Dump stored FUEL onto the field</li>
 *   <li>Throw FUEL at the hub for points</li>
 * </ul>
 * 
 * <p>Each outpost starts with 24 FUEL balls and can accumulate more from robot scoring.
 */
public class RebuiltOutpost extends Goal {

  // Outpost positions on the field
  protected static final Translation3d redOutpostPose = new Translation3d(16.621, 7.403338, 0);
  protected static final Translation3d redLaunchPose = new Translation3d(16, 8.2, 0);
  protected static final Translation3d redDumpPose = new Translation3d(16.421, 7.2, 0);

  protected static final Translation3d blueOutpostPose = new Translation3d(0, 0.665988, 0);
  protected static final Translation3d blueDumpPose = new Translation3d(0.2, 0.665988, 0);
  protected static final Translation3d blueLaunchPose = new Translation3d(0.665988, -0.254, 0);

  // Render positions for visualization
  protected static final Translation3d redRenderPose = new Translation3d(16.640988, 7.7, 0.844502);
  protected static final Translation3d blueRenderPose = new Translation3d(-0.12, 0.325, 0.844502);

  /** NetworkTables publisher for outpost pose */
  StructPublisher<Pose3d> posePublisher;

  /** Reference to the parent arena */
  protected Arena2026Rebuilt arena;

  /**
   * Creates an OUTPOST for the specified alliance.
   * 
   * @param arena The parent arena simulation
   * @param isBlue Whether this is the blue alliance outpost (false = red)
   */
  public RebuiltOutpost(Arena2026Rebuilt arena, boolean isBlue) {
    super(
        arena,
        Centimeters.of(3),   // width (small receiving area)
        Inches.of(21),       // depth
        Centimeters.of(10),  // height tolerance
        "Fuel",
        isBlue ? blueOutpostPose : redOutpostPose,
        isBlue,
        false);  // Additional parameter for maple-sim 0.4.0

    this.arena = arena;
    gamePieceCount = 24;  // Start with 24 FUEL

    // Publish poses for AdvantageScope visualization
    StructPublisher<Pose3d> OutpostPublisher = NetworkTableInstance.getDefault()
        .getStructTopic(
            "/SmartDashboard/MapleSim/Goals/" + (isBlue ? "BlueOutpost" : "RedOutpost"), Pose3d.struct)
        .publish();

    StructPublisher<Pose3d> OutpostThrowPublisher = NetworkTableInstance.getDefault()
        .getStructTopic(
            "/SmartDashboard/MapleSim/Goals/" + (isBlue ? "BlueOutpostThrow" : "RedOutpostThrow"),
            Pose3d.struct)
        .publish();
        
    StructPublisher<Pose3d> OutpostDumpPublisher = NetworkTableInstance.getDefault()
        .getStructTopic(
            "/SmartDashboard/MapleSim/Goals/" + (isBlue ? "BlueOutpostDump" : "RedOutpostDump"),
            Pose3d.struct)
        .publish();

    OutpostPublisher.set(new Pose3d(position, new Rotation3d()));
    OutpostDumpPublisher.set(new Pose3d(isBlue ? blueDumpPose : redDumpPose, new Rotation3d()));
    OutpostThrowPublisher.set(new Pose3d(isBlue ? blueLaunchPose : redLaunchPose, new Rotation3d()));
  }

  @Override
  protected void addPoints() {
    arena.addValueToMatchBreakdown(isBlue, "TotalFuelInOutpost", 1);
    this.gamePieceCount++;
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    super.simulationSubTick(subTickNum);
    arena.replaceValueInMatchBreakDown(isBlue, "CurrentFuelInOutpost", gamePieceCount);
  }

  @Override
  public void draw(List<Pose3d> drawList) {
    // Draw stacked FUEL balls in the outpost
    int count = 0;
    for (int col = 0; col < 5 && count < gamePieceCount; col++) {
      for (int row = 0; row < 5 && count < gamePieceCount; row++) {
        count++;
        if (isBlue) {
          drawList.add(new Pose3d(
              blueRenderPose.plus(
                  new Translation3d(Inches.of(-6 * col), Inches.of(6 * row), Inches.of(1.3 * col))),
              new Rotation3d()));
        } else {
          drawList.add(new Pose3d(
              redRenderPose.plus(
                  new Translation3d(Inches.of(6 * col), Inches.of(-6 * row), Inches.of(1.3 * col))),
              new Rotation3d()));
        }
      }
    }
  }

  /** Resets the outpost's internal FUEL counter to 24. */
  public void reset() {
    gamePieceCount = 24;
  }

  /**
   * Attempts to throw a FUEL at the hub.
   * 
   * <p>This method includes variance to simulate human inconsistency.
   * Success rate is approximately 50%.
   */
  public void throwForGoal() {
    throwFuel(
        isBlue ? Rotation2d.fromDegrees(45) : Rotation2d.fromDegrees(220),
        Degrees.of(75),
        MetersPerSecond.of(11.2));
  }

  /**
   * Dumps all stored FUEL onto the field.
   * 
   * <p>Dumps up to 24 FUEL balls depending on current storage.
   */
  public void dump() {
    for (int i = 0; i < 24 && gamePieceCount > 0; i++) {
      gamePieceCount--;
      this.arena.addPieceWithVariance(
          isBlue ? blueDumpPose.toTranslation2d() : redDumpPose.toTranslation2d(),
          new Rotation2d(),
          Meters.of(1.7),
          isBlue ? MetersPerSecond.of(2) : MetersPerSecond.of(-2),
          Degrees.of(0),
          0,      // x variance
          0.2,    // y variance
          5,      // yaw variance
          0.2,    // speed variance
          5.0);   // pitch variance
    }
  }

  /**
   * Throws a FUEL from the outpost with specified parameters.
   * 
   * <p>Includes variance to simulate human inconsistency.
   * 
   * @param yaw The direction to throw
   * @param pitch The launch angle
   * @param speed The launch speed
   */
  public void throwFuel(Rotation2d yaw, Angle pitch, LinearVelocity speed) {
    if (gamePieceCount > 0) {
      gamePieceCount--;
      arena.addPieceWithVariance(
          isBlue ? blueLaunchPose.toTranslation2d() : redLaunchPose.toTranslation2d(),
          yaw,
          Meters.of(1.7),
          speed,
          pitch,
          0,      // x variance
          0,      // y variance
          15,     // yaw variance
          2,      // speed variance
          5);     // pitch variance
    }
  }
}
