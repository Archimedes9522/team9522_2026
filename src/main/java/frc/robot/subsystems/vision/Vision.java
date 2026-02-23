// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem for AprilTag-based pose estimation.
 * 
 * <p>This subsystem processes camera data to estimate the robot's position on the field.
 * It supports multiple cameras, filters out bad observations, and calculates appropriate
 * standard deviations to tell the pose estimator how much to trust each observation.
 * 
 * <p><b>How it works:</b>
 * <ol>
 *   <li>Each robot loop, cameras report AprilTag detections via VisionIO</li>
 *   <li>This subsystem filters out obviously bad poses (too far from field, too high Z, etc.)</li>
 *   <li>Good poses are sent to SwerveSubsystem via the VisionConsumer callback</li>
 *   <li>SwerveSubsystem fuses vision with odometry using its pose estimator</li>
 * </ol>
 * 
 * <p><b>Note:</b> This subsystem appears "unused" in RobotContainer, but it runs
 * automatically because it extends SubsystemBase - the CommandScheduler calls
 * periodic() every robot loop.
 * 
 * @see SwerveSubsystem#addVisionMeasurement For where vision data is consumed
 * @see VisionConstants For tuning parameters
 */
public class Vision extends SubsystemBase {
  
  /** Callback to send accepted vision measurements to the drive subsystem */
  private final VisionConsumer consumer;
  
  /** Array of camera IO interfaces (one per camera) */
  private final VisionIO[] io;
  
  /** Array of input objects for each camera (populated by IO, logged by AdvantageKit) */
  private final VisionIOInputsAutoLogged[] inputs;
  
  /** Alerts that appear in DriverStation when cameras disconnect */
  private final Alert[] disconnectedAlerts;

  /**
   * Creates a new Vision subsystem.
   * 
   * @param consumer Callback function that receives accepted pose observations.
   *                 Typically SwerveSubsystem::addVisionMeasurement
   * @param io One or more VisionIO implementations (one per camera)
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Create an inputs object for each camera
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Create disconnect alerts for each camera
    // These show up in the DriverStation alerts tab when a camera stops responding
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the horizontal angle to the best target for simple aiming.
   * 
   * <p>Use this for commands that just need to point at a target without
   * full pose estimation (e.g., "turn until target is centered").
   *
   * @param cameraIndex Which camera to use (0 = first camera, 1 = second, etc.)
   * @return The horizontal angle to target (positive = target is to the right)
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /**
   * Called every robot loop by the CommandScheduler.
   * 
   * <p>This is where all the vision processing happens:
   * reading cameras, filtering poses, calculating confidence, and sending to drive.
   */
  @Override
  public void periodic() {
    // === READ CAMERA DATA ===
    // Update inputs from each camera and log them with AdvantageKit
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // === PREPARE LOGGING ===
    // Collect poses from all cameras for combined visualization
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // === PROCESS EACH CAMERA ===
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update the disconnect alert for this camera
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Prepare per-camera logging lists
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Log positions of visible AprilTags (for visualization)
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // === PROCESS EACH POSE OBSERVATION ===
      for (var observation : inputs[cameraIndex].poseObservations) {
        
        // === POSE REJECTION FILTERS ===
        // These catch obviously bad pose estimates before they corrupt odometry
        boolean rejectPose =
            // Must see at least one tag
            observation.tagCount() == 0
            
            // Single-tag observations with high ambiguity are unreliable
            // (Multi-tag observations are inherently more accurate)
            || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
            
            // Z coordinate should be near 0 (robot is on the floor)
            // Large Z values indicate a bad solve
            || Math.abs(observation.pose().getZ()) > maxZError

            // Must be within field boundaries
            // Poses outside the field are definitely wrong
            || observation.pose().getX() < 0.0
            || observation.pose().getX() > aprilTagLayout.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Log all poses (for debugging in AdvantageScope)
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip rejected poses
        if (rejectPose) {
          continue;
        }

        // === CALCULATE STANDARD DEVIATIONS ===
        // Standard deviation tells the pose estimator how much to trust this measurement.
        // Higher std dev = less trust, lower std dev = more trust.
        //
        // Formula: stdDev = baseline * (distance² / tagCount)
        // - Further tags are less accurate (distance squared)
        // - More tags are more accurate (divided by tag count)
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        
        // Apply MegaTag 2 multipliers if applicable
        // (MegaTag 2 is more stable for position but has no rotation data)
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        
        // Apply per-camera trust factors
        // (Some cameras may be more reliable than others)
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // === SEND TO DRIVE SUBSYSTEM ===
        // The consumer callback sends this observation to the pose estimator
        consumer.accept(
            observation.pose().toPose2d(), // Convert 3D pose to 2D (X, Y, rotation)
            observation.timestamp(),        // When the image was captured
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)); // [X, Y, theta] std devs
      }

      // === LOG PER-CAMERA DATA ===
      // These can be viewed in AdvantageScope for debugging
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      // Log which tag IDs this camera is seeing
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/VisibleTagIds",
          inputs[cameraIndex].tagIds);
      
      // Add to combined lists
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // === LOG COMBINED DATA ===
    // Summary across all cameras
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  /**
   * Functional interface for consuming vision measurements.
   * 
   * <p>This allows the Vision subsystem to send pose updates without
   * directly depending on SwerveSubsystem (looser coupling).
   * 
   * <p>In RobotContainer, this is typically connected like:
   * <pre>
   *   new Vision(m_robotDrive::addVisionMeasurement, ...)
   * </pre>
   */
  @FunctionalInterface
  public static interface VisionConsumer {
    /**
     * Accepts a vision measurement to update the pose estimator.
     * 
     * @param visionRobotPoseMeters The estimated robot pose from vision
     * @param timestampSeconds When the image was captured (FPGA time)
     * @param visionMeasurementStdDevs Standard deviations [X, Y, theta] indicating confidence
     */
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
