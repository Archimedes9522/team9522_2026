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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
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

  /**
   * Hard-reset callback for multi-tag poses seen while disabled.
   * Called with resetOdometry instead of addVisionMeasurement so the gyro-wrong
   * heading is fully overwritten before the match starts. Null = disabled.
   */
  private Consumer<Pose2d> poseResetConsumer = null;

  /** Supplier for current robot heading from odometry (used to reject flipped vision poses) */
  private final Supplier<Rotation2d> headingSupplier;

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
   * @param headingSupplier Supplier for current odometry heading (used to reject flipped vision poses)
   * @param io One or more VisionIO implementations (one per camera)
   */
  public Vision(VisionConsumer consumer, Supplier<Rotation2d> headingSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.headingSupplier = headingSupplier;
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
   * Sets a callback that hard-resets odometry from multi-tag vision poses while the
   * robot is disabled. Wire this to {@code SwerveSubsystem::resetOdometry} so the
   * robot enters the match with the correct heading even if the gyro started wrong.
   */
  public void setPoseResetConsumer(Consumer<Pose2d> consumer) {
    this.poseResetConsumer = consumer;
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
    int totalObservations = 0;
    int totalAccepted = 0;
    int totalRejected = 0;

    // === PROCESS EACH CAMERA ===
    boolean anyCameraConnected = false;
    int totalTagsSeen = 0;
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update the disconnect alert for this camera
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      anyCameraConnected = anyCameraConnected || inputs[cameraIndex].connected;

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
      totalTagsSeen += inputs[cameraIndex].tagIds.length;

      // === PROCESS EACH POSE OBSERVATION ===
      for (var observation : inputs[cameraIndex].poseObservations) {
        
        // === POSE REJECTION FILTERS ===
        // These catch obviously bad pose estimates before they corrupt odometry
        // While disabled with the hard-reset consumer wired, skip the heading
        // filter — the whole point of the hard reset is to fix a wrong heading.
        // Without this, Red-alliance boots at (0,0,180°) and the 45° heading
        // gate rejects every single-tag observation, so the pose never corrects.
        boolean disabledResetActive =
            DriverStation.isDisabled() && poseResetConsumer != null;

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
            || observation.pose().getY() > aprilTagLayout.getFieldWidth()

            // Reject single-tag observations whose heading differs too much from odometry.
            // Single-tag PnP has an inherent 180° ambiguity — the wrong solution sneaks
            // through the ambiguity filter and flips the pose estimator's heading, which
            // inverts field-relative driving for the driver.
            // Skipped while disabled so the hard-reset can correct the initial pose.
            || (!disabledResetActive
                && observation.tagCount() == 1
                && Math.abs(
                    observation.pose().toPose2d().getRotation()
                        .minus(headingSupplier.get()).getDegrees()) > maxHeadingError);

        // Count observations for diagnostic logging
        totalObservations++;

        // Log all poses (for debugging in AdvantageScope)
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          totalRejected++;
        } else {
          robotPosesAccepted.add(observation.pose());
          totalAccepted++;
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


        
        // Apply per-camera trust factors
        // (Some cameras may be more reliable than others)
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // === SEND TO DRIVE SUBSYSTEM ===
        Pose2d pose2d = observation.pose().toPose2d();
        boolean isUnambiguousSingleTag =
            observation.tagCount() == 1 && observation.ambiguity() < 0.1;
        boolean isDisabledHardReset =
            DriverStation.isDisabled()
            && poseResetConsumer != null
            && (observation.tagCount() >= 2 || isUnambiguousSingleTag);
        if (isDisabledHardReset) {
          // While disabled, hard-reset odometry from any reliable observation so the
          // correct heading is established before the match starts. Multi-tag is always
          // accepted; single-tag is only accepted when ambiguity < 0.1 (near-zero
          // chance of the 180° flip that the normal filter guards against).
          poseResetConsumer.accept(pose2d);
        } else {
          consumer.accept(
              pose2d,
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
      }

    // === LOG PER-CAMERA DATA ===
    // These can be viewed in AdvantageScope for debugging
    if (kVerboseVisionLogs) {
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
    }
      
      // Add to combined lists
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

  // === LOG COMBINED DATA ===
  // Summary across all cameras
  if (kVerboseVisionLogs) {
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

    // === DIAGNOSTIC COUNTERS (always on) ===
    Logger.recordOutput("Vision/ObservationsTotal", totalObservations);
    Logger.recordOutput("Vision/ObservationsAccepted", totalAccepted);
    Logger.recordOutput("Vision/ObservationsRejected", totalRejected);

    // === SMARTDASHBOARD STATUS ===
    SmartDashboard.putBoolean("Vision Connected", anyCameraConnected);
    SmartDashboard.putNumber("Vision Tags Seen", totalTagsSeen);
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
