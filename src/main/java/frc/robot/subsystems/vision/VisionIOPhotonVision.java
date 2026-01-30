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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/**
 * PhotonVision implementation of the VisionIO interface.
 * 
 * <p>This class reads AprilTag detections from a PhotonVision camera and
 * converts them into robot pose estimates. It supports both multi-tag
 * (more accurate) and single-tag (fallback) pose estimation.
 * 
 * <p><b>Setup Requirements:</b>
 * <ul>
 *   <li>PhotonVision must be installed on a coprocessor (Raspberry Pi, Orange Pi, etc.)</li>
 *   <li>Camera name in PhotonVision must match the name passed to constructor</li>
 *   <li>AprilTag pipeline must be configured in PhotonVision UI</li>
 *   <li>Network Tables must be accessible (same network as robot)</li>
 * </ul>
 * 
 * @see VisionIO For the interface this implements
 * @see VisionConstants For camera position configuration
 */
public class VisionIOPhotonVision implements VisionIO {
  /** PhotonVision camera object for reading detections via NetworkTables */
  protected final PhotonCamera camera;
  
  /** Transform from robot center to camera - used to calculate robot pose from camera pose */
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision for one camera.
   *
   * @param name The configured name of the camera in PhotonVision (must match exactly!)
   * @param robotToCamera The 3D transform from robot center to camera lens
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  /**
   * Reads the latest camera results and converts them to pose observations.
   * 
   * <p>This method:
   * <ol>
   *   <li>Checks if camera is connected</li>
   *   <li>Reads all unprocessed pipeline results</li>
   *   <li>For each result, calculates robot pose from tag detection(s)</li>
   *   <li>Populates the inputs object for logging and processing</li>
   * </ol>
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Check camera connection status
    inputs.connected = camera.isConnected();

    // Collect all tag IDs and pose observations from this update cycle
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    
    // Process all pipeline results since last call
    // (Camera may run faster than robot loop, so there could be multiple)
    for (var result : camera.getAllUnreadResults()) {
      
      // === UPDATE SIMPLE TARGET OBSERVATION ===
      // Used for basic aiming (e.g., "turn toward the tag")
      if (result.hasTargets()) {
        // Get angle to the "best" target (usually closest/most centered)
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),   // Horizontal angle
                Rotation2d.fromDegrees(result.getBestTarget().getPitch())); // Vertical angle
      } else {
        // No targets - reset to zero
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // === MULTI-TAG POSE ESTIMATION ===
      // If we see 2+ tags, PhotonVision can do a more accurate "multi-tag" solve
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // PhotonVision gives us field-to-camera transform; we need field-to-robot
        // Math: field_to_robot = field_to_camera + camera_to_robot
        //       camera_to_robot = inverse(robot_to_camera)
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average distance to all tags (used for std dev calculation)
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Record which tags we're seeing
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Create the pose observation with all relevant metadata
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),              // When image was captured
                robotPose,                                  // Calculated robot position
                multitagResult.estimatedPose.ambiguity,     // How certain the solve is
                multitagResult.fiducialIDsUsed.size(),      // Number of tags used
                totalTagDistance / result.targets.size(),   // Average tag distance
                PoseObservationType.PHOTONVISION));         // Algorithm type

      // === SINGLE-TAG POSE ESTIMATION (FALLBACK) ===
      // If only one tag is visible, we can still estimate pose but it's less accurate
      } else if (!result.targets.isEmpty()) {
        var target = result.targets.get(0);

        // Look up where this tag is on the field
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          // Calculate robot pose from single tag observation
          // Math: field_to_robot = field_to_tag + tag_to_camera + camera_to_robot
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Record this tag
          tagIds.add((short) target.fiducialId);

          // Create pose observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  target.poseAmbiguity,                       // Single-tag ambiguity
                  1,                                          // Only 1 tag
                  cameraToTarget.getTranslation().getNorm(),  // Distance to tag
                  PoseObservationType.PHOTONVISION));
        }
      }
    }

    // === SAVE RESULTS TO INPUTS ===
    // These will be logged by AdvantageKit and processed by the Vision subsystem
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // Convert tag ID set to array
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (short id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
