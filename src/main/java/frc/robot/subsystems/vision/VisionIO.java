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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for vision systems.
 * 
 * <p>This interface follows the AdvantageKit "IO Layer" pattern, which provides:
 * <ul>
 *   <li>Hardware abstraction - swap between real cameras, simulation, and replay</li>
 *   <li>Deterministic replay - all inputs are logged and can be replayed exactly</li>
 *   <li>Clean separation of hardware code from processing logic</li>
 * </ul>
 * 
 * <p>The {@code @AutoLog} annotation automatically generates a {@code VisionIOInputsAutoLogged}
 * class that handles logging all the input values to the AdvantageKit log file.
 * 
 * @see VisionIOPhotonVision For the real hardware implementation
 * @see Vision For the main subsystem that uses this interface
 */
public interface VisionIO {
  
  /**
   * Container for all vision sensor inputs.
   * 
   * <p>The {@code @AutoLog} annotation tells AdvantageKit to generate a subclass
   * that automatically logs all these fields every robot loop. This is essential
   * for deterministic replay - during replay, these values are read from the log
   * instead of from hardware.
   */
  @AutoLog
  public static class VisionIOInputs {
    /** Whether the camera is connected and communicating */
    public boolean connected = false;
    
    /** 
     * The most recent target observation (for simple aiming commands).
     * tx = horizontal angle, ty = vertical angle to target center.
     */
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    
    /** 
     * All pose observations from this camera since last update.
     * Multiple observations can occur if the camera runs faster than the robot loop.
     */
    public PoseObservation[] poseObservations = new PoseObservation[0];
    
    /** IDs of all AprilTags currently visible to this camera */
    public int[] tagIds = new int[0];
  }

  /**
   * Simple target observation for aiming (not full pose estimation).
   * 
   * <p>Use this when you just need to point at a target without knowing
   * your exact field position.
   * 
   * @param tx Horizontal angle to target center (positive = target is to the right)
   * @param ty Vertical angle to target center (positive = target is above center)
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * Full robot pose observation from AprilTag detection.
   * 
   * <p>This contains everything needed to update the robot's pose estimator
   * with a vision measurement, including confidence information.
   * 
   * @param timestamp When this observation was captured (FPGA timestamp in seconds)
   * @param pose The estimated 3D robot pose on the field
   * @param ambiguity How uncertain the pose solution is (0.0 = certain, 1.0 = very uncertain)
   * @param tagCount Number of AprilTags used for this estimate (more tags = more accurate)
   * @param averageTagDistance Average distance to the detected tags in meters
   * @param type The algorithm used to calculate this pose (affects std dev calculation)
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /**
   * Types of pose estimation algorithms.
   * 
   * <p>Different algorithms have different accuracy characteristics, which
   * affects how much we trust each observation.
   */
  public static enum PoseObservationType {
    /** Limelight MegaTag 1 - single tag 3D solve */
    MEGATAG_1,
    
    /** Limelight MegaTag 2 - multi-tag, but no rotation data */
    MEGATAG_2,
    
    /** PhotonVision pose estimation (single or multi-tag) */
    PHOTONVISION
  }

  /**
   * Updates the inputs with the latest sensor readings.
   * 
   * <p>This method is called once per robot loop by the Vision subsystem.
   * Implementations should read from their hardware and populate the inputs object.
   * 
   * @param inputs The inputs object to populate with current sensor values
   */
  public default void updateInputs(VisionIOInputs inputs) {}
}
