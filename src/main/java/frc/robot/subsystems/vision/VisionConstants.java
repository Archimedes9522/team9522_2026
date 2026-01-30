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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * Configuration constants for the vision subsystem.
 * 
 * <p>This file contains camera names, positions, and filtering parameters used
 * for AprilTag-based pose estimation. Tune these values based on your robot's
 * camera setup and desired accuracy vs responsiveness tradeoff.
 * 
 * <p><b>IMPORTANT:</b> Camera names must match EXACTLY what's configured in PhotonVision.
 * Camera positions must be measured accurately for good pose estimation.
 */
public class VisionConstants {
  
  // === FIELD LAYOUT ===
  /**
   * The official AprilTag layout for the 2026 "Rebuilt" game.
   * This tells us where each AprilTag is located on the field, which is
   * needed to calculate our robot's position from tag detections.
   */
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // === CAMERA CONFIGURATION ===
  // Camera names MUST match the names configured in PhotonVision exactly!
  /** Name of the front-facing camera (as configured in PhotonVision) */
  public static String camera0Name = "frontCamera";
  
  /** Name of the back-facing camera (as configured in PhotonVision) */
  public static String camera1Name = "backCamera";

  // === ROBOT TO CAMERA TRANSFORMS ===
  // These describe where each camera is mounted relative to robot center.
  // Getting these right is CRITICAL for accurate pose estimation!
  //
  // Coordinate system:
  //   X: positive = forward (toward front of robot)
  //   Y: positive = left (when looking from behind robot)
  //   Z: positive = up (toward ceiling)
  //   Rotation: positive yaw = counter-clockwise (when viewed from above)
  
  /**
   * Front camera position and orientation relative to robot center.
   * This camera faces forward and is offset slightly to the right.
   */
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(14),  // X: 14 inches forward from robot center
          Units.inchesToMeters(-3),  // Y: 3 inches right of center (negative = right)
          Units.inchesToMeters(7.5), // Z: 7.5 inches up from floor
          new Rotation3d(0.0, 0.0, 0.0)); // Yaw = 0 means facing forward

  /**
   * Back camera position and orientation relative to robot center.
   * This camera faces backward (rotated 180 degrees) and is offset to the left.
   */
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-14), // X: 14 inches backward from robot center
          Units.inchesToMeters(3),   // Y: 3 inches left of center (positive = left)
          Units.inchesToMeters(7.5), // Z: 7.5 inches up from floor
          new Rotation3d(0.0, 0.0, Math.PI)); // Yaw = π (180°) means facing backward

  // === POSE FILTERING THRESHOLDS ===
  // These help filter out bad pose estimates from the cameras
  
  /**
   * Maximum allowable pose ambiguity for single-tag observations.
   * Ambiguity measures how confident PhotonVision is in the pose.
   * Lower = stricter filtering. Range: 0.0 (perfect) to 1.0 (very uncertain)
   */
  public static double maxAmbiguity = 0.3;
  
  /**
   * Maximum allowable Z-coordinate error in meters.
   * The robot should always be on the floor, so Z should be near 0.
   * Large Z values indicate a bad pose estimate.
   */
  public static double maxZError = 0.75;

  // === STANDARD DEVIATION CONFIGURATION ===
  // Standard deviations tell the pose estimator how much to trust vision measurements.
  // Lower values = more trust. These are baselines for 1 tag at 1 meter distance.
  // Actual std devs are scaled based on distance and number of tags seen.
  
  /** Baseline XY position uncertainty in meters (for 1 tag at 1 meter) */
  public static double linearStdDevBaseline = 0.02;
  
  /** Baseline rotation uncertainty in radians (for 1 tag at 1 meter) */
  public static double angularStdDevBaseline = 0.06;

  /**
   * Per-camera trust multipliers.
   * Higher values = less trust (higher uncertainty).
   * Use this to make the estimator trust some cameras more than others.
   * Example: A camera with a dirty lens might have a factor of 2.0.
   */
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0 (front) - fully trusted
        1.0  // Camera 1 (back) - fully trusted
      };

  // === MEGATAG 2 SPECIFIC SETTINGS ===
  // MegaTag 2 is a Limelight-specific mode that uses multiple tags for more stable estimates
  
  /**
   * Linear std dev multiplier for MegaTag 2 observations.
   * MegaTag 2 is more stable, so we trust it more (lower factor).
   */
  public static double linearStdDevMegatag2Factor = 0.5;
  
  /**
   * Angular std dev for MegaTag 2.
   * MegaTag 2 doesn't provide reliable rotation, so we ignore it (infinite uncertainty).
   */
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
