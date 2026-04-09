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
    // Currently using SINGLE CAMERA setup: Rubik Pi 3 mounted at back-left.
  
  /** 
   * Name of the primary camera (Rubik Pi 3 with PhotonVision).
   * This MUST match the camera name configured in the PhotonVision web interface.
   * Default PhotonVision name is often "OV9281" or "Arducam_OV9281" - check your setup!
   * 
   * This is the ONLY camera currently in use.
   */
  public static String camera0Name = "back_camera";
  
  /** 
   * Name of the secondary camera (Raspberry Pi 5) - NOT CURRENTLY USED.
   * This is reserved for future expansion if you add a second camera.
   * Currently disabled in RobotContainer (uses empty VisionIO).
   */
  public static String camera1Name = "front_camera";

  // === ROBOT TO CAMERA TRANSFORMS ===
  // These describe where each camera is mounted relative to robot center.
  // Getting these right is CRITICAL for accurate pose estimation!
  //
  // Coordinate system (WPILib standard):
  //   X: positive = forward (toward front of robot)
  //   Y: positive = left (when looking from behind robot)
  //   Z: positive = up (toward ceiling)
  //
  // Rotation order: Roll, Pitch, Yaw
  //   Roll: rotation around X (forward) axis
  //   Pitch: rotation around Y (left) axis - POSITIVE = camera tilted UP
  //   Yaw: rotation around Z (up) axis - 0 = forward, π = backward
  
  /**
   * Primary camera (Rubik Pi 3) - Back-left, facing backward, tilted UP.
   * 
   * Mounting position (measured on 9522 robot):
   * - X: -9.464 in (behind robot center)
   * - Y: +12.855 in (left of center)
   * - Z: 11.919 in (above floor)
   * - Pitch: +21° (tilted up to see elevated AprilTags)
   * - Yaw: π (180°, facing backward)
   */
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(-9.464),   // X: 9.464 inches behind robot center
          Units.inchesToMeters(12.855),   // Y: 12.855 inches LEFT of center (positive = left)
          Units.inchesToMeters(11.919),   // Z: 11.919 inches up from floor
          new Rotation3d(
              0.0,                              // Roll: 0 (camera is level side-to-side)
              Units.degreesToRadians(-21),       // Pitch: +21° (tilted UP 21 degrees)
              Math.PI));                        // Yaw: π (180°, facing backward)

  /**
   * Secondary camera (Raspberry Pi 5) - OPTIONAL, disabled by default.
   * 
   * If you add a second camera later, consider mounting it on the FRONT
   * facing forward to see AprilTags when approaching targets.
   * 
   * Suggested front-mount position:
   * - X: +12 inches (front of robot)
   * - Y: 0 inches (centered)
   * - Z: 14 inches (elevated)
   * - Pitch: -20° (tilted down to see floor-level tags)
   * - Yaw: 0° (facing forward)
   * 
   * CURRENTLY NOT USED - keeping for future expansion.
   * To enable: Update RobotContainer to use VisionIOPhotonVision for camera1.
   */
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(12),  // X: 12 inches FORWARD from robot center
          Units.inchesToMeters(0),   // Y: Centered
          Units.inchesToMeters(14),  // Z: 14 inches up from floor
          new Rotation3d(
              0.0,                              // Roll: 0
              Units.degreesToRadians(-20),      // Pitch: -20° (tilted DOWN)
              0.0));                            // Yaw: 0 (facing FORWARD)

  // === POSE FILTERING THRESHOLDS ===
  // These help filter out bad pose estimates from the cameras
  
  /**
   * Maximum allowable pose ambiguity for single-tag observations.
   * Ambiguity measures how confident PhotonVision is in the pose.
   * Lower = stricter filtering. Range: 0.0 (perfect) to 1.0 (very uncertain)
   */
  public static double maxAmbiguity = 0.15;
  
  /**
   * Maximum allowable Z-coordinate error in meters.
   * The robot should always be on the floor, so Z should be near 0.
   * Large Z values indicate a bad pose estimate.
   */
  public static double maxZError = 0.75;

  /**
   * Maximum heading difference (degrees) between a single-tag vision pose and odometry.
   * Single-tag PnP has 180° ambiguity — reject observations whose heading is too far off.
   */
  public static double maxHeadingError = 45.0;

  // === STANDARD DEVIATION CONFIGURATION ===
  // Standard deviations tell the pose estimator how much to trust vision measurements.
  // Lower values = more trust. These are baselines for 1 tag at 1 meter distance.
  // Actual std devs are scaled based on distance and number of tags seen.
  
  /** Baseline XY position uncertainty in meters (for 1 tag at 1 meter) */
  public static double linearStdDevBaseline = 0.10;
  
  /**
   * Baseline rotation uncertainty in radians (for 1 tag at 1 meter).
   * Balances gyro dominance during fast rotation with vision's ability to
   * correct heading drift. Too high and vision can never fix a wrong heading;
   * too low and latency makes field-relative driving feel sluggish.
   */
  public static double angularStdDevBaseline = 0.7;

    /**
     * Enable verbose per-camera and summary pose logging in Vision.
     * Disable to reduce log size and NT bandwidth when not actively tuning vision.
     */
    public static boolean kVerboseVisionLogs = true;

  /**
   * Per-camera trust multipliers.
   * Higher values = less trust (higher uncertainty).
   * Use this to make the estimator trust some cameras more than others.
   * Example: A camera with a dirty lens might have a factor of 2.0.
   */
  public static double[] cameraStdDevFactors =
      new double[] {
                1.0, // Camera 0 (back) - fully trusted
                1.0  // Camera 1 (front) - fully trusted
      };


}
