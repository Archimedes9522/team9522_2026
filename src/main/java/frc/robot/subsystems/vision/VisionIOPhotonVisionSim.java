// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Simulated PhotonVision camera implementation for testing without hardware.
 * 
 * <p>This class extends {@link VisionIOPhotonVision} to add simulation support.
 * It uses PhotonLib's simulation classes to generate realistic AprilTag detections
 * based on the simulated robot pose on the field.
 * 
 * <p><b>How it works:</b>
 * <ol>
 *   <li>The {@link VisionSystemSim} maintains a virtual field with AprilTags</li>
 *   <li>The {@link PhotonCameraSim} simulates what the camera would see from the robot's position</li>
 *   <li>Target detections are published to NetworkTables, just like a real camera</li>
 *   <li>The parent class {@link VisionIOPhotonVision} reads these detections normally</li>
 * </ol>
 * 
 * <p><b>PhotonVision vs Limelight (CA26):</b>
 * <ul>
 *   <li>CA26 uses Limelight with MegaTag2, which has a built-in IMU for pose filtering</li>
 *   <li>We use PhotonVision multi-tag pose estimation, which is purely vision-based</li>
 *   <li>Both achieve similar results - stable pose estimates from multiple AprilTags</li>
 *   <li>PhotonVision doesn't have IMU assist, but our pose estimator fuses vision with gyro</li>
 * </ul>
 * 
 * <p><b>Usage:</b> Create this class instead of {@link VisionIOPhotonVision} when in simulation mode.
 * Call {@link #updateSimulation(Pose2d)} every robot loop with the simulated robot pose.
 * 
 * @see VisionIOPhotonVision The real hardware implementation
 * @see VisionSystemSim PhotonLib's field simulation
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

  // === SIMULATION OBJECTS ===
  
  /**
   * Shared vision system simulation.
   * All simulated cameras share this to see the same virtual field.
   * Static because multiple cameras need the same simulation world.
   */
  private static VisionSystemSim visionSim;
  
  /** The simulated camera that generates fake detections */
  private final PhotonCameraSim cameraSim;
  
  /** Supplies the current simulated robot pose for camera positioning */
  private final Supplier<Pose2d> poseSupplier;
  
  /** Robot-to-camera transform for simulation positioning */
  private final Transform3d robotToCamera;

  /**
   * Creates a simulated PhotonVision camera.
   * 
   * @param name Camera name (must match PhotonVision config, same as real camera)
   * @param robotToCamera Transform from robot center to camera position
   * @param poseSupplier Supplier that returns the current simulated robot pose
   */
  public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    // Initialize parent class (creates the PhotonCamera object)
    super(name, robotToCamera);
    
    this.robotToCamera = robotToCamera;
    this.poseSupplier = poseSupplier;
    
    // Create shared vision simulation if it doesn't exist
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      
      // Add the 2026 Rebuilt field AprilTags to the simulation
      // This lets simulated cameras "see" tags when positioned correctly
      visionSim.addAprilTags(aprilTagLayout);
    }
    
    // === CONFIGURE SIMULATED CAMERA PROPERTIES ===
    // These should approximate your real camera's specs
    SimCameraProperties cameraProps = new SimCameraProperties();
    
    // Resolution and FOV (OV9281 global shutter camera specs)
    // Adjust these if using a different camera
    cameraProps.setCalibration(
        1280,                           // Width in pixels
        720,                            // Height in pixels
        Rotation2d.fromDegrees(75));    // Diagonal FOV
    
    // Detection accuracy simulation
    // Average error and standard deviation in pixels for target detection
    cameraProps.setCalibError(0.25, 0.08);
    
    // Frame rate (limited by robot loop rate in simulation)
    cameraProps.setFPS(30);
    
    // Latency simulation
    // Average and std dev of image processing latency in milliseconds
    // PhotonVision typically has 20-50ms latency depending on hardware
    cameraProps.setAvgLatencyMs(35);
    cameraProps.setLatencyStdDevMs(5);
    
    // Create the simulated camera
    cameraSim = new PhotonCameraSim(camera, cameraProps);
    
    // Add camera to the vision simulation
    visionSim.addCamera(cameraSim, robotToCamera);
    
    // Enable camera streams for debugging in Shuffleboard/AdvantageScope
    // These show what the simulated camera "sees"
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    
    // Wireframe drawing is CPU-intensive, enable only if needed for debugging
    // Shows field elements in the camera view
    cameraSim.enableDrawWireframe(false);
  }

  /**
   * Updates inputs and advances the simulation.
   * 
   * <p>This method:
   * <ol>
   *   <li>Updates the simulation with the current robot pose</li>
   *   <li>The simulation calculates what AprilTags are visible</li>
   *   <li>Fake camera results are published to NetworkTables</li>
   *   <li>Parent class reads those results normally via updateInputs()</li>
   * </ol>
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update simulation with current robot pose
    // This recalculates which tags are visible from this position
    visionSim.update(poseSupplier.get());
    
    // Now read the simulated camera results using the parent class
    // The parent class doesn't know it's reading simulated data - 
    // it just sees normal PhotonCamera results
    super.updateInputs(inputs);
  }
  
  /**
   * Gets the vision system simulation for debugging/visualization.
   * 
   * <p>The VisionSystemSim contains a Field2d object that can be displayed
   * in Shuffleboard or AdvantageScope to show simulated camera views.
   * 
   * @return The shared VisionSystemSim instance
   */
  public static VisionSystemSim getVisionSim() {
    return visionSim;
  }
  
  /**
   * Updates the camera position in simulation.
   * 
   * <p>Call this if the camera is mounted on a moving mechanism (like a turret).
   * For fixed cameras, this isn't needed.
   * 
   * @param newRobotToCamera The new robot-to-camera transform
   */
  public void updateCameraPosition(Transform3d newRobotToCamera) {
    visionSim.adjustCamera(cameraSim, newRobotToCamera);
  }
}
