// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * DriverControls configures the driver controller bindings.
 * 
 * <p>This class uses YAGSL's SwerveInputStream for smoother joystick input handling.
 * SwerveInputStream provides:
 * <ul>
 *   <li>Automatic deadband application</li>
 *   <li>Alliance-relative control (field orientation flips for red alliance)</li>
 *   <li>Speed scaling</li>
 *   <li>Heading aim lock capability</li>
 * </ul>
 * 
 * <p>Control scheme (Xbox controller):
 * <ul>
 *   <li>Left Stick: Translation (forward/backward, strafe)</li>
 *   <li>Right Stick X: Rotation</li>
 *   <li>Left Stick Button (L3): Lock wheels in X pattern</li>
 *   <li>Start: Zero gyro heading</li>
 *   <li>Y: Center modules (test mode only)</li>
 *   <li>X: Lock wheels (test mode only)</li>
 * </ul>
 */
public class DriverControls {
  
  /** The driver's Xbox controller */
  private static CommandXboxController controller;
  
  /** YAGSL input stream for smooth joystick handling */
  private static SwerveInputStream driveInputStream;

  /**
   * Configures the driver controller bindings.
   * Call this once from RobotContainer.
   * 
   * @param port Controller USB port (usually 0)
   * @param drivetrain The swerve drive subsystem
   */
  public static void configure(int port, SwerveSubsystem drivetrain) {
    controller = new CommandXboxController(port);
    
    // ==================== SWERVE INPUT STREAM ====================
    // SwerveInputStream wraps joystick inputs with useful features:
    // - Deadband: Ignores small joystick movements (drift)
    // - Scale: Limits max speed for training/precision
    // - Alliance-relative: Automatically flips for red alliance
    driveInputStream = SwerveInputStream.of(
            drivetrain.getSwerveDrive(),
            // Left stick Y = forward/back (negated - up is negative on controller)
            () -> controller.getLeftY() * -1,
            // Left stick X = strafe (negated - left is negative)
            () -> controller.getLeftX() * -1)
        // Right stick X = rotation (negated)
        .withControllerRotationAxis(() -> controller.getRightX() * -1)
        // Field-relative driving (robot moves relative to field, not robot)
        .robotRelative(false)
        // Flip controls for red alliance (driver station on opposite side)
        .allianceRelativeControl(true)
        // Scale speed (0.0 to 1.0) - start low, increase as drivers improve
        .scaleTranslation(ControllerConstants.kDriveSpeedScale)
        // Ignore small stick movements below this threshold
        .deadband(ControllerConstants.kDeadband);
    
    // ==================== DEFAULT COMMAND ====================
    // The default command runs continuously when no other command uses the subsystem
    drivetrain.setDefaultCommand(
        drivetrain.driveFieldOriented(driveInputStream)
            .withName("DriverControls.defaultDrive"));
    
    // ==================== BUTTON BINDINGS ====================
    configureButtonBindings(drivetrain);
  }
  
  /**
   * Configures button bindings for the driver controller.
   */
  private static void configureButtonBindings(SwerveSubsystem drivetrain) {
    // Left Stick Button (L3) - Lock wheels in X pattern
    // Prevents robot from being pushed
    controller.leftStick()
        .whileTrue(drivetrain.lockCommand());
    
    // Start Button - Zero gyro heading
    // Press when robot is facing AWAY from driver station
    controller.start()
        .onTrue(drivetrain.zeroHeadingCommand());
    
    // ==================== TEST MODE BINDINGS ====================
    // These bindings are only active in Test mode (useful for debugging)
    if (DriverStation.isTest()) {
      // Y Button - Center all modules (point forward)
      controller.y()
          .whileTrue(drivetrain.centerModulesCommand());
      
      // X Button - Lock wheels in X pattern
      controller.x()
          .whileTrue(drivetrain.lockCommand());
      
      // B Button - Zero gyro
      controller.b()
          .onTrue(drivetrain.zeroHeadingCommand());
    }
  }
  
  /**
   * Gets the driver controller.
   * @return The CommandXboxController for the driver
   */
  public static CommandXboxController getController() {
    return controller;
  }
  
  /**
   * Gets the drive input stream.
   * @return The SwerveInputStream for advanced input handling
   */
  public static SwerveInputStream getDriveInputStream() {
    return driveInputStream;
  }
}
