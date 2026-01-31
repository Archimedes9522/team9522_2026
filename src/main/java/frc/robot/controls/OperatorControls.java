// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Import Constants.ControllerConstants when adding mechanism bindings:
// import frc.robot.Constants.ControllerConstants;

/**
 * OperatorControls configures the operator controller bindings.
 * 
 * <p>The operator typically controls game-specific mechanisms like:
 * <ul>
 *   <li>Shooter/intake</li>
 *   <li>Turret aiming</li>
 *   <li>Hood angle</li>
 *   <li>Climbing mechanisms</li>
 * </ul>
 * 
 * <p>This is a placeholder - add bindings as mechanisms are added to the robot.
 * 
 * <p>Control scheme (Xbox controller):
 * <ul>
 *   <li>Right Trigger: Shoot</li>
 *   <li>Left Trigger: Intake</li>
 *   <li>Right Bumper: Outtake/reverse</li>
 *   <li>A Button: Stow mechanisms</li>
 *   <li>B Button: Score low</li>
 *   <li>Y Button: Score high</li>
 *   <li>X Button: Auto-aim</li>
 *   <li>D-Pad: Manual turret control</li>
 * </ul>
 */
public class OperatorControls {
  
  /** The operator's Xbox controller */
  private static CommandXboxController controller;

  /**
   * Configures the operator controller bindings.
   * Call this once from RobotContainer.
   * 
   * @param port Controller USB port (usually 1)
   */
  public static void configure(int port) {
    controller = new CommandXboxController(port);
    
    // ==================== BUTTON BINDINGS ====================
    // Add mechanism control bindings here as you add subsystems
    
    // Example bindings (uncomment and modify when you have mechanisms):
    
    // // Right Trigger - Shoot while held
    // controller.rightTrigger(ControllerConstants.kTriggerThreshold)
    //     .whileTrue(shooter.shootCommand());
    
    // // Left Trigger - Intake while held
    // controller.leftTrigger(ControllerConstants.kTriggerThreshold)
    //     .whileTrue(intake.intakeCommand());
    
    // // A Button - Stow all mechanisms
    // controller.a()
    //     .onTrue(superstructure.stowCommand());
    
    // // X Button - Auto-aim at target
    // controller.x()
    //     .whileTrue(superstructure.autoAimCommand());
    
    // // D-Pad - Manual turret control
    // controller.povLeft()
    //     .whileTrue(turret.manualLeft());
    // controller.povRight()
    //     .whileTrue(turret.manualRight());
  }
  
  /**
   * Configures operator bindings with a Superstructure subsystem.
   * Use this version when you have a Superstructure that coordinates mechanisms.
   * 
   * @param port Controller USB port
   * @param superstructure The Superstructure subsystem (coordinates mechanisms)
   */
  // public static void configure(int port, Superstructure superstructure) {
  //   controller = new CommandXboxController(port);
  //   
  //   // Add Superstructure-based bindings here
  // }
  
  /**
   * Gets the operator controller.
   * @return The CommandXboxController for the operator
   */
  public static CommandXboxController getController() {
    return controller;
  }
}
