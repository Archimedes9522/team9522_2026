// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;

/**
 * OperatorControls configures the operator controller bindings.
 * 
 * <p>The operator controls game-specific mechanisms like:
 * <ul>
 *   <li>Shooter/intake</li>
 *   <li>Turret aiming</li>
 *   <li>Hood angle</li>
 *   <li>Feeding system (hopper/kicker)</li>
 * </ul>
 * 
 * <p>Control scheme (Xbox controller - Port 1):
 * <ul>
 *   <li>Right Bumper: Deploy intake and run rollers</li>
 *   <li>Y Button: Shoot (spin up shooter and fire when ready)</li>
 *   <li>X Button: Stop shooting</li>
 *   <li>A Button: Feed all (run hopper and kicker forward)</li>
 *   <li>B Button: Back feed (reverse hopper and kicker)</li>
 *   <li>Left Bumper: Toggle auto-aim (shoot on the move)</li>
 *   <li>D-Pad Up: Turret forward</li>
 *   <li>D-Pad Left: Turret left (90°)</li>
 *   <li>D-Pad Right: Turret right (-90°)</li>
 *   <li>Start: Re-zero intake pivot and turret</li>
 * </ul>
 */
public class OperatorControls {
  
  /** The operator's Xbox controller */
  private static CommandXboxController controller;

  /**
   * Configures the operator controller bindings.
   * Use this version before Superstructure is available.
   * 
   * @param port Controller USB port (usually 1)
   */
  public static void configure(int port) {
    controller = new CommandXboxController(port);
    // No bindings without Superstructure - waiting for full integration
  }
  
  /**
   * Configures operator bindings with drivetrain and Superstructure.
   * This is the main configuration method when mechanisms are available.
   * 
   * @param port Controller USB port
   * @param drivetrain The swerve drive subsystem
   * @param superstructure The Superstructure subsystem (coordinates mechanisms)
   */
  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    controller = new CommandXboxController(port);
    
    // ==================== INITIALIZATION ====================
    
    // Start Button: Re-zero intake pivot and turret encoders
    controller.start()
        .onTrue(superstructure.rezeroIntakePivotAndTurretCommand()
            .ignoringDisable(true));
    
    // ==================== INTAKE CONTROLS ====================
    
    // Right Bumper: Deploy intake and run rollers while held
    controller.rightBumper()
        .whileTrue(superstructure.setIntakeDeployAndRoll()
            .withName("OperatorControls.intakeDeployed"));
    
    // ==================== SHOOTING CONTROLS ====================
    
    // Y Button: Shoot (spins up shooter and fires when ready)
    controller.y()
        .onTrue(superstructure.shootCommand());
    
    // X Button: Stop shooting (cancel shoot sequence)
    controller.x()
        .whileTrue(superstructure.stopShootingCommand());
    
    // ==================== FEEDING CONTROLS ====================
    
    // A Button: Feed all (run hopper and kicker forward)
    // Stops feeding when released
    controller.a()
        .whileTrue(superstructure.feedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));
    
    // B Button: Back feed (reverse hopper and kicker to clear jams)
    // Stops when released
    controller.b()
        .whileTrue(superstructure.backFeedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));
    
    // ==================== TURRET CONTROLS ====================
    
    // D-Pad Up: Turret forward (0°)
    controller.povUp()
        .onTrue(superstructure.setTurretForward()
            .withName("OperatorControls.setTurretForward"));
    
    // D-Pad Left: Turret left (+90°)
    controller.povLeft()
        .onTrue(superstructure.setTurretLeft()
            .withName("OperatorControls.setTurretLeft"));
    
    // D-Pad Right: Turret right (-90°)
    controller.povRight()
        .onTrue(superstructure.setTurretRight()
            .withName("OperatorControls.setTurretRight"));
    
    // ==================== AUTO-AIM ====================
    
    // Left Bumper: Toggle auto-aim (shoot on the move with lead compensation)
    controller.leftBumper()
        .toggleOnTrue(new ShootOnTheMoveCommand(drivetrain, superstructure, 
                () -> superstructure.getAimPoint())
            .ignoringDisable(true)
            .withName("OperatorControls.aimCommand"));
  }
  
  /**
   * Gets the operator controller.
   * @return The CommandXboxController for the operator
   */
  public static CommandXboxController getController() {
    return controller;
  }
}
