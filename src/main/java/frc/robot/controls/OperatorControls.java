// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
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
 *   <li>Left Trigger: Auto-RPM only (distance-based shooter speed, turret stays manual)</li>
 *   <li>D-Pad Up: Turret forward</li>
 *   <li>D-Pad Left: Turret left (+45°)</li>
 *   <li>D-Pad Right: Turret right (-45°)</li>
 *   <li>D-Pad Down: Stow intake</li>
 *   <li>Right Stick X: Manual turret aim (stick position = turret angle within ±90°)</li>
 *   <li>Start: Re-zero intake pivot and turret</li>
 * </ul>
 */
public class OperatorControls {
  
  /** The operator's Xbox controller */
  private static CommandXboxController controller;

  /** Tracks whether auto-aim is enabled (toggled by left bumper) */
  private static boolean autoAimEnabled = false;

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
    controller.a()
        .whileTrue(superstructure.feedAllCommand()
            .finallyDo(() -> CommandScheduler.getInstance().schedule(superstructure.stopFeedingAllCommand()))
            .withName("OperatorControls.feedAll"));
    
    // B Button: Back feed (reverse hopper and kicker to clear jams)
    controller.b()
        .whileTrue(superstructure.backFeedAllCommand()
            .finallyDo(() -> CommandScheduler.getInstance().schedule(superstructure.stopFeedingAllCommand()))
            .withName("OperatorControls.backFeed"));
    
    // ==================== TURRET CONTROLS ====================
    
    // D-Pad Up: Turret forward (0°)
    controller.povUp()
        .onTrue(superstructure.setTurretForward()
            .withName("OperatorControls.setTurretForward"));
    
    // D-Pad Left: Turret left (+45°)
    controller.povLeft()
        .onTrue(superstructure.setTurretLeft()
            .withName("OperatorControls.setTurretLeft"));
    
    // D-Pad Right: Turret right (-45°)
    controller.povRight()
        .onTrue(superstructure.setTurretRight()
            .withName("OperatorControls.setTurretRight"));

    // Right Stick X: Manual turret aim — stick position maps to turret angle
    // Pushing stick right → turret rotates right (negative angle)
    // Pushing stick left → turret rotates left (positive angle)
    // Overrides D-pad presets while stick is active; D-pad presets override back when pressed
    Trigger rightStickActive = new Trigger(() ->
        Math.abs(controller.getRightX()) > ControllerConstants.kDeadband);

    rightStickActive.whileTrue(
        superstructure.turret.setAngleDynamic(
            () -> Degrees.of(
                -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDeadband)
                * TurretConstants.kMaxAngleDegrees))
            .withName("OperatorControls.manualTurret"));

    // D-Pad Down: Stow intake
    controller.povDown()
        .onTrue(superstructure.intake.stow().asProxy()
            .withName("OperatorControls.stowIntake"));
    
    // ==================== AUTO-AIM ====================

    // Left Bumper: Toggle auto-aim on/off using a boolean + Trigger.whileTrue.
    // The boolean tracks intended state; whileTrue handles command lifecycle.
    // This avoids issues with toggleOnTrue/isScheduled through WrapperCommand.
    controller.leftBumper()
        .onTrue(Commands.runOnce(() -> {
          autoAimEnabled = !autoAimEnabled;
          SmartDashboard.putBoolean("Auto-Aim Active", autoAimEnabled);
        }));

    new Trigger(() -> autoAimEnabled)
        .whileTrue(new ShootOnTheMoveCommand(drivetrain, superstructure,
            () -> superstructure.getAimPoint())
            .ignoringDisable(true)
            .withName("OperatorControls.aimCommand"));

    // ==================== AUTO-RPM FALLBACK ====================

    // Left Trigger: Distance-based auto-RPM without turret auto-aim
    // Shooter speed auto-adjusts based on distance to aim point.
    // Turret stays under manual right-stick/D-pad control.
    // Use this as a fallback when full auto-aim (left bumper) is broken.
    controller.leftTrigger(ControllerConstants.kTriggerThreshold)
        .whileTrue(Commands.run(() -> {
          Translation3d target = superstructure.getAimPoint();
          double distance = drivetrain.getPose().getTranslation()
              .getDistance(new Translation2d(target.getX(), target.getY()));
          distance = Math.max(2.0, Math.min(12.0, distance));
          double rpm = ShootOnTheMoveCommand.SHOOTING_SPEED_BY_DISTANCE.get(distance);
          superstructure.shooter.setTargetSpeed(RPM.of(rpm));
          SmartDashboard.putNumber("Auto-RPM/DistanceM", distance);
          SmartDashboard.putNumber("Auto-RPM/TargetRPM", rpm);
        }, superstructure.shooter)
        .finallyDo(() -> superstructure.shooter.setTargetSpeed(RPM.of(0)))
        .withName("OperatorControls.autoRPM"));
  }
  
  /**
   * Gets the operator controller.
   * @return The CommandXboxController for the operator
   */
  public static CommandXboxController getController() {
    return controller;
  }
}
