// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * Controls for moving a virtual target pose on the field.
 * 
 * <p>The target pose can be used for:
 * <ul>
 *   <li>Autonomous navigation (drive-to-pose)</li>
 *   <li>Testing auto-aim without physical targets</li>
 *   <li>Visualizing paths in AdvantageScope</li>
 * </ul>
 * 
 * <p>Uses a third Xbox controller (port 2) with:
 * <ul>
 *   <li>Left Stick: Adjust X/Y position</li>
 *   <li>Right Stick X: Adjust rotation</li>
 *   <li>Y Button: Reset target to current robot position</li>
 * </ul>
 * 
 * <p>Controls are alliance-relative, so "forward" always pushes the target
 * away from the driver station regardless of alliance.
 */
public class PoseControls {
  
  /** The virtual target pose on the field */
  private static Pose2d targetPose = new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0));

  /** Virtual pose controller */
  private static CommandXboxController controller;

  /** Translation speed when adjusting target (meters per cycle) */
  private static final double TRANSLATION_SPEED = 0.05;
  
  /** Rotation speed when adjusting target (degrees per cycle) */
  private static final double ROTATION_SPEED = 2.0;

  /**
   * Configure pose controls on the given controller.
   *
   * @param port USB port of the controller (usually 2)
   * @param drivetrain The swerve subsystem for updating the field visualization
   */
  public static void configure(int port, SwerveSubsystem drivetrain) {
    controller = new CommandXboxController(port);

    // Update the field visualization with initial pose
    updateFieldPose(drivetrain);

    // Continuous joystick control for target pose adjustment
    // Left stick: X/Y translation, Right stick X: Rotation
    // Using ignoringDisable(true) so it runs even when disabled
    Command poseUpdateCmd = Commands.run(() -> {
      double leftX = controller.getLeftX();
      double leftY = controller.getLeftY();
      double rightX = controller.getRightX();

      // Apply deadband
      if (Math.abs(leftX) < Constants.ControllerConstants.kDeadband)
        leftX = 0;
      if (Math.abs(leftY) < Constants.ControllerConstants.kDeadband)
        leftY = 0;
      if (Math.abs(rightX) < Constants.ControllerConstants.kDeadband)
        rightX = 0;

      // Alliance-relative controls
      boolean isRedAlliance = DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red)
          .orElse(false);

      double allianceMultiplier = isRedAlliance ? -1.0 : 1.0;

      if (leftX != 0 || leftY != 0) {
        adjustTargetTranslation(
            -leftY * TRANSLATION_SPEED * allianceMultiplier,
            -leftX * TRANSLATION_SPEED * allianceMultiplier);
      }
      if (rightX != 0) {
        adjustTargetRotation(-rightX * ROTATION_SPEED);
      }

      updateFieldPose(drivetrain);
    }).ignoringDisable(true).withName("PoseControls.Update");
    CommandScheduler.getInstance().schedule(poseUpdateCmd);

    // Y button: Reset target pose to robot's current position
    controller.y().onTrue(Commands.runOnce(() -> {
      targetPose = drivetrain.getPose();
      updateFieldPose(drivetrain);
    }).ignoringDisable(true));
  }

  /**
   * Adjust the target translation by the given delta values.
   *
   * @param deltaX Change in X position (meters)
   * @param deltaY Change in Y position (meters)
   */
  private static void adjustTargetTranslation(double deltaX, double deltaY) {
    targetPose = new Pose2d(
        targetPose.getX() + deltaX,
        targetPose.getY() + deltaY,
        targetPose.getRotation());
  }

  /**
   * Adjust the target rotation by the given delta.
   *
   * @param deltaDegrees Change in rotation (degrees)
   */
  private static void adjustTargetRotation(double deltaDegrees) {
    targetPose = new Pose2d(
        targetPose.getTranslation(),
        targetPose.getRotation().plus(Rotation2d.fromDegrees(deltaDegrees)));
  }

  /**
   * Update the field visualization with the current target pose.
   *
   * @param drivetrain The swerve subsystem containing the field
   */
  @SuppressWarnings("resource")
  private static void updateFieldPose(SwerveSubsystem drivetrain) {
    drivetrain.getSwerveDrive().field.getObject("targetPose").setPose(targetPose);
  }

  /**
   * Get the current target pose.
   *
   * @return The current target pose
   */
  public static Pose2d getTargetPose() {
    return targetPose;
  }

  /**
   * Get a supplier for the current target pose.
   * Useful for commands that need a dynamic pose reference.
   *
   * @return Supplier that returns the current target pose
   */
  public static Supplier<Pose2d> getTargetPoseSupplier() {
    return () -> targetPose;
  }

  /**
   * Set the target pose directly.
   *
   * @param pose The new target pose
   */
  public static void setTargetPose(Pose2d pose) {
    targetPose = pose;
  }

  /**
   * Set the target pose and update the field visualization.
   *
   * @param pose The new target pose
   * @param drivetrain The swerve subsystem for updating the field
   */
  public static void setTargetPose(Pose2d pose, SwerveSubsystem drivetrain) {
    targetPose = pose;
    updateFieldPose(drivetrain);
  }
}
