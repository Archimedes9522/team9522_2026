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
import frc.robot.commands.mechanisms.ShootOnTheMoveCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;

/**
 * Operator controller bindings for mechanism control.
 *
 * <p>RB = intake, Y = shoot, X = stop, A = feed, B = back-feed,
 * LB = toggle auto-aim, LT = auto-RPM, D-pad = turret presets,
 * Right stick X = manual turret, Start = rezero.
 */
public class OperatorControls {

  private static CommandXboxController controller;
  private static boolean autoAimEnabled = false;
  private static double manualTurretTargetDeg = 0.0;

  public static void configure(int port) {
    controller = new CommandXboxController(port);
  }

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    controller = new CommandXboxController(port);

    // Rezero
    controller.start()
        .onTrue(superstructure.rezeroIntakePivotAndTurretCommand().ignoringDisable(true));

    // Intake
    controller.rightBumper()
        .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intake"));

    // Shooting
    controller.y().onTrue(superstructure.shootCommand());
    controller.x().whileTrue(superstructure.stopShootingCommand());

    // Feeding
    controller.a()
        .whileTrue(superstructure.feedAllCommand()
            .finallyDo(() -> CommandScheduler.getInstance().schedule(superstructure.stopFeedingAllCommand()))
            .withName("OperatorControls.feedAll"));

    controller.b()
        .whileTrue(superstructure.backFeedAllCommand()
            .finallyDo(() -> CommandScheduler.getInstance().schedule(superstructure.stopFeedingAllCommand()))
            .withName("OperatorControls.backFeed"));

    // Turret presets
    controller.povUp()
        .onTrue(Commands.runOnce(() -> manualTurretTargetDeg = 0)
            .andThen(superstructure.setTurretForward()).withName("OperatorControls.turretForward"));

    controller.povLeft()
        .onTrue(Commands.runOnce(() -> manualTurretTargetDeg = 45)
            .andThen(superstructure.setTurretLeft()).withName("OperatorControls.turretLeft"));

    controller.povRight()
        .onTrue(Commands.runOnce(() -> manualTurretTargetDeg = -45)
            .andThen(superstructure.setTurretRight()).withName("OperatorControls.turretRight"));

    // Manual turret: rate-based (~2°/cycle = ~100°/sec at 50Hz)
    final double turretRatePerCycle = 2.0;
    Trigger rightStickActive = new Trigger(() ->
        Math.abs(controller.getRightX()) > ControllerConstants.kDeadband);

    rightStickActive.onTrue(Commands.runOnce(() ->
        manualTurretTargetDeg = superstructure.getTurretAngle().in(Degrees)));

    rightStickActive.whileTrue(
        Commands.run(() -> {
          double stickInput = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDeadband);
          manualTurretTargetDeg += stickInput * turretRatePerCycle;
          manualTurretTargetDeg = MathUtil.clamp(
              manualTurretTargetDeg, -TurretConstants.kMaxAngleDegrees, TurretConstants.kMaxAngleDegrees);
          superstructure.turret.setTargetAngle(Degrees.of(manualTurretTargetDeg));
        }, superstructure.turret).withName("OperatorControls.manualTurret"));

    // Stow intake
    controller.povDown()
        .onTrue(superstructure.intake.stow().asProxy().withName("OperatorControls.stowIntake"));

    // Auto-aim toggle
    controller.leftBumper()
        .onTrue(Commands.runOnce(() -> {
          autoAimEnabled = !autoAimEnabled;
          SmartDashboard.putBoolean("Auto-Aim Active", autoAimEnabled);
        }));

    new Trigger(() -> autoAimEnabled)
        .whileTrue(new ShootOnTheMoveCommand(drivetrain, superstructure, superstructure::getAimPoint)
            .ignoringDisable(true).withName("OperatorControls.autoAim"));

    // Auto-RPM fallback (distance-based shooter speed, manual turret)
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
        }, superstructure.shooter).withName("OperatorControls.autoRPM"));
  }

  public static CommandXboxController getController() { return controller; }
}
