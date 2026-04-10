// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;
import frc.robot.subsystems.mechanisms.TurretSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

/**
 * Driver controller bindings using YAGSL SwerveInputStream.
 *
 * <p>Controls: Left Stick = translate, Right Stick X = rotate,
 * LB = slow mode, L3 = lock, Start = zero gyro.
 */
public class DriverControls {

  private static CommandXboxController controller;
  private static SwerveInputStream driveInputStream;
  private static SwerveInputStream driveInputStreamSlow;
  private static final double SLOW_MODE_SCALE = 0.35;

  public static void configure(int port, SwerveSubsystem drivetrain) {
    configure(port, drivetrain, null);
  }

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    controller = new CommandXboxController(port);

    // Normal speed input stream
    driveInputStream = SwerveInputStream.of(
            drivetrain.getSwerveDrive(),
            () -> controller.getLeftY() * -1,
            () -> controller.getLeftX() * -1)
        .withControllerRotationAxis(() -> {
            if (RobotBase.isSimulation()) {
                double rot = controller.getRightX();
                if (Math.abs(rot) < 0.05) rot = controller.getRawAxis(2);
                return rot * -1;
            }
            return controller.getRightX() * -1;
        })
        .robotRelative(false)
        .allianceRelativeControl(true)
        .scaleTranslation(ControllerConstants.kDriveSpeedScale)
        .deadband(ControllerConstants.kDeadband);

    // Slow mode input stream
    driveInputStreamSlow = SwerveInputStream.of(
            drivetrain.getSwerveDrive(),
            () -> controller.getLeftY() * -1,
            () -> controller.getLeftX() * -1)
        .withControllerRotationAxis(() -> {
            if (RobotBase.isSimulation()) {
                double rot = controller.getRightX();
                if (Math.abs(rot) < 0.05) rot = controller.getRawAxis(2);
                return rot * -1;
            }
            return controller.getRightX() * -1;
        })
        .robotRelative(false)
        .allianceRelativeControl(true)
        .scaleTranslation(ControllerConstants.kDriveSpeedScale * SLOW_MODE_SCALE)
        .scaleRotation(SLOW_MODE_SCALE)
        .deadband(ControllerConstants.kDeadband);

    // Default command
    drivetrain.setDefaultCommand(
        drivetrain.driveFieldOriented(driveInputStream)
            .withName("DriverControls.defaultDrive"));

    configureButtonBindings(drivetrain, superstructure);
  }

  private static void configureButtonBindings(SwerveSubsystem drivetrain, Superstructure superstructure) {
    controller.leftBumper()
        .whileTrue(drivetrain.driveFieldOriented(driveInputStreamSlow)
            .withName("DriverControls.slowModeDrive"));

    controller.leftStick()
        .whileTrue(drivetrain.lockCommand());

    controller.start()
        .onTrue(drivetrain.zeroHeadingForAllianceCommand());

    // Pathfinding to driver station corners
    controller.povLeft().whileTrue(
        Commands.defer(() -> {
          var target = FieldConstants.AimPoints.getAllianceStationTopPosition();
          var isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
          var rot = Rotation2d.fromDegrees(isRed ? 0.0 : 180.0);
          return AutoBuilder.pathfindToPose(
              new Pose2d(target.getX(), target.getY(), rot),
              new PathConstraints(3.0, 2.5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.0);
        }, java.util.Set.of(drivetrain))
        .withName("DriverControls.pathfindTopCorner"));

    controller.povRight().whileTrue(
        Commands.defer(() -> {
          var target = FieldConstants.AimPoints.getAllianceStationBottomPosition();
          var isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
          var rot = Rotation2d.fromDegrees(isRed ? 0.0 : 180.0);
          return AutoBuilder.pathfindToPose(
              new Pose2d(target.getX(), target.getY(), rot),
              new PathConstraints(3.0, 2.5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.0);
        }, java.util.Set.of(drivetrain))
        .withName("DriverControls.pathfindBottomCorner"));

    // Test mode bindings
    if (DriverStation.isTest()) {
      controller.y().whileTrue(drivetrain.centerModulesCommand());
      controller.x().whileTrue(drivetrain.lockCommand());
      controller.b().onTrue(drivetrain.zeroHeadingForAllianceCommand());
    }

    // Back button: flip heading 180° (invert alliance controls)
    if (Robot.isReal()) {
      controller.back()
          .onTrue(drivetrain.flipHeadingCommand());
    }

    // Simulation bindings
    if (Robot.isSimulation() && superstructure != null) {
      controller.back().whileTrue(
          Commands.repeatingSequence(fireFuel(drivetrain, superstructure), Commands.waitSeconds(0.1))
          .withName("DriverControls.fireFuelRepeating"));
    }
  }

  public static CommandXboxController getController() { return controller; }
  public static SwerveInputStream getDriveInputStream() { return driveInputStream; }

  /** Spawns a FUEL projectile in MapleSim using current shooter state. */
  public static Command fireFuel(SwerveSubsystem drivetrain, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      SimulatedArena arena = SimulatedArena.getInstance();
      LinearVelocity shooterVelocity = superstructure.getTangentialVelocity();
      Logger.recordOutput("Shooter/ActualTangentialVelocity", shooterVelocity.in(MetersPerSecond));

      // Clamp to minimum before 0.5x spin scaling
      LinearVelocity minVelocityPreScale = MetersPerSecond.of(10.0);
      if (shooterVelocity.lt(minVelocityPreScale)) {
        shooterVelocity = minVelocityPreScale;
        Logger.recordOutput("Shooter/UsingMinimumVelocity", true);
      } else {
        Logger.recordOutput("Shooter/UsingMinimumVelocity", false);
      }

      GamePieceProjectile fuel = new RebuiltFuelOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(
              TurretSubsystem.TURRET_TRANSLATION.getX() * -1,
              TurretSubsystem.TURRET_TRANSLATION.getY()),
          drivetrain.getSwerveDrive().getRobotVelocity(),
          drivetrain.getPose().getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()),
          TurretSubsystem.TURRET_TRANSLATION.getMeasureZ(),
          shooterVelocity.times(0.5),
          superstructure.getHoodAngle());

      fuel.withProjectileTrajectoryDisplayCallBack(
          (poses) -> {
            if (!poses.isEmpty())
              Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot", poses.toArray(Pose3d[]::new));
          },
          (poses) -> {
            if (!poses.isEmpty())
              Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot", poses.toArray(Pose3d[]::new));
          });

      arena.addGamePieceProjectile(fuel);
    }).withName("DriverControls.fireFuel");
  }
}
