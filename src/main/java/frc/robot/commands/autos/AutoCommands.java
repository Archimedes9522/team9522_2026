// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.HoodConstants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.commands.mechanisms.ShootOnTheMoveCommand;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;

/**
 * Auto command bank — all reusable PathPlanner named commands in one place.
 * 
 * <p>Named commands are referenced by name inside PathPlanner {@code .auto} and
 * {@code .path} files. Any command string used in a PathPlanner event marker or
 * auto sequence <b>must</b> be registered here (or in RobotContainer) before the
 * auto chooser is built.
 * 
 * <h3>Naming convention</h3>
 * <ul>
 *   <li>camelCase, no spaces (PathPlanner requirement)</li>
 *   <li>Action first: {@code shootClose}, {@code intakeGround}</li>
 *   <li>Timed variants end with duration: {@code shoot1s}, {@code intake2s}</li>
 * </ul>
 * 
 * <h3>Usage in PathPlanner</h3>
 * Place an Event Marker on a path and set its command name to one of the
 * registered names below. Or use the command name in an Auto Sequence step.
 */
public final class AutoCommands {

  private AutoCommands() {} // utility class, no instances

  /**
   * Registers all named commands for PathPlanner autos.
   * Call this once from RobotContainer, after subsystems are created.
   * 
   * @param drivetrain The swerve drivetrain
   * @param superstructure The mechanism superstructure (null-safe: skips mechanism commands)
   */
  public static void registerAll(SwerveSubsystem drivetrain, Superstructure superstructure) {

    // ==================== DRIVE COMMANDS ====================
    NamedCommands.registerCommand("setX", drivetrain.setXCommand());
    NamedCommands.registerCommand("zeroHeading", drivetrain.zeroHeadingCommand());

    // Skip mechanism commands when running chassis-only
    if (superstructure == null) {
      System.out.println("[AutoCommands] Chassis-only mode — mechanism commands NOT registered");
      return;
    }

    // ==================== SHOOTING — STATIC ====================
    // Pre-spin the flywheel to default speed
    NamedCommands.registerCommand("spinUp",
        superstructure.shootCommand()
            .withName("Auto.spinUp"));

    // Stop the shooter flywheel
    NamedCommands.registerCommand("stopShooting",
        superstructure.stopShootingCommand()
            .withName("Auto.stopShooting"));

    // Stop ALL mechanisms (shooter, turret open-loop 0, hood open-loop 0)
    NamedCommands.registerCommand("stopAll",
        superstructure.stopAllCommand()
            .withName("Auto.stopAll"));

    // ==================== SHOOTING — SPECIFIC RPM ====================
    // Close-range shot (~2 m) — low RPM, fast spin-up
    NamedCommands.registerCommand("shootClose",
        superstructure.aimCommand(RPM.of(2700), Degrees.of(0),
            Degrees.of(HoodConstants.kFixedShootingAngle))
            .withName("Auto.shootClose"));

    // Mid-range shot (~3-4 m)
    NamedCommands.registerCommand("shootMid",
        superstructure.aimCommand(RPM.of(3300), Degrees.of(0),
            Degrees.of(HoodConstants.kFixedShootingAngle))
            .withName("Auto.shootMid"));

    // Long-range shot (~5-6 m)
    NamedCommands.registerCommand("shootFar",
        superstructure.aimCommand(RPM.of(4200), Degrees.of(0),
            Degrees.of(HoodConstants.kFixedShootingAngle))
            .withName("Auto.shootFar"));

    // ==================== SHOOTING — AUTO-AIM ====================
    // Enable continuous auto-aim (turret + flywheel track the alliance hub)
    NamedCommands.registerCommand("enableAutoAim",
        new ShootOnTheMoveCommand(drivetrain, superstructure,
            () -> superstructure.getAimPoint())
            .withName("Auto.enableAutoAim"));

    // Disable auto-aim — requires turret/shooter/hood so the scheduler interrupts
    // any running ShootOnTheMoveCommand (which requires the same subsystems).
    NamedCommands.registerCommand("disableAutoAim",
        Commands.runOnce(() -> {}, superstructure.turret, superstructure.shooter, superstructure.hood)
            .withName("Auto.disableAutoAim"));

    // ==================== SHOOTING — AIM AT HUB ====================
    // Aim the turret at the alliance hub and spin up — stationary shot preparation
    NamedCommands.registerCommand("aimAtHub",
        new ShootOnTheMoveCommand(drivetrain, superstructure,
            () -> FieldConstants.AimPoints.getAllianceHubPosition())
            .withName("Auto.aimAtHub"));

    // ==================== TURRET PRESETS ====================
    NamedCommands.registerCommand("turretForward",
        superstructure.setTurretForward()
            .withName("Auto.turretForward"));

    NamedCommands.registerCommand("turretLeft",
        superstructure.setTurretLeft()
            .withName("Auto.turretLeft"));

    NamedCommands.registerCommand("turretRight",
        superstructure.setTurretRight()
            .withName("Auto.turretRight"));

    // ==================== FEEDING ====================
    // Run hopper + kicker to feed balls into the shooter
    NamedCommands.registerCommand("feedAll",
        superstructure.feedAllCommand()
            .withName("Auto.feedAll"));

    // Stop hopper + kicker
    NamedCommands.registerCommand("stopFeeding",
        superstructure.stopFeedingAllCommand()
            .withName("Auto.stopFeeding"));

    // ==================== INTAKE ====================
    // Deploy intake and run rollers (runs while held / until cancelled)
    NamedCommands.registerCommand("intake",
        superstructure.setIntakeDeployAndRoll()
            .withName("Auto.intake"));

    // Eject game pieces through the intake
    NamedCommands.registerCommand("eject",
        superstructure.ejectCommand()
            .withName("Auto.eject"));

    // Back-feed: reverse hopper + eject through intake (unjam)
    NamedCommands.registerCommand("backFeed",
        superstructure.backFeedAllCommand()
            .withName("Auto.backFeed"));

    // ==================== TIMED SEQUENCES ====================
    // These are self-terminating — good for PathPlanner event markers that need
    // a command that starts AND ends on its own.

    // Spin up, wait for ready, feed for 0.5 s, then stop shooter
    NamedCommands.registerCommand("shoot",
        superstructure.shootCommand()
            .alongWith(superstructure.waitUntilReadyCommand()
                .andThen(superstructure.feedAllCommand().withTimeout(0.5)))
            .withTimeout(3.0)
            .andThen(superstructure.stopShootingCommand())
            .withName("Auto.shoot"));

    // Intake for 2 seconds then hold position
    NamedCommands.registerCommand("intake2s",
        superstructure.setIntakeDeployAndRoll()
            .withTimeout(2.0)
            .withName("Auto.intake2s"));

    // Intake for 2 seconds then stow
    NamedCommands.registerCommand("intake2sStow",
        superstructure.setIntakeDeployAndRoll()
            .withTimeout(2.0)
            .andThen(superstructure.intake.stow().asProxy())
            .withName("Auto.intake2sStow"));

    // Intake for 3 seconds then hold position
    NamedCommands.registerCommand("intake3s",
        superstructure.setIntakeDeployAndRoll()
            .withTimeout(3.0)
            .withName("Auto.intake3s"));

    // Intake for 3 seconds then stow
    NamedCommands.registerCommand("intake3sStow",
        superstructure.setIntakeDeployAndRoll()
            .withTimeout(3.0)
            .andThen(superstructure.intake.stow().asProxy())
            .withName("Auto.intake3sStow"));

    // Stow intake immediately
    NamedCommands.registerCommand("intakeStow",
        superstructure.intake.stow().asProxy()
            .withName("Auto.intakeStow"));

    // Feed for 0.5 seconds (fire one ball)
    NamedCommands.registerCommand("feedOnce",
        superstructure.feedAllCommand()
            .withTimeout(0.5)
            .withName("Auto.feedOnce"));

    // Feed for 1.5 seconds (dump all stored balls)
    NamedCommands.registerCommand("feedDump",
        superstructure.feedAllCommand()
            .withTimeout(1.5)
            .withName("Auto.feedDump"));

    // ==================== COMPOSITE — INTAKE-WHILE-DRIVING ====================
    // Intake stays deployed while the path drives; stows when cancelled
    NamedCommands.registerCommand("intakeWhileDriving",
        superstructure.setIntakeDeployAndRoll()
            .withName("Auto.intakeWhileDriving"));

    // ==================== COMPOSITE — AIM + SHOOT + STOP ====================
    // Full shot sequence: auto-aim → wait for ready → feed 1 ball → stop
    NamedCommands.registerCommand("autoShot",
        new ShootOnTheMoveCommand(drivetrain, superstructure,
            () -> superstructure.getAimPoint())
            .raceWith(
                superstructure.waitUntilReadyCommand()
                    .andThen(superstructure.feedAllCommand().withTimeout(1.0)))
            .andThen(superstructure.stopShootingCommand())
            .withTimeout(4.0)
            .withName("Auto.autoShot"));

    // Full preload dump: auto-aim → wait for ready → dump all 8 FUEL → stop
    // Use this at the start of any auto to empty the preloaded hopper
    NamedCommands.registerCommand("autoDumpAll",
        new ShootOnTheMoveCommand(drivetrain, superstructure,
            () -> superstructure.getAimPoint())
            .raceWith(
                superstructure.waitUntilReadyCommand()
                    .andThen(superstructure.feedAllCommand().withTimeout(4.0)))
            .andThen(superstructure.stopShootingCommand())
            .withTimeout(6.0)
            .withName("Auto.autoDumpAll"));

    // Preload shot: feed briefly to fire 1 ball — assumes shooter is already spun up
    NamedCommands.registerCommand("firePreload",
        superstructure.feedAllCommand()
            .withTimeout(0.5)
            .andThen(superstructure.stopFeedingAllCommand())
            .withName("Auto.firePreload"));

    // Dump all 8 preloaded FUEL — feeds for 4 seconds to empty the hopper
    NamedCommands.registerCommand("dumpPreload",
        superstructure.feedAllCommand()
            .withTimeout(4.0)
            .andThen(superstructure.stopFeedingAllCommand())
            .withName("Auto.dumpPreload"));

    // ==================== SIMULATION-ONLY ====================
    if (Robot.isSimulation()) {
      NamedCommands.registerCommand("fireFuel",
          DriverControls.fireFuel(drivetrain, superstructure)
              .withName("Auto.fireFuel"));
      NamedCommands.registerCommand("fireFuelRepeating",
          Commands.repeatingSequence(
              DriverControls.fireFuel(drivetrain, superstructure),
              Commands.waitSeconds(0.1))
              .withName("Auto.fireFuelRepeating"));
    } else {
      NamedCommands.registerCommand("fireFuel",
          superstructure.feedAllCommand().withTimeout(0.5)
              .withName("Auto.fireFuel"));
      NamedCommands.registerCommand("fireFuelRepeating",
          superstructure.feedAllCommand()
              .withName("Auto.fireFuelRepeating"));
    }

    System.out.println("[AutoCommands] All named commands registered");
  }
}
