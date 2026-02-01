// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;
import frc.robot.subsystems.mechanisms.TurretSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;
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
 *   <li>Left Bumper: Slow mode (50% speed while held)</li>
 *   <li>Left Stick Button (L3): Lock wheels in X pattern</li>
 *   <li>Start: Zero gyro heading</li>
 *   <li>Y: Center modules (test mode only)</li>
 *   <li>X: Lock wheels (test mode only)</li>
 * </ul>
 */
public class DriverControls {
  
  /** The driver's Xbox controller */
  private static CommandXboxController controller;
  
  /** YAGSL input stream for smooth joystick handling - full speed */
  private static SwerveInputStream driveInputStream;
  
  /** YAGSL input stream for slow mode - 50% speed */
  private static SwerveInputStream driveInputStreamSlow;
  
  /** Slow mode speed multiplier (0.5 = 50% speed) */
  private static final double SLOW_MODE_SCALE = 0.5;

  /**
   * Configures the driver controller bindings.
   * Call this once from RobotContainer.
   * 
   * @param port Controller USB port (usually 0)
   * @param drivetrain The swerve drive subsystem
   */
  public static void configure(int port, SwerveSubsystem drivetrain) {
    configure(port, drivetrain, null);
  }

  /**
   * Configures the driver controller bindings with Superstructure access.
   * 
   * <p>This overload enables simulation-specific bindings like fireFuel
   * which requires access to the shooter mechanism state.
   * 
   * @param port Controller USB port (usually 0)
   * @param drivetrain The swerve drive subsystem
   * @param superstructure The Superstructure (optional, enables sim features)
   */
  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    controller = new CommandXboxController(port);
    
    // ==================== SWERVE INPUT STREAM (NORMAL SPEED) ====================
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
        // Scale speed (0.0 to 1.0) - full speed
        .scaleTranslation(ControllerConstants.kDriveSpeedScale)
        // Ignore small stick movements below this threshold
        .deadband(ControllerConstants.kDeadband);
    
    // ==================== SWERVE INPUT STREAM (SLOW MODE) ====================
    // Same as normal but with reduced speed for precision movements
    driveInputStreamSlow = SwerveInputStream.of(
            drivetrain.getSwerveDrive(),
            () -> controller.getLeftY() * -1,
            () -> controller.getLeftX() * -1)
        .withControllerRotationAxis(() -> controller.getRightX() * -1)
        .robotRelative(false)
        .allianceRelativeControl(true)
        // Slow mode: 50% of normal speed for precision
        .scaleTranslation(ControllerConstants.kDriveSpeedScale * SLOW_MODE_SCALE)
        .deadband(ControllerConstants.kDeadband);
    
    // ==================== DEFAULT COMMAND ====================
    // The default command runs continuously when no other command uses the subsystem
    drivetrain.setDefaultCommand(
        drivetrain.driveFieldOriented(driveInputStream)
            .withName("DriverControls.defaultDrive"));
    
    // ==================== BUTTON BINDINGS ====================
    configureButtonBindings(drivetrain, superstructure);
  }
  
  /**
   * Configures button bindings for the driver controller.
   */
  private static void configureButtonBindings(SwerveSubsystem drivetrain, Superstructure superstructure) {
    // Left Bumper - Slow mode (50% speed while held)
    // Use for precision movements and alignment
    controller.leftBumper()
        .whileTrue(drivetrain.driveFieldOriented(driveInputStreamSlow)
            .withName("DriverControls.slowModeDrive"));
    
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

    // ==================== SIMULATION BINDINGS ====================
    // These bindings only work in simulation mode with Superstructure
    if (Robot.isSimulation() && superstructure != null) {
      // Back Button - Fire FUEL projectile (10 times per second while held)
      // Shows trajectory in AdvantageScope for testing aim
      controller.back().whileTrue(
          Commands.repeatingSequence(
              fireFuel(drivetrain, superstructure),
              Commands.waitSeconds(0.1))
          .withName("DriverControls.fireFuelRepeating"));
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

  /**
   * Creates a command that spawns a FUEL projectile in MapleSim.
   * 
   * <p>This command reads the current shooter state (turret angle, hood angle, 
   * shooter speed) and creates a physics-simulated projectile that follows a
   * realistic trajectory. The projectile will:
   * <ul>
   *   <li>Show its trajectory in AdvantageScope</li>
   *   <li>Score in the hub if aimed correctly</li>
   *   <li>Become a ground game piece if it misses</li>
   * </ul>
   * 
   * <p>Use this with the shoot command to visualize shots in simulation.
   * 
   * @param drivetrain The swerve drive subsystem (for position/velocity)
   * @param superstructure The superstructure (for turret/hood/shooter state)
   * @return A command that fires a FUEL projectile
   */
  public static Command fireFuel(SwerveSubsystem drivetrain, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      SimulatedArena arena = SimulatedArena.getInstance();

      // Get shooter velocity, use minimum test speed if shooter isn't spun up
      LinearVelocity shooterVelocity = superstructure.getTangentialVelocity();
      LinearVelocity minTestVelocity = MetersPerSecond.of(15.0); // ~3500 RPM equivalent
      if (shooterVelocity.lt(minTestVelocity)) {
        shooterVelocity = minTestVelocity;
      }

      // Create projectile with current shooter parameters
      GamePieceProjectile fuel = new RebuiltFuelOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(
              TurretSubsystem.TURRET_TRANSLATION.getX() * -1,
              TurretSubsystem.TURRET_TRANSLATION.getY()),
          drivetrain.getSwerveDrive().getRobotVelocity(),
          drivetrain.getPose().getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()),
          TurretSubsystem.TURRET_TRANSLATION.getMeasureZ(),
          // 0.5 times because we're applying spin to the fuel as we shoot it
          shooterVelocity.times(0.5),
          superstructure.getHoodAngle());

      // Configure callbacks to visualize the flight trajectory of the projectile
      fuel.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the FUEL will eventually hit the target
          (poses) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              poses.toArray(Pose3d[]::new)),
          // Callback for when the FUEL will miss the target
          (poses) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              poses.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(fuel);
    }).withName("DriverControls.fireFuel");
  }
}
