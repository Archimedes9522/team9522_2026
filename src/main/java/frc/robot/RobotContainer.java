// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

/**
 * RobotContainer is where we define all subsystems, commands, and button bindings.
 * This follows the "declarative" Command-based paradigm where we declare WHAT the robot
 * should do, not HOW (the subsystems handle the how).
 * 
 * Key concepts:
 * - Subsystems: Hardware abstractions (drive, vision, etc.)
 * - Commands: Actions the robot performs (drive, auto routines, etc.)
 * - Triggers: Button bindings that activate commands
 */
public class RobotContainer {
        // ==================== SUBSYSTEMS ====================
        // Subsystems are created here and persist for the entire robot lifetime.
        // They register themselves with the CommandScheduler automatically.
        
        /** Swerve drive subsystem - handles all driving and odometry */
        public final DriveSubsystem m_robotDrive = new DriveSubsystem();
        
        /**
         * Vision subsystem for AprilTag pose estimation.
         * Runs automatically via CommandScheduler.periodic() - no explicit calls needed.
         * Sends pose updates to DriveSubsystem via the addVisionMeasurement callback.
         */
        @SuppressWarnings("unused") // Field appears unused but runs via CommandScheduler
        private final Vision m_vision;
        
        // ==================== AUTO CHOOSER ====================
        /** Dropdown in SmartDashboard/Shuffleboard to select autonomous routine */
        private final SendableChooser<Command> autoChooser;
        
        // ==================== CONTROLLERS ====================
        /** Driver's Xbox controller for teleop control */
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        /**
         * Constructor - called once when robot code starts.
         * Sets up all subsystems, commands, and button bindings.
         */
        public RobotContainer() {
                // ==================== VISION SETUP ====================
                // Vision uses the IO pattern for AdvantageKit logging and replay support.
                // Different IO implementations are used based on robot mode:
                // - REAL: Actual PhotonVision cameras
                // - SIM: Simulated cameras (or dummy for now)
                // - REPLAY: Dummy IO for log replay analysis
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot - create PhotonVision IO for each camera
                                // The method reference m_robotDrive::addVisionMeasurement connects
                                // vision pose estimates to the drive's pose estimator
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                                        new VisionIOPhotonVision(camera1Name, robotToCamera1));
                                break;

                        case SIM:
                                // Simulation - use empty IO implementations
                                // TODO: Add VisionIOPhotonVisionSim for full simulation support
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        new VisionIO() {},
                                        new VisionIO() {});
                                break;

                        default:
                                // Replay mode - empty IO, data comes from log file
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        new VisionIO() {},
                                        new VisionIO() {});
                                break;
                }

                // ==================== PATHPLANNER NAMED COMMANDS ====================
                // Named commands can be triggered from PathPlanner autonomous paths.
                // In PathPlanner, add an "Event Marker" and use these names.
                NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());
                NamedCommands.registerCommand("zeroHeading", m_robotDrive.zeroHeadingCommand());
                
                // Configure controller button bindings
                configureButtonBindings();
                
                // ==================== AUTO CHOOSER ====================
                // AutoBuilder.buildAutoChooser() automatically finds all .auto files
                // in src/main/deploy/pathplanner/autos/ and creates a dropdown
                autoChooser = AutoBuilder.buildAutoChooser("None");
                SmartDashboard.putData("Auto Chooser", autoChooser);

                // ==================== DEFAULT COMMANDS ====================
                // Default commands run when no other command is using the subsystem.
                // The drive's default command reads joystick inputs for teleop control.
                m_robotDrive.setDefaultCommand(
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                // Left stick Y = forward/backward (negated for correct direction)
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband)
                                                                                * OIConstants.kDriveSpeedFactor,
                                                                // Left stick X = strafe left/right (negated for correct direction)
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband)
                                                                                * OIConstants.kDriveSpeedFactor,
                                                                // Right stick X = rotation (negated for correct direction)
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true), // true = field-relative driving
                                                m_robotDrive));
        }

        /**
         * Configure controller button bindings.
         * Each binding maps a button/trigger to a command.
         * 
         * Common trigger types:
         * - onTrue(): Run once when button is first pressed
         * - whileTrue(): Run continuously while button is held
         * - onFalse(): Run once when button is released
         * - toggleOnTrue(): Toggle command on/off each press
         */
        private void configureButtonBindings() {
                // Left Stick Button (L3) -> Lock wheels in X pattern to prevent pushing
                m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

                // Start Button -> Reset gyro heading (use when robot is facing away from driver)
                m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());
        }

        /**
         * Returns the autonomous command selected in the dashboard.
         * Called by Robot.java at the start of autonomous period.
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
