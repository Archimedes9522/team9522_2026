// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

/**
 * RobotContainer is where we define all subsystems, commands, and button bindings.
 * This follows the "declarative" Command-based paradigm where we declare WHAT the robot
 * should do, not HOW (the subsystems handle the how).
 * 
 * <p>Key concepts:
 * <ul>
 *   <li>Subsystems: Hardware abstractions (drive, vision, etc.)</li>
 *   <li>Commands: Actions the robot performs (drive, auto routines, etc.)</li>
 *   <li>Triggers: Button bindings that activate commands</li>
 * </ul>
 * 
 * <p>Controller bindings are handled by the controls package:
 * <ul>
 *   <li>{@link DriverControls} - Driver controller (port 0)</li>
 *   <li>{@link OperatorControls} - Operator controller (port 1)</li>
 * </ul>
 */
public class RobotContainer {
        // ==================== SUBSYSTEMS ====================
        // Subsystems are created here and persist for the entire robot lifetime.
        // They register themselves with the CommandScheduler automatically.
        
        /** YAGSL-based swerve drive subsystem - handles all driving and odometry */
        public final SwerveSubsystem m_robotDrive = new SwerveSubsystem();
        
        /**
         * Vision subsystem for AprilTag pose estimation.
         * Runs automatically via CommandScheduler.periodic() - no explicit calls needed.
         * Sends pose updates to SwerveSubsystem via the addVisionMeasurement callback.
         */
        @SuppressWarnings("unused") // Field appears unused but runs via CommandScheduler
        private final Vision m_vision;
        
        // ==================== AUTO CHOOSER ====================
        /** Dropdown in SmartDashboard/Shuffleboard to select autonomous routine */
        private final SendableChooser<Command> autoChooser;

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
                //
                // Currently using SINGLE CAMERA setup (Rubik Pi 3 on front).
                // To add a second camera later, replace the empty VisionIO() with:
                //   new VisionIOPhotonVision(camera1Name, robotToCamera1)
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot - Rubik Pi 3 camera on front
                                // The method reference m_robotDrive::addVisionMeasurement connects
                                // vision pose estimates to the drive's pose estimator
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                                        new VisionIO() {});  // Second camera disabled - add later if needed
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
                
                // ==================== CONTROLLER BINDINGS ====================
                // Controller bindings are configured in separate classes for organization.
                // This keeps RobotContainer clean and makes bindings easier to find/modify.
                DriverControls.configure(ControllerConstants.kDriverControllerPort, m_robotDrive);
                OperatorControls.configure(ControllerConstants.kOperatorControllerPort);
                
                // ==================== AUTO CHOOSER ====================
                // AutoBuilder.buildAutoChooser() automatically finds all .auto files
                // in src/main/deploy/pathplanner/autos/ and creates a dropdown
                autoChooser = AutoBuilder.buildAutoChooser("None");
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        /**
         * Returns the autonomous command selected in the dashboard.
         * Called by Robot.java at the start of autonomous period.
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
